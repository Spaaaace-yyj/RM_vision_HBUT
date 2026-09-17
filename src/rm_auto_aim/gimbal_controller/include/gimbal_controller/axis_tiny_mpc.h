//
// Created by spaaaaace on 2026/7/8.
//

#ifndef BUILD_AXIS_TINY_MPC_H
#define BUILD_AXIS_TINY_MPC_H

#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>

#include <Eigen/Dense>

// TinyMPC 当前 C++ 示例使用这些宏定义问题规模。
// 如果你的 TinyMPC 版本已经不需要这些宏，也可以保留，不影响。
#ifndef NSTATES
#define NSTATES 2
#endif

#ifndef NINPUTS
#define NINPUTS 1
#endif

#ifndef NHORIZON
#define NHORIZON 100
#endif

#include <tinympc/tiny_api.hpp>
#include <tinympc/types.hpp>

class AxisTinyMPC
{
public:
    struct Config
    {
        double dt = 0.01;

        // 100 步，每步 0.01s，共 1s 窗口
        int horizon = NHORIZON;

        // 当前你的控制器先用“未来窗口”，所以建议输出第 1 步。
        // 如果以后你做了“过去 0.5s + 未来 0.5s”的居中窗口，
        // 并且 ref[50] 对应当前时刻，再把 output_index 改成 50。
        int output_index = 1;

        double max_acc = 50.0; // rad/s^2
        double max_vel = 30.0; // rad/s

        double q_pos = 1.0; // 角度误差权重
        double q_vel = 0.05; // 角速度误差权重
        double r_acc = 0.001; // 加速度惩罚

        double rho = 1.0; // ADMM rho
        int max_iter = 30; // TinyMPC ADMM 迭代次数
    };

    struct Output
    {
        double pos = 0.0;
        double vel = 0.0;
        double acc = 0.0;
        bool success = false;
        int iter = 0;
    };

    explicit AxisTinyMPC(const Config& cfg)
    :
    cfg_(cfg)
    {
        setupSolver();
    }

    Output solve(
        double current_pos,
        double current_vel,
        const std::vector<double>& ref_pos_raw)
    {
        Output out;
        out.pos = current_pos;
        out.vel = current_vel;
        out.acc = 0.0;
        out.success = false;

        if (ref_pos_raw.size() < static_cast<size_t>(cfg_.horizon))
        {
            return out;
        }

        std::vector<double> ref_pos = ref_pos_raw;
        unwrapReference(current_pos, ref_pos);

        std::vector<double> ref_vel(cfg_.horizon, 0.0);
        for (int i = 1; i < cfg_.horizon; ++i)
        {
            ref_vel[i] = (ref_pos[i] - ref_pos[i - 1]) / cfg_.dt;
        }
        if (cfg_.horizon >= 2)
        {
            ref_vel[0] = ref_vel[1];
        }

        // 设置参考轨迹 Xref = [angle_ref; speed_ref]
        for (int k = 0; k < cfg_.horizon; ++k)
        {
            solver_->work->Xref(0, k) = static_cast<tinytype>(ref_pos[k]);
            solver_->work->Xref(1, k) = static_cast<tinytype>(ref_vel[k]);
        }

        tinyVector x0 = tinyVector::Zero(NSTATES);
        x0(0) = static_cast<tinytype>(current_pos);
        x0(1) = static_cast<tinytype>(current_vel);

        int status = tiny_set_x0(solver_, x0);
        if (status != 0)
        {
            return out;
        }

        status = tiny_solve(solver_);
        if (status != 0)
        {
            return out;
        }

        int idx = std::clamp(cfg_.output_index, 1, cfg_.horizon - 1);

        // TinyMPC 解出的状态轨迹和输入轨迹。
        // u 的列数是 horizon - 1，所以 acc 用 idx - 1。
        out.pos = static_cast<double>(solver_->work->x(0, idx));
        out.vel = static_cast<double>(solver_->work->x(1, idx));
        out.acc = static_cast<double>(solver_->work->u(0, idx - 1));
        out.iter = solver_->solution->iter;
        out.success = std::isfinite(out.pos) && std::isfinite(out.vel) && std::isfinite(out.acc);

        return out;
    }

private:
    void setupSolver()
    {
        tinyMatrix A = tinyMatrix::Zero(NSTATES, NSTATES);
        tinyMatrix B = tinyMatrix::Zero(NSTATES, NINPUTS);
        tinyVector f = tinyVector::Zero(NSTATES);

        // 状态: [angle, speed]
        // 输入: acc
        //
        // angle_{k+1} = angle_k + speed_k * dt + 0.5 * acc_k * dt^2
        // speed_{k+1} = speed_k + acc_k * dt
        A(0, 0) = 1.0;
        A(0, 1) = cfg_.dt;
        A(1, 0) = 0.0;
        A(1, 1) = 1.0;

        B(0, 0) = 0.5 * cfg_.dt * cfg_.dt;
        B(1, 0) = cfg_.dt;

        tinyMatrix Q = tinyMatrix::Zero(NSTATES, NSTATES);
        tinyMatrix R = tinyMatrix::Zero(NINPUTS, NINPUTS);

        Q(0, 0) = cfg_.q_pos;
        Q(1, 1) = cfg_.q_vel;
        R(0, 0) = cfg_.r_acc;

        int status = tiny_setup(
            &solver_,
            A,
            B,
            f,
            Q,
            R,
            static_cast<tinytype>(cfg_.rho),
            NSTATES,
            NINPUTS,
            cfg_.horizon,
            1);

        if (status != 0)
        {
            throw std::runtime_error("TinyMPC tiny_setup failed");
        }

        solver_->settings->max_iter = cfg_.max_iter;

        // 状态约束：angle 基本不限制，speed 限幅
        tinyMatrix x_min = tinyMatrix::Constant(NSTATES, cfg_.horizon, -1e6);
        tinyMatrix x_max = tinyMatrix::Constant(NSTATES, cfg_.horizon, 1e6);

        for (int k = 0; k < cfg_.horizon; ++k)
        {
            x_min(1, k) = -cfg_.max_vel;
            x_max(1, k) = cfg_.max_vel;
        }

        // 输入约束：acc 限幅
        tinyMatrix u_min = tinyMatrix::Constant(NINPUTS, cfg_.horizon - 1, -cfg_.max_acc);
        tinyMatrix u_max = tinyMatrix::Constant(NINPUTS, cfg_.horizon - 1, cfg_.max_acc);

        status = tiny_set_bound_constraints(solver_, x_min, x_max, u_min, u_max);

        if (status != 0)
        {
            throw std::runtime_error("TinyMPC set_bound_constraints failed");
        }
    }

    static double normalizeAngle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    static void unwrapReference(double current_pos, std::vector<double>& ref)
    {
        if (ref.empty())
        {
            return;
        }

        ref[0] = current_pos + normalizeAngle(ref[0] - current_pos);

        for (size_t i = 1; i < ref.size(); ++i)
        {
            ref[i] = ref[i - 1] + normalizeAngle(ref[i] - ref[i - 1]);
        }
    }

private:
    Config cfg_;
    TinySolver* solver_ = nullptr;
};

#endif //BUILD_AXIS_TINY_MPC_H
