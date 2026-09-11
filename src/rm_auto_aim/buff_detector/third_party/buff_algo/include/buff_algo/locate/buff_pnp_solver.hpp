#pragma once

#include <array>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "buff_algo/common/types.hpp"

namespace buff_algo {

// 单次 PnP 解算的候选信息（平面 4 点 PnP 通常存在 2 个解）。
struct BuffPnPCandidate {
    std::array<Pose3f, 2> candidates;
    int selected_index = 0;
    std::array<float, 2> reprojection_errors {0.0f, 0.0f};
};

// 能量机关扇叶 4 关键点 PnP 解算器。
// 使用 IPPE 算法求解平面四点 PnP（存在镜像歧义的两个解），
// 并结合上一帧解算结果选择更连续的一个解。
class BuffPnPSolver {
public:
    void set_camera_matrix(const cv::Mat intrinsic, const cv::Mat distortion);

    // 输入：本帧检测到的能量机关目标（通常只有 0 或 1 个）。
    // 输出：每个目标相对相机坐标系的位姿（已选择较优解）。
    std::vector<Pose3f> solve_pnp(const std::vector<BuffDetection>& buff_detections);

    // 获取上一次 solve_pnp 调用中，每个目标的两个候选解及重投影误差，
    // 可用于自行绘制可视化 marker。
    const std::vector<BuffPnPCandidate>& get_last_candidates() const { return last_pnp_infos_; }

    std::vector<float> get_selected_reprojection_errors() const;

private:
    std::vector<BuffPnPCandidate> last_pnp_infos_;
    bool has_previous_solution_ = false;
    Eigen::Quaternionf previous_rotation_ = Eigen::Quaternionf::Identity();
    Eigen::Vector3f previous_translation_ = Eigen::Vector3f::Zero();

    cv::Mat camera_intrinsic_ = (cv::Mat_<float>(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    cv::Mat camera_distortion_ = (cv::Mat_<float>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

    void solve_pnp_cv(
        const std::vector<BuffDetection>& buff_detections,
        std::vector<std::array<cv::Mat, 2>>& rvecs,
        std::vector<std::array<cv::Mat, 2>>& tvecs,
        std::vector<std::array<float, 2>>& reprojerrs
    ) const;

    void cvcoord_to_tfcoord(
        const std::vector<std::array<cv::Mat, 2>>& rvecs,
        const std::vector<std::array<cv::Mat, 2>>& tvecs,
        std::vector<std::array<Eigen::Quaternionf, 2>>& rotations,
        std::vector<std::array<Eigen::Vector3f, 2>>& translations
    ) const;

    int select_candidate(
        const std::array<Eigen::Quaternionf, 2>& rotations,
        const std::array<Eigen::Vector3f, 2>& translations,
        const std::array<float, 2>& reprojerrs
    );
};

} // namespace buff_algo
