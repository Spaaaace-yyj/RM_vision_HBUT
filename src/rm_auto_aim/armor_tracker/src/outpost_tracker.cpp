// 前哨站（outpost）专用跟踪器实现
//
// 三块装甲板绕同一竖直轴旋转、圆心在地面投影重合、半径相同、高度各不相同。
// 状态 11 维：[xc, vx, yc, vy, za, vz, yaw, v_yaw, r, h1, h2]
//   h1/h2 = 板1/板2 相对板0 的高度差（板0 的高度即 za）
// 观测 4 维：[azimuth, pitch, distance, armor_yaw]（与普通 EKF 相同）

#include "armor_tracker/outpost_tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace rm_auto_aim
{

namespace
{
constexpr int kStateDim = 11;

/// EKF 坐标系 xyz -> ypd = [azimuth, pitch, distance]（与普通 EKF 保持一致）
Eigen::Vector3d outpostXyzToYpd(const Eigen::Vector3d & xyz)
{
  const double x = xyz.x();
  const double y = xyz.y();
  const double z = xyz.z();
  const double xy_norm = std::max(std::hypot(x, y), 1e-6);
  const double distance = std::max(xyz.norm(), 1e-6);
  return Eigen::Vector3d(std::atan2(y, x), std::atan2(z, xy_norm), distance);
}
}  // namespace

OutpostTracker::OutpostTracker(
  double max_match_distance, double max_match_yaw_diff, int tracking_thres, int lost_thres)
: target_state(Eigen::VectorXd::Zero(kStateDim)),
  max_match_distance_(max_match_distance),
  max_match_yaw_diff_(max_match_yaw_diff),
  tracking_thres_(tracking_thres),
  lost_thres_(lost_thres)
{
}

void OutpostTracker::setProcessNoise(
  double sigma2_q_xyz, double sigma2_q_yaw, double dt, double initial_v_yaw)
{
  s2qxyz_ = sigma2_q_xyz;
  s2qyaw_ = sigma2_q_yaw;
  initial_v_yaw_ = initial_v_yaw;
  if (dt > 1e-6) {
    dt_ = dt;
  }
}

void OutpostTracker::init(const Armors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {
    return;
  }

  // 取离图像中心最近的那块板作为初始化观测
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors.front();
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  tracked_id = tracked_armor.number;
  initEKF(tracked_armor);
  tracker_state = Tracker::DETECTING;
}

void OutpostTracker::initEKF(const Armor & a)
{
  const double xa = a.pose.position.x;
  const double ya = a.pose.position.y;
  const double za = a.pose.position.z;

  // yaw 初值不能用 PnP 解出的板姿态：前哨站任一时刻只可能看到"正对相机"
  // 的那块板，而板面正对相机时绕法线的旋转（yaw）在几何上不可观测，
  // PnP 会输出随机值（实测在 -87°~-35° 之间乱跳）。yaw 初值错 50°，
  // gc 按它算出的板位置也错 50°，云台直接转到看不见目标的地方。
  //
  // 但"能完整看到的那块板一定在圆心靠近相机的一侧"这个几何关系是确定的，
  // 所以用板位置的方位角作为 yaw 初值：从板指向圆心的方向 = 远离相机的方向。
  const double yaw = std::atan2(ya, xa);
  last_yaw_ = yaw;

  // 由单块板反推旋转中心：板在圆周上，圆心 = 板位置 + r * (cos/sin)
  const double r = kRadius;
  const double xc = xa + r * std::cos(yaw);
  const double yc = ya + r * std::sin(yaw);

  // 初始认为观测到的这块是板0（高度基准），另外两块按固定高度差建模；
  // 由于三块板高度固定且已知，h1/h2 直接给标称值，靠后续观测微调。
  target_state = Eigen::VectorXd::Zero(kStateDim);
  target_state << xc, 0.0, yc, 0.0, za, 0.0, yaw, initial_v_yaw_, r, kH1, kH2;

  // 初始协方差（对角）
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(kStateDim, kStateDim);
  P0.diagonal() << 1.0, 10.0, 1.0, 10.0,    // xc, vx, yc, vy
    0.25, 1.0,                              // za, vz
    1.0, 25.0,                              // yaw, v_yaw
    0.01, 0.0025, 0.0025;                   // r, h1, h2

  // 11D CV 模型，过程噪声形式与四板 Tracker 完全一致。
  // 之前这里用默认构造的 EKF（Q 未设置），导致圆心估计抖动极大
  // （实测 x 在 2.1~4.7 m 间跳、yaw 抖 ±12°），云台按错误的 yaw 追板就越追越偏。
  const double dt = dt_;
  auto f = [dt](const Eigen::VectorXd & x) -> Eigen::VectorXd {
      Eigen::VectorXd x_new = x;
      x_new(0) += x(1) * dt;
      x_new(2) += x(3) * dt;
      x_new(4) += x(5) * dt;
      x_new(6) = limitRad(x(6) + x(7) * dt);
      return x_new;
    };

  auto j_f = [dt](const Eigen::VectorXd &) -> Eigen::MatrixXd {
      Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kStateDim, kStateDim);
      F(0, 1) = dt;
      F(2, 3) = dt;
      F(4, 5) = dt;
      F(6, 7) = dt;
      return F;
    };

  // h / j_h 是占位，实际观测函数在 updateOneArmor 里按板 id 传入
  auto h = [](const Eigen::VectorXd &) -> Eigen::VectorXd {
      return Eigen::VectorXd::Zero(4);
    };
  auto j_h = [](const Eigen::VectorXd &) -> Eigen::MatrixXd {
      return Eigen::MatrixXd::Zero(4, kStateDim);
    };

  auto u_q = [this]() -> Eigen::MatrixXd {
      const double t = dt_;
      // 关键：前哨站是固定在地面上的，圆心位置（xc/yc）基本不动，
      // 而四板靶标装在车体上、圆心会随车移动。
      // 如果用四板那样大的圆心过程噪声，EKF 会用"平移圆心"来解释板的位置
      // 变化（实测圆心 x 在 2.6~4.0 之间抖），而不是让 yaw 转起来，
      // 结果 v_yaw 永远学不到、yaw 斜率只有 0.02 rad/s（真实 1.0）。
      // 这里把圆心/高度的过程噪声压到极小，EKF 只能靠"转 yaw"来拟合观测。
      const double v1 = 0.01;
      const double v2 = s2qyaw_;
      const double a = std::pow(t, 4) / 4.0;
      const double b = std::pow(t, 3) / 2.0;
      const double c = std::pow(t, 2);

      Eigen::MatrixXd q = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
      auto fill_cv_noise = [&](int pos, int vel, double var) {
        q(pos, pos) = a * var;
        q(pos, vel) = b * var;
        q(vel, pos) = b * var;
        q(vel, vel) = c * var;
      };
      fill_cv_noise(0, 1, v1);  // xc / vx
      fill_cv_noise(2, 3, v1);  // yc / vy
      fill_cv_noise(4, 5, v1);  // za / vz
      fill_cv_noise(6, 7, v2);  // yaw / v_yaw
      // 前哨站转速是固定的已知量，v_yaw 几乎不允许漂移（否则角度会乱飘）
      q(7, 7) = 1e-6;
      // r / h1 / h2 是固定几何量，过程噪声为 0，靠观测修正
      return q;
    };

  auto u_r = [](const Eigen::VectorXd &) -> Eigen::MatrixXd {
      return Eigen::MatrixXd::Identity(4, 4);
    };

  ekf_ = ExtendedKalmanFilter(f, h, j_f, j_h, u_q, u_r, P0);
  ekf_.setState(target_state, P0);
}

void OutpostTracker::update(const Armors::SharedPtr & armors_msg)
{
  // 预测（predict 返回预测后的状态）
  target_state = ekf_.predict();

  bool matched = false;
  int same_id_count = 0;

  for (const auto & armor : armors_msg->armors) {
    if (armor.number != tracked_id) {
      continue;
    }
    same_id_count++;
    if (updateOneArmor(armor)) {
      matched = true;
      tracked_armor = armor;
    }
  }

  // 状态机
  switch (tracker_state) {
    case Tracker::DETECTING:
      if (matched) {
        if (++detect_count_ > tracking_thres_) {
          detect_count_ = 0;
          tracker_state = Tracker::TRACKING;
          lost_count_ = 0;
        }
      } else {
        detect_count_ = 0;
        tracker_state = Tracker::LOST;
      }
      break;
    case Tracker::TRACKING:
      if (!matched) {
        if (++lost_count_ > lost_thres_) {
          lost_count_ = 0;
          tracker_state = Tracker::TEMP_LOST;
        }
      } else {
        lost_count_ = 0;
      }
      break;
    case Tracker::TEMP_LOST:
      if (matched) {
        lost_count_ = 0;
        tracker_state = Tracker::TRACKING;
      } else if (++lost_count_ > lost_thres_ * 3) {
        lost_count_ = 0;
        tracker_state = Tracker::LOST;
      }
      break;
    default:
      break;
  }
}

bool OutpostTracker::updateOneArmor(const Armor & a)
{
  if (target_state.size() != kStateDim) {
    return false;
  }

  const int id = matchArmorId(a, target_state);
  last_model_id = id;

  const Eigen::Vector4d z = makeArmorYpdaMeasurement(a);
  const Eigen::Matrix4d R = makeYpdaMeasurementNoiseR(
    Eigen::Vector3d(a.pose.position.x, a.pose.position.y, a.pose.position.z));
  const Eigen::MatrixXd H = hJacobian(target_state, id);

  auto h = [this, id](const Eigen::VectorXd & x) -> Eigen::VectorXd {
      return hArmorYPDA(x, id);
    };

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
      Eigen::VectorXd c = a - b;
      if (c.size() >= 4) {
        c[0] = limitRad(c[0]);
        c[1] = limitRad(c[1]);
        c[3] = limitRad(c[3]);
      }
      return c;
    };

  const Eigen::VectorXd x_prior = target_state;
  const Eigen::VectorXd x_post = ekf_.update(z, H, R, h, z_subtract);

  // 更新后若偏差过大则拒绝本次观测（避免把滤波器带偏）
  const Eigen::Vector3d pred = hArmorXYZ(x_prior, id);
  const Eigen::Vector3d meas(a.pose.position.x, a.pose.position.y, a.pose.position.z);
  if ((pred - meas).norm() > max_match_distance_) {
    ekf_.setState(x_prior);
    target_state = x_prior;
    return false;
  }

  target_state = x_post;
  last_yaw_ = target_state[6];
  return true;
}

int OutpostTracker::matchArmorId(const Armor & a, const Eigen::VectorXd & x) const
{
  // 关键：用"高度"判定是第几块板，不能用位置。
  //
  // 前哨站三块板规格完全相同（都是小装甲板），任一时刻只能看到一块，
  // 位置上有 120° 的三重歧义 —— 无法从位置判断看到的是 0/1/2 号板，
  // 于是 EKF 的 yaw 完全不可观测（实测 v_yaw 恒为 0，真实 1.0 rad/s）。
  // 四板靶标没这个问题是因为 4 块板是 2 大 2 小交替排布，靠 armor.type
  // 就能区分板编号。
  //
  // 前哨站有一个四板没有的特征：三块板中心高度各不相同
  // （仿真里 1.114 / 1.216 / 1.318 m，互差 0.102 m），
  // PnP 解出的 z 足够区分它们，所以用高度来消除歧义。
  const double az = a.pose.position.z;
  const double za = x[4];
  const double z_of_id[3] = {za, za + x[9], za + x[10]};

  int best_id = 0;
  double best_dz = DBL_MAX;
  for (int id = 0; id < kArmorsNum; ++id) {
    const double d = std::abs(z_of_id[id] - az);
    if (d < best_dz) {
      best_dz = d;
      best_id = id;
    }
  }
  return best_id;
}

Eigen::Vector3d OutpostTracker::hArmorXYZ(const Eigen::VectorXd & x, int id) const
{
  const double angle = limitRad(x[6] + id * 2.0 * M_PI / static_cast<double>(kArmorsNum));
  const double r = x[8];
  // 三块板高度不同：板0 用基准 za，板1/板2 各加一个高度差
  const double h = (id == 1) ? x[9] : ((id == 2) ? x[10] : 0.0);

  const double armor_x = x[0] - r * std::cos(angle);
  const double armor_y = x[2] - r * std::sin(angle);
  const double armor_z = x[4] + h;
  return Eigen::Vector3d(armor_x, armor_y, armor_z);
}

Eigen::Vector4d OutpostTracker::hArmorYPDA(const Eigen::VectorXd & x, int id) const
{
  const Eigen::Vector3d xyz = hArmorXYZ(x, id);
  const Eigen::Vector3d ypd = outpostXyzToYpd(xyz);
  const double angle = limitRad(x[6] + id * 2.0 * M_PI / static_cast<double>(kArmorsNum));
  return Eigen::Vector4d(ypd[0], ypd[1], ypd[2], angle);
}

Eigen::MatrixXd OutpostTracker::hJacobian(const Eigen::VectorXd & x, int id) const
{
  // 数值差分求雅可比：结构简单、不易推错
  constexpr double kEps = 1e-6;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, kStateDim);
  for (int i = 0; i < kStateDim; ++i) {
    Eigen::VectorXd xp = x;
    Eigen::VectorXd xm = x;
    xp[i] += kEps;
    xm[i] -= kEps;
    H.col(i) = (hArmorYPDA(xp, id) - hArmorYPDA(xm, id)) / (2.0 * kEps);
  }
  return H;
}

Eigen::Vector4d OutpostTracker::makeArmorYpdaMeasurement(const Armor & a) const
{
  const Eigen::Vector3d xyz(a.pose.position.x, a.pose.position.y, a.pose.position.z);
  const Eigen::Vector3d ypd = outpostXyzToYpd(xyz);
  return Eigen::Vector4d(ypd[0], ypd[1], ypd[2], getRawYaw(a.pose.orientation));
}

Eigen::Matrix4d OutpostTracker::makeYpdaMeasurementNoiseR(const Eigen::Vector3d & armor_xyz) const
{
  // 距离越远，方位/俯仰噪声换算到 xyz 后越大；这里用对角近似
  const double distance = std::max(armor_xyz.norm(), 1e-3);
  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
  R(0, 0) = r_azimuth_ * distance * distance;
  R(1, 1) = r_pitch_ * distance * distance;
  R(2, 2) = r_distance_base_ * distance * distance;
  // 第 4 维是 PnP 解出的板朝向。前哨站任一时刻只能看到"正对相机"的那块板，
  // 板面正对时绕法线的旋转在几何上不可观测，PnP 输出的是噪声
  // （实测 -87°~-35° 随机跳）。把它按正常权重喂进去，v_yaw 会被压成 0
  // （实测 yaw 斜率只有 0.05 rad/s，真实 1.0）。
  // 位置观测本身可以通过"板在圆周上"的几何约束解出 yaw，所以这里直接
  // 把这一维的权重放到极大，等价于不使用该观测。
  R(3, 3) = 1e6;
  return R;
}

void OutpostTracker::fillTargetMsg(Target & msg) const
{
  msg.tracking = (tracker_state == Tracker::TRACKING ||
                  tracker_state == Tracker::TEMP_LOST);
  msg.id = tracked_id;
  msg.armors_num = kArmorsNum;

  if (target_state.size() == kStateDim) {
    msg.position.x = target_state[0];
    msg.position.y = target_state[2];
    msg.position.z = target_state[4];
    msg.velocity.x = target_state[1];
    msg.velocity.y = target_state[3];
    msg.velocity.z = target_state[5];
    msg.yaw = target_state[6];
    msg.v_yaw = target_state[7];
    msg.radius_1 = target_state[8];
    msg.radius_2 = target_state[8];   // 前哨站两两半径相同
    msg.dz = 0.0;                     // 无“高低两组板”的概念
  }
}

double OutpostTracker::limitRad(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double OutpostTracker::getRawYaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace rm_auto_aim
