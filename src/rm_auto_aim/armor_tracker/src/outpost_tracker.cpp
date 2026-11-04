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

  last_yaw_ = 0.0;
  const double yaw = getRawYaw(a.pose.orientation);
  last_yaw_ = yaw;

  // 由单块板反推旋转中心：板在圆周上，圆心 = 板位置 + r * (cos/sin)
  const double r = kRadius;
  const double xc = xa + r * std::cos(yaw);
  const double yc = ya + r * std::sin(yaw);

  // 初始认为观测到的这块是板0（高度基准），另外两块按固定高度差建模；
  // 由于三块板高度固定且已知，h1/h2 直接给标称值，靠后续观测微调。
  target_state = Eigen::VectorXd::Zero(kStateDim);
  target_state << xc, 0.0, yc, 0.0, za, 0.0, yaw, 0.0, r, kH1, kH2;

  // 初始协方差（对角）
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(kStateDim, kStateDim);
  P0.diagonal() << 1.0, 10.0, 1.0, 10.0,    // xc, vx, yc, vy
    0.25, 1.0,                              // za, vz
    1.0, 25.0,                              // yaw, v_yaw
    0.01, 0.0025, 0.0025;                   // r, h1, h2

  // 用默认构造的 EKF（内部已有匀速模型与默认 Q/R），只设置状态与协方差
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
  // 在 3 块板里找一个与观测最接近的
  const Eigen::Vector3d meas(a.pose.position.x, a.pose.position.y, a.pose.position.z);
  int best_id = 0;
  double best_dist = DBL_MAX;
  for (int id = 0; id < kArmorsNum; ++id) {
    const double d = (hArmorXYZ(x, id) - meas).norm();
    if (d < best_dist) {
      best_dist = d;
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
  R(3, 3) = r_yaw_base_;
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
