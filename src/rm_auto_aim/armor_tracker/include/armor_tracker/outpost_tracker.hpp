// 前哨站（outpost）专用跟踪器
//
// 前哨站运动模型：三块装甲板绕同一竖直轴旋转，轨迹圆的圆心在地面投影重合：
//   - 三块板半径相同        r = 0.275 m
//   - 三块板间隔 120°       angle_i = yaw + i * 2π/3
//   - 三块板高度各不相同     h_0 = 0, h_1 = +0.102, h_2 = +0.204（相对基准高度）
//   - 圆心高度固定（不上下运动）
//
// 与普通四板 EKF（11 维 [xc,vx,yc,vy,za,vz,yaw,v_yaw,r,l,h]）的差别：
//   普通四板：r1/r2 两组半径 + za/za+dza 两组高度（大板与小板的差异）
//   前哨站  ：半径只有一个；高度有三档，用 h1/h2 两个偏移量描述
// 因此本 EKF 状态改为：
//   [xc, vx, yc, vy, za, vz, yaw, v_yaw, r, h1, h2]
// 观测仍与普通 EKF 一致：[ypd_yaw, ypd_pitch, ypd_distance, armor_yaw]（4 维）

#ifndef ARMOR_TRACKER__OUTPOST_TRACKER_HPP_
#define ARMOR_TRACKER__OUTPOST_TRACKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
using auto_aim_interfaces::msg::Armor;
using auto_aim_interfaces::msg::Armors;
using auto_aim_interfaces::msg::Target;

class OutpostTracker
{
public:
  /// 前哨站固定几何参数（来自仿真模型，实际车体上以实测为准）
  static constexpr int kArmorsNum = 3;          ///< 装甲板数量
  static constexpr double kRadius = 0.2765;     ///< 旋转半径 [m]
  static constexpr double kH1 = 0.102;          ///< 板1 相对板0 的高度差 [m]
  static constexpr double kH2 = 0.204;          ///< 板2 相对板0 的高度差 [m]

  OutpostTracker(
    double max_match_distance, double max_match_yaw_diff,
    int tracking_thres, int lost_thres);

  void init(const Armors::SharedPtr & armors_msg);
  void update(const Armors::SharedPtr & armors_msg);

  /// 把 EKF 状态填进 Target 消息（tracker_node 发布用）
  void fillTargetMsg(Target & msg) const;

  Tracker::State tracker_state = Tracker::LOST;
  std::string tracked_id;
  Eigen::VectorXd target_state;   ///< 11 维状态
  Armor tracked_armor;

private:
  void initEKF(const Armor & a);
  bool updateOneArmor(const Armor & a);
  int matchArmorId(const Armor & a, const Eigen::VectorXd & x) const;

  Eigen::Vector3d hArmorXYZ(const Eigen::VectorXd & x, int id) const;
  Eigen::Vector4d hArmorYPDA(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd hJacobian(const Eigen::VectorXd & x, int id) const;

  Eigen::Vector4d makeArmorYpdaMeasurement(const Armor & a) const;
  Eigen::Matrix4d makeYpdaMeasurementNoiseR(const Eigen::Vector3d & armor_xyz) const;

  static double limitRad(double angle);
  static double getRawYaw(const geometry_msgs::msg::Quaternion & q);

  ExtendedKalmanFilter ekf_;

  double max_match_distance_;
  double max_match_yaw_diff_;
  int tracking_thres_;
  int lost_thres_;

  int detect_count_ = 0;
  int lost_count_ = 0;
  double last_yaw_ = 0.0;
  int last_model_id = 0;

  // 测量噪声参数
  double r_azimuth_ = 4e-3;
  double r_pitch_ = 4e-3;
  double r_distance_base_ = 1.0;
  double r_yaw_base_ = 9e-2;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_TRACKER__OUTPOST_TRACKER_HPP_
