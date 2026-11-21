// 前哨站专用跟踪器
//
// 前哨站运动模型：三块装甲板绕同一竖直轴旋转，轨迹圆的圆心在地面投影重合：
//   - 三块板半径相同        固定常数 kRadius
//   - 三块板间隔 120°       angle_i = yaw + i * 2π/3
//   - 三块板高度各不相同     h_0 = 0, h_1 = +0.102, h_2 = +0.204（相对基准高度）
//   - 圆心高度固定（不上下运动）
//
// 与普通四板 EKF 的差别：
//   普通四板：r1/r2 两组半径 + za/za+dza 两组高度（大板与小板的差异）
//   前哨站  ：半径只有一个且写死；高度有三档，用 h1/h2 两个偏移量描述
//
// 半径不放进状态。单看一块板时半径不可观，交给滤波估会一路塌到几毫米，
// 圆心也跟着被带偏。半径本来就是机械尺寸，直接当常数用。
//
// 因此本 EKF 状态为 10 维：
//   [xc, vx, yc, vy, za, vz, yaw, v_yaw, h1, h2]
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

  /// 设置过程噪声参数（10 维匀速模型），必须在 init 之前调用
  void setProcessNoise(
    double sigma2_q_xyz, double sigma2_q_yaw, double dt, double initial_v_yaw = 0.0);

  /// 更新门限。预测的板位置和实测位置差超过这个值就丢掉本次观测，不喂给滤波器。
  /// 前哨站要比四板严得多，默认 0.1 米，四板用的是 max_match_distance，两者互不影响。
  void setPositionGate(double gate) { position_gate_ = gate; }

  /// 检测态用的宽门限，默认 1.0 米
  void setAcquireGate(double gate) { acquire_gate_ = gate; }

  /// 是否强制转速。默认关闭，因为判方向要用板朝向，而我们解出来的朝向基本是噪声，
  /// 判错方向比不判更糟。等朝向估计可信了再打开。
  void setFixSpin(bool on) { fix_spin_ = on; }

  // 只做时间传播，不融合观测。由节点的固定频率定时器调用。
  void propagate(double dt);

  /// 转向符号。仿真里前哨站逆时针转，但本模型 yaw 的正方向与它相反，所以是 -1。
  /// 自动判方向依赖板朝向，而我们的朝向估计基本是噪声，判反了命中率直接归零，
  /// 所以做成参数，实测一下定死。
  void setSpinSign(double sign) { spin_sign_ = sign; }

  void init(const Armors::SharedPtr & armors_msg);
  void update(const Armors::SharedPtr & armors_msg);

  /// 把 EKF 状态填进 Target 消息（tracker_node 发布用）
  void fillTargetMsg(Target & msg) const;

  Tracker::State tracker_state = Tracker::LOST;
  std::string tracked_id;
  Eigen::VectorXd target_state;   ///< 10 维状态
  bool initialized_ = false;      ///< initEKF 跑过没有，没跑过不能预测

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

  // 转速方向判定。前哨站的转速大小是已知的，方向靠攒一批板朝向来投票。
  // 判出来之后直接把 v_yaw 写成正负常量，不再交给滤波去估。
  void calDirection(double orient_yaw);

  // 用滤波后的 yaw 序列反推转速。yaw 本身每帧都被观测约束，是可观的，
  // 但 v_yaw 作为状态却学不出来，因为滤波器不需要靠它就能预测下一步。
  // 所以这里单独算：取最近若干帧的 yaw 差分，去掉板号跳变那些异常值，取平均。
  void updateSpinFromYaw(double plate_azimuth);

  std::vector<double> yaw_history_;   // 最近若干帧的 yaw
  double frame_dt_ = 0.02;            // 实测帧间隔，从消息时间戳算出来
  double last_stamp_ = -1.0;          // 上一帧时间戳
  int direction_vote_ = 0;             // 板号轮换推出来的方向
  double estimated_spin_ = 0.0;        // 从 yaw 序列估出来的转速，带符号
  bool estimated_spin_valid_ = false;  // 估出来过没有
  int direction_vote_count_ = 0;       // 连续一致次数
  int spin_direction_ = 0;            // 1 逆时针，-1 顺时针，0 还没判出来
  std::vector<double> direction_data_;

  double position_gate_ = 0.1;        // 跟踪态门限 [m]，收紧防拖
  double acquire_gate_ = 1.0;         // 检测态门限 [m]，放松好锁上
  bool fix_spin_ = false;             // 是否强制转速，默认关闭，让滤波器自己估
  double spin_sign_ = 0.0;            // 0 表示自动估转速，非零表示手动压住

  ExtendedKalmanFilter ekf_;

  double max_match_distance_;
  double max_match_yaw_diff_;
  int tracking_thres_;
  int lost_thres_;

  int detect_count_ = 0;
  double lost_time_ = 0.0;  // 连续丢失的累计时长
  int lost_count_ = 0;
  double last_yaw_ = 0.0;
  int last_model_id = -1;

  // 过程噪声参数（CV 模型，与四板 Tracker 相同形式）
  double s2qxyz_ = 30.0;
  double s2qyaw_ = 10.0;
  double dt_ = 0.02;
  double initial_v_yaw_ = 0.0;   // 前哨站转速先验 [rad/s]

  // 测量噪声参数
  double r_azimuth_ = 4e-3;
  double r_pitch_ = 4e-3;
  double r_distance_base_ = 1.0;
  double r_yaw_base_ = 9e-2;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_TRACKER__OUTPOST_TRACKER_HPP_
