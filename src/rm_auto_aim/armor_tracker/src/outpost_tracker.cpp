// 前哨站专用跟踪器。
//
// 目标几何：三块小装甲板绕同一根竖直轴旋转，半径相同，高度各不相同，圆心在
// 地面上的投影重合。水平面上看是个圆，竖直面上看是三个高度。
//
// 状态 10 维：圆心 x、vx、圆心 y、vy、基准高度、vz、旋转角、角速度、
// 板一高度差、板二高度差。板零的高度就是基准高度。
//
// 半径不放进状态里，直接用 kRadius 这个常数。单看一块板的时候半径根本不可观，
// 交给滤波去估只会越估越离谱，实测会一路塌到几毫米，圆心也跟着被拉偏。
// 半径本来是机械尺寸，写死更稳。
//
// 观测 4 维：方位角、俯仰角、斜距、板朝向角，和四板一致。

#include "armor_tracker/outpost_tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>

#include <rclcpp/rclcpp.hpp>

namespace rm_auto_aim
{

namespace
{
// 状态维数，所有矩阵都按这个尺寸开
constexpr int kStateDim = 10;

// 转速方向投票用。攒够 30 帧板朝向，每隔 3 帧比一次转角。
// 单帧转角太小时不算数，跳变太大的当成解算锯齿丢掉。
// 攒 15 帧就够投票了。二十五赫兹下约半秒，转速一弧度每秒的话
// 隔三帧已有零点一二弧度的变化，远高于判定阈值，不需要等三十帧。
constexpr int kDirectionDataThresh = 15;
constexpr std::size_t kAngleCompareInterval = 3;
constexpr double kJumpThresh = 0.4;
constexpr double kStableThresh = 5e-3;

// 检测态允许连续丢多少帧才判丢。一帧就判丢的话，检测端抖一下就被打回 LOST，
// 然后重新初始化，整个跟踪一直在原地重启。设成 50 帧，二十多赫兹下差不多两秒，
// 足够扛过检测端常见的连续漏检。
// 丢帧阈值改成按时间。检测帧间隔实测二十五到五百毫秒不等，按帧数算没有意义。
// 阈值要比最坏帧间隔宽裕。实测最长断帧五百七十六毫秒，取八百毫秒，
// 断一两帧不至于降级。检测态给两秒，因为刚起步时观测本来就稀。
constexpr double kDetectLostTime = 2.0;   // 检测态丢这么久就回 LOST
constexpr double kTrackLostTime = 0.8;    // 跟踪态丢这么久降为临时丢失

// 反推转速用的滑窗长度和合理性范围。十二帧在二十五赫兹下约半秒。
// 单帧 yaw 变化超过 0.3 弧度基本就是板号跳了 120 度，不能算进平均。
// 板号连读几次一致才认方向。两次就够，再多会拖慢判定。
constexpr int kDirectionVoteThresh = 2;

// 主动相位修正的系数和异常门限。系数是每帧吃掉多少比例的相位误差，
// 门限用来挡掉板号判错时那一百二十度的大跳。
constexpr double kPhaseFeedback = 0.0;  // 先关掉：实测会让转速虚高，见下
constexpr double kPhaseJumpThresh = 0.4;

constexpr std::size_t kYawHistoryLen = 12;
constexpr double kYawJumpThresh = 0.3;
constexpr double kMinSpinRate = 0.2;
constexpr double kMaxSpinRate = 5.0;

// 直角坐标转方位角、俯仰角、斜距
Eigen::Vector3d outpostXyzToYpd(const Eigen::Vector3d & xyz)
{
  const double x = xyz.x();                                 // 取 x
  const double y = xyz.y();                                 // 取 y
  const double z = xyz.z();                                 // 取 z
  const double xy_norm = std::max(std::hypot(x, y), 1e-6);  // 水平距离，防除零
  const double distance = std::max(xyz.norm(), 1e-6);       // 斜距，防除零
  return Eigen::Vector3d(std::atan2(y, x), std::atan2(z, xy_norm), distance);  // 方位、俯仰、斜距
}
}  // namespace

OutpostTracker::OutpostTracker(
  double max_match_distance, double max_match_yaw_diff, int tracking_thres, int lost_thres)
: target_state(Eigen::VectorXd::Zero(kStateDim)),  // 状态先清零
  max_match_distance_(max_match_distance),         // 认同一块板的距离门限
  max_match_yaw_diff_(max_match_yaw_diff),         // 板朝向的匹配门限
  tracking_thres_(tracking_thres),                 // 进跟踪要连续多少帧
  lost_thres_(lost_thres)                          // 丢多少帧算丢
{
}

void OutpostTracker::setProcessNoise(
  double sigma2_q_xyz, double sigma2_q_yaw, double dt, double initial_v_yaw)
{
  s2qxyz_ = sigma2_q_xyz;          // 位置的过程噪声
  s2qyaw_ = sigma2_q_yaw;          // 角度的过程噪声
  initial_v_yaw_ = initial_v_yaw;  // 转速先验
  if (dt > 1e-6) {                 // dt 有效才更新
    dt_ = dt;
  }
}

void OutpostTracker::init(const Armors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {  // 一帧里没板就没法初始化
    return;
  }

  // 挑离图像中心最近的一块，它最可能正对相机，解算也最准
  double min_distance = DBL_MAX;                 // 当前最小的偏距
  tracked_armor = armors_msg->armors.front();    // 先拿第一块兜底
  for (const auto & armor : armors_msg->armors) {  // 遍历本帧所有板
    if (armor.distance_to_image_center < min_distance) {  // 这块更靠中间
      min_distance = armor.distance_to_image_center;      // 更新最小偏距
      tracked_armor = armor;                              // 换成这块
    }
  }

  tracked_id = tracked_armor.number;   // 记下板号
  RCLCPP_INFO(
    rclcpp::get_logger("outpost_tracker"), "初始化，用板 %.2f 米处的观测",
    Eigen::Vector3d(
      tracked_armor.pose.position.x, tracked_armor.pose.position.y,
      tracked_armor.pose.position.z).norm());
  initEKF(tracked_armor);              // 用这块板起滤波
  tracker_state = Tracker::DETECTING;  // 先进检测态
}

void OutpostTracker::initEKF(const Armor & a)
{
  const double xa = a.pose.position.x;  // 板的 x
  const double ya = a.pose.position.y;  // 板的 y
  const double za = a.pose.position.z;  // 板的 z，也就是高度

  // yaw 不能直接用 PnP 解出来的板姿态。板正对相机时，板绕自身法线的旋转在几何上
  // 不可观测，解出来是噪声，实测在负八十七度到负三十五度之间乱跳。初值差五十度，
  // 云台按它算出的板位置也差五十度，转过头就看不见目标了。
  //
  // 但能完整看到的那块板一定在圆心靠近相机的一侧，所以拿板位置的方位角当初值。
  double yaw = std::atan2(ya, xa);  // 方位角当 yaw 初值
  last_yaw_ = yaw;                  // 记下最近一次的角度

  // 板落在圆周上，圆心就是板往外再推一个半径
  const double r = kRadius;            // 标称半径
  double xc = xa + r * std::cos(yaw);  // 圆心 x
  double yc = ya + r * std::sin(yaw);  // 圆心 y

  // 圆心物理上是钉死的。如果之前锁过一次，就沿用旧圆心，只按高度重解相位。
  // 不这样做的话，每次丢锁重新捕获都从一块板重推圆心，推出来的值必然和上次不同，
  // 画面上就是圆心在跳，转速也跟着忽快忽慢。
  if (center_locked_) {
    xc = locked_xc_;
    yc = locked_yc_;
    const double dz_arr[kArmorsNum] = {0.0, kH1, kH2};  // 三块板各自的高度差
    int best_id = 0;                                    // 按高度判板号
    double best = DBL_MAX;
    for (int i = 0; i < kArmorsNum; ++i) {
      const double d = std::abs(locked_za_ + dz_arr[i] - za);
      if (d < best) {
        best = d;
        best_id = i;
      }
    }
    yaw = std::atan2(yc - ya, xc - xa) - best_id * 2.0 * M_PI / kArmorsNum;
    last_yaw_ = yaw;
  }


  // 先当看到的是板零，另外两块用标称高度差。半径不进状态，上面算圆心时用的是常数
  target_state = Eigen::VectorXd::Zero(kStateDim);  // 先清空
  // 转速初值给零，代表暂时不知道转多快，交给滤波器从观测里学。
  // 注意零是待估初值，不是强制值，和以前那种每帧覆盖成常量完全是两回事。
  // 前哨站一直在转，所以预测必须跟着转，不能停在原地，否则板转到背面看不见的时候
  // 云台会对着一个不存在的静止相位一直打。
  const double spin = 0.0;
  const double za_state = center_locked_ ? locked_za_ : za;  // 圆心锁定时高度也沿用
  target_state << xc, 0.0, yc, 0.0, za_state, 0.0, yaw, spin, kH1, kH2;  // 灌初值

  // 初始协方差。位置和高度给中等，速度必须给得很小。
  //
  // 之前速度那一项给了 10，标准差三米多每秒。前几帧观测一冲，滤波器就给圆心
  // 安上一个很大的速度，之后圆心按这个速度一路漂走，漂远了门限就拒掉所有观测，
  // 跟踪断掉重来，再漂一次。前哨站是固定在地上的，速度本来就该是零。
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(kStateDim, kStateDim);  // 先给单位阵
  P0.diagonal() << 0.5, 0.1, 0.5, 0.1,    // 圆心位置和速度，速度方差取 0.1
    0.25, 0.1,                            // 高度和速度，同样压住
    1.0, 25.0,                            // 角度和角速度
    1e-9, 1e-9;                           // 两个高度差必须钉死，见下面的说明

  // 从这里开始搭 10 维匀速模型。过程噪声的构造和四板一样，把加速度噪声按 dt
  // 展开到位置和速度上。早先这里用默认构造的 EKF，噪声矩阵压根没设，圆心估计
  // 抖得厉害，x 在两米一到四米七之间跳，云台拿着错的 yaw 追板，越追越偏。
  const double dt = dt_;  // 周期

  auto 
  f = [dt](const Eigen::VectorXd & x) -> Eigen::VectorXd {  // 状态转移
      Eigen::VectorXd x_new = x;                 // 拷一份出来改
      x_new(0) += x(1) * dt;                     // 圆心 x 往前走
      x_new(2) += x(3) * dt;                     // 圆心 y 往前走
      x_new(4) += x(5) * dt;                     // 高度往前走
      x_new(6) = limitRad(x(6) + x(7) * dt);     // 角度往前走并归一
      return x_new;
    };

  auto j_f = [dt](const Eigen::VectorXd &) -> Eigen::MatrixXd {  // 转移的雅可比
      Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kStateDim, kStateDim);  // 单位阵起步
      F(0, 1) = dt;                              // x 对 vx
      F(2, 3) = dt;                              // y 对 vy
      F(4, 5) = dt;                              // z 对 vz
      F(6, 7) = dt;                              // 角度对角速度
      return F;
    };

  // 观测函数这里是占位，真正用到时按板号在 updateOneArmor 里传
  auto h = [](const Eigen::VectorXd &) -> Eigen::VectorXd {
      return Eigen::VectorXd::Zero(4);           // 四维观测
    };
  auto j_h = [](const Eigen::VectorXd &) -> Eigen::MatrixXd {
      return Eigen::MatrixXd::Zero(4, kStateDim);  // 四乘十的占位
    };

  auto u_q = [this]() -> Eigen::MatrixXd {
      const double t = dt_;
      // 过程噪声怎么分配，是这套滤波能不能用的关键。前哨站钉在地面上，圆心基本
      // 不动；四板的圆心会跟着车跑。照四板那样给大的圆心噪声，滤波器就会用平移
      // 圆心去解释板上位置的变化，yaw 转不起来，v_yaw 永远学不到。实测那时 yaw
      // 斜率只有每秒零点零二弧度，真实是一弧度每秒。所以这里把圆心压死。
      // 圆心必须钉住。日志实测：噪声给大之后，滤波器用平移圆心去解释板绕圈的轨迹，
      // 预测和实测几乎完全重合，但 yaw 一直趴在零附近不动，xc 在四米一到四米四九之间跳。
      // 前哨站是钉在地上的，圆心物理上不动，压小之后只剩转 yaw 一条路可走。
      const double v1 = 0.01;                    // 圆心和高度噪声
      const double v2 = s2qyaw_;                 // 角度噪声
      const double a = std::pow(t, 4) / 4.0;     // t 的四次项系数
      const double b = std::pow(t, 3) / 2.0;     // t 的三次项系数
      const double c = std::pow(t, 2);           // t 的二次项系数

      Eigen::MatrixXd q = Eigen::MatrixXd::Zero(kStateDim, kStateDim);  // 全零起步
      auto fill_cv_noise = [&](int pos, int vel, double var) {  // 给一个轴填噪声
        q(pos, pos) = a * var;                   // 位置方差
        q(pos, vel) = b * var;                   // 位置和速度的相关项
        q(vel, pos) = b * var;                   // 对称的那一项
        q(vel, vel) = c * var;                   // 速度方差
      };
      fill_cv_noise(0, 1, v1);  // 圆心 x 和 vx
      fill_cv_noise(2, 3, v1);  // 圆心 y 和 vy
      fill_cv_noise(4, 5, v1);  // 高度和 vz
      fill_cv_noise(6, 7, v2);  // 角度和角速度，和四板用同一套噪声
      // 这里原来有一行 q(7, 7) = 1e-6，把转速维几乎锁死，理由是当初估出来的转速
      // 只有真值的三成。那个结论是在检测端七成假观测、圆心冻在错误位置的时候得出的。
      // 现在观测干净了，圆心也准了，转速应当交回滤波器自己估。
      // 两个高度差是机械尺寸，当常量处理，噪声留零，只靠观测修正
      return q;
    };

  auto u_r = [](const Eigen::VectorXd &) -> Eigen::MatrixXd {
      return Eigen::MatrixXd::Identity(4, 4);    // 观测噪声在这里是占位
    };

  ekf_ = ExtendedKalmanFilter(f, h, j_f, j_h, u_q, u_r, P0);  // 建滤波器
  ekf_.setState(target_state, P0);                            // 灌初值
  initialized_ = true;                                       // 模型函数和状态都齐了
}

void OutpostTracker::update(const Armors::SharedPtr & armors_msg)
{
  const Tracker::State prev_state = tracker_state;  // 记下迁移前的状态，方便打日志

  // 真实帧间隔。节点传进来的 dt 是写死的 0.02，实际跑起来是二十五赫兹左右，
  // 反推转速必须用真实间隔，不然后面差一倍。
  const double stamp = armors_msg->header.stamp.sec + armors_msg->header.stamp.nanosec * 1e-9;
  if (last_stamp_ > 0 && stamp > last_stamp_) {
    const double d = stamp - last_stamp_;
    if (d > 1e-3 && d < 0.5) frame_dt_ = d;
  }
  last_stamp_ = stamp;

  // 用实测帧间隔当 dt。节点传进来的那个是写死的 0.02 秒，而实际跑十二三赫兹，
  // 真实间隔六十多毫秒。不换的话每帧只按三分之一的时间推算旋转，转速算出来
  // 只有真值的三成，画面上就是框比真板转得慢一大截。
  dt_ = frame_dt_;

  // 预测已经搬到固定频率的定时器里做了，这里只负责融合这一帧观测。
  // 检测输出间隔中位三十八毫秒、尾部五百多毫秒，跟着检测节拍预测的话，
  // 框会停一下再猛冲一段，看起来就是转速时快时慢。

  lost_time_ += frame_dt_;  // 先按丢帧累加，匹配上之后再清零

  bool matched = false;       // 本帧有没有匹配上
  int same_id_count = 0;      // 同号板的个数

  for (const auto & armor : armors_msg->armors) {  // 遍历本帧的板
    if (armor.number != tracked_id) {              // 板号不同就跳过
      continue;
    }
    same_id_count++;              // 同号板加一
    if (updateOneArmor(armor)) {  // 逐块融合
      matched = true;             // 标记匹配上了
      tracked_armor = armor;      // 记下这块板
    }
  }

  // 状态机
  switch (tracker_state) {
    case Tracker::DETECTING:  // 检测态
      if (matched) {
        lost_time_ = 0.0;                         // 有匹配就把丢失时长清掉
        if (++detect_count_ > tracking_thres_) {  // 连续够帧数了
          detect_count_ = 0;                      // 计数清零
          tracker_state = Tracker::TRACKING;      // 进跟踪
          lost_count_ = 0;                        // 丢失计数也清
        }
      } else {
        // 按时间判丢，不按帧数。帧间隔实测从二十五毫秒到五百多毫秒都有，
        // 二十帧可能是半秒也可能是十秒，按帧数算的阈值在时间上没有意义。
        if (lost_time_ > kDetectLostTime) {
          detect_count_ = 0;
          tracker_state = Tracker::LOST;          // 真的跟丢了
        }
      }
      break;
    case Tracker::TRACKING:  // 跟踪态
      if (!matched) {
        if (lost_time_ > kTrackLostTime) {    // 丢掉超过这个时长才降级
          tracker_state = Tracker::TEMP_LOST;
        }
      } else {
        lost_time_ = 0.0;                     // 有板就清零
      }
      break;
    case Tracker::TEMP_LOST:  // 临时丢失
      if (matched) {
        lost_count_ = 0;
        tracker_state = Tracker::TRACKING;   // 找回来了
      } else if (lost_time_ > kTrackLostTime * 3) {  // 再丢这么久就真没了
        tracker_state = Tracker::LOST;
      }
      break;
    default:
      break;
  }

  // 状态迁移打日志。跟踪老是断的时候，靠这个看是卡在检测态进不去，
  // 还是进了跟踪态又被丢帧打下来。
  if (prev_state != tracker_state) {
    auto name = [](Tracker::State s) {
        switch (s) {
          case Tracker::LOST: return "LOST";
          case Tracker::DETECTING: return "DETECTING";
          case Tracker::TRACKING: return "TRACKING";
          case Tracker::TEMP_LOST: return "TEMP_LOST";
        }
        return "?";
      };
    RCLCPP_INFO(
      rclcpp::get_logger("outpost_tracker"), "状态 %s -> %s  匹配=%d 已丢 %.2f 秒",
      name(prev_state), name(tracker_state), static_cast<int>(matched), lost_time_);
  }
}

bool OutpostTracker::updateOneArmor(const Armor & a)
{
  if (target_state.size() != kStateDim) {  // 没初始化就直接退出
    return false;
  }

  const int id = matchArmorId(a, target_state);  // 靠高度判出板号

  // 用板号轮换顺序判转向。三块板高度不同，板号是唯一的，它按什么顺序换就说明了
  // 目标往哪边转。实测逆时针时是 0 变 2 变 1 变 0，也就是每次减一；反过来是加一。
  // 板朝向在正对相机时根本不可观测，所以方向不能从朝向拿。
  if (last_model_id >= 0 && id != last_model_id) {
    const int step = (id - last_model_id + kArmorsNum) % kArmorsNum;
    const int dir = (step == kArmorsNum - 1) ? 1 : (step == 1 ? -1 : 0);
    if (dir != 0) {
      if (dir == direction_vote_) {
        ++direction_vote_count_;
      } else {
        direction_vote_ = dir;
        direction_vote_count_ = 1;
      }
      // 官方规则：前哨站转向整局固定不变，所以判到一次就锁死，不再改。
      // 锁死之后即使偶尔出现两次反向的误判也不会把方向带翻。
      if (spin_direction_ == 0 && direction_vote_count_ >= kDirectionVoteThresh) {
        spin_direction_ = direction_vote_;
      }
    }
  }
  last_model_id = id;                            // 记下这次用的板号

  // 先做门限判断，不合格的观测直接扔掉，不要进滤波器。
  // 之前是先更新再回滚，等于让一个坏观测先在协方差里留下痕迹，几次下来圆心就被
  // 拖到错误的位置去了。前哨站的门限要比四板严得多，默认 0.1 米。
  const Eigen::Vector3d pred = hArmorXYZ(target_state, id);  // 按状态算出的板位置
  const Eigen::Vector3d meas(a.pose.position.x, a.pose.position.y, a.pose.position.z);  // 实测位置
  // 门限分两段。刚起步还在检测态时用宽门限，先把跟踪锁上；进入跟踪之后再收紧，
  // 防止假观测把已经锁好的状态拖走。全程用窄门限的话，检测端漏几帧就再也进不了
  // 跟踪态，一直在原地初始化。
  const double gate =
    tracker_state == Tracker::DETECTING ? acquire_gate_ : position_gate_;
  if ((pred - meas).norm() > gate) {
    return false;  // 差太远，这块板不是当前目标，或者距离解错了
  }

  const Eigen::Vector4d z = makeArmorYpdaMeasurement(a);  // 观测向量
  const Eigen::Matrix4d R = makeYpdaMeasurementNoiseR(    // 观测噪声
    Eigen::Vector3d(a.pose.position.x, a.pose.position.y, a.pose.position.z));
  const Eigen::MatrixXd H = hJacobian(target_state, id);  // 观测的雅可比

  auto h = [this, id](const Eigen::VectorXd & x) -> Eigen::VectorXd {
      return hArmorYPDA(x, id);  // 按板号预测观测
    };

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
      Eigen::VectorXd c = a - b;  // 先求残差
      if (c.size() >= 4) {
        c[0] = limitRad(c[0]);    // 方位角归一
        c[1] = limitRad(c[1]);    // 俯仰角归一
        c[3] = limitRad(c[3]);    // 板朝向归一
      }
      return c;
    };

  const Eigen::VectorXd x_post = ekf_.update(z, H, R, h, z_subtract);  // 融合一次
  target_state = x_post;        // 接受这次更新
  last_yaw_ = target_state[6];  // 记下当前角度

  // 转速大小由参数给定，方向由板号轮换判出来。
  //
  // 为什么不交给滤波器估：三块板一百二十度对称，板号又能自由重选，滤波器只要换个
  // 板号就不用转 yaw。更根本的原因是相机永远只看得到正对镜头的那块板，它在 odom
  // 里几乎不动，整个旋转过程在观测上不存在。实测把强制值从一点零改成零点三，估出
  // 来的值就跟着从一点零变成零点三，完全放开则收敛到零点零九，等于没有信息。
  //
  // 大小按官方规则配，方向能判就判，判不出来先按正方向跑，别停在原地不动。
  if (spin_sign_ != 0.0) {
    spin_direction_ = (spin_sign_ > 0.0) ? 1 : -1;  // 手动覆盖，调试用
  }
  int spin_dir = spin_direction_;
  if (spin_dir == 0) {
    spin_dir = 1;
  }
  target_state[7] = spin_dir * std::abs(initial_v_yaw_);

  // 两个高度差按回常数。这两个量是前哨站的机械尺寸，不是待估参数。
  // 实测放任它去估会塌成零，三块板的理论高度变成同一个值，判板号就永远返回零号，
  // 进而方向投票失效、yaw 找不到参照、转速转不起来。日志里高度从 0.42 跳到 0.20
  // 板号还是零，就是这个原因。
  target_state[8] = kH1;
  target_state[9] = kH2;

  // 记住这次收敛到的圆心，丢锁重新捕获时沿用，避免圆心跳变
  locked_xc_ = target_state[0];
  locked_yc_ = target_state[2];
  locked_za_ = target_state[4];
  center_locked_ = true;


  // 主动相位修正。
  //
  // 只靠滤波更新的话，yaw 的修正量受卡尔曼增益限制，每帧只能拉一点，跟不上转速
  // 误差的累积，表现就是瞄着瞄着越来越偏。这里再用实测的板方位角和预测的板方位角
  // 做一次直接比对，按比例把相位拽过去。圆心和半径都已知，板的方位角就精确对应
  // 一个 yaw，这条修正是无偏的。
  //
  // 系数取小一点，单帧拉两成，几帧就把偏差吃掉了，又不会让框跟着测量噪声抖。
  const Eigen::Vector3d pred_after = hArmorXYZ(target_state, id);
  const double az_meas = std::atan2(meas.y() - target_state[2], meas.x() - target_state[0]);
  const double az_pred = std::atan2(pred_after.y() - target_state[2], pred_after.x() - target_state[0]);
  double phase_err = limitRad(az_meas - az_pred);
  if (std::abs(phase_err) > kPhaseJumpThresh) {
    phase_err = 0.0;  // 差得离谱多半是板号判错了，这次不修
  }
  target_state[6] += kPhaseFeedback * phase_err;

  ekf_.setState(target_state);
  return true;
}

void OutpostTracker::propagate(double dt)
{
  if (!initialized_ || target_state.size() != kStateDim) {
    return;  // 还没初始化，这时候调 predict 会因为模型函数没设而崩溃
  }
  if (dt > 1e-4 && dt < 0.5) {
    dt_ = dt;  // 用调用方给的真实时间步
  }
  target_state = ekf_.predict();

  // 和 update 里保持一致的三个强制项：转速大小、转速方向、两个高度差
  if (spin_sign_ != 0.0) {
    spin_direction_ = (spin_sign_ > 0.0) ? 1 : -1;
  }
  // 转速大小取参数，方向优先用板号轮换判出来的结果，判不出来先按正方向跑。
  // 不转是不行的：目标一直在转，转速给零等于预测一个静止的假目标。
  // 判出来之后会被下面的赋值覆盖，所以先给个默认值不会锁死。
  int spin_dir = spin_direction_;
  if (spin_dir == 0) {
    spin_dir = 1;
  }
  target_state[7] = spin_dir * std::abs(initial_v_yaw_);
  target_state[8] = kH1;
  target_state[9] = kH2;
  ekf_.setState(target_state);
}

void OutpostTracker::updateSpinFromYaw(double plate_azimuth)
{
  yaw_history_.push_back(plate_azimuth);  // 收一帧实测方位角
  if (yaw_history_.size() > kYawHistoryLen) {
    yaw_history_.erase(yaw_history_.begin());  // 滑窗
  }
  if (yaw_history_.size() < kYawHistoryLen) {
    return;  // 窗口还没填满
  }

  double sum = 0.0;
  int valid = 0;
  for (std::size_t i = 1; i < yaw_history_.size(); ++i) {
    const double diff = limitRad(yaw_history_[i] - yaw_history_[i - 1]);  // 解缠
    if (std::abs(diff) > kYawJumpThresh) {
      continue;  // 板号跳变，丢掉这个样本
    }
    sum += diff;
    ++valid;
  }
  if (valid < static_cast<int>(kYawHistoryLen) / 2) {
    return;  // 有效样本太少，这次不更新
  }

  const double rate = sum / valid / frame_dt_;  // 平均每帧变化除以帧间隔
  if (!std::isfinite(rate) || std::abs(rate) < kMinSpinRate || std::abs(rate) > kMaxSpinRate) {
    return;  // 不在合理范围，多半是观测在跳
  }

  // 一阶平滑。单帧估计有噪声，直接用会让框抖；平滑之后场上改转速也能在半秒内跟上。
  if (estimated_spin_valid_) {
    estimated_spin_ = 0.7 * estimated_spin_ + 0.3 * rate;
  } else {
    estimated_spin_ = rate;
    estimated_spin_valid_ = true;
  }
  spin_direction_ = (estimated_spin_ >= 0.0) ? 1 : -1;  // 符号一起估出来
}

void OutpostTracker::calDirection(double orient_yaw)
{
  // 方向一旦判出来就不再改，避免来回横跳
  if (spin_direction_ != 0) {
    return;
  }
  direction_data_.push_back(orient_yaw);  // 攒这一帧的板朝向
  if (static_cast<int>(direction_data_.size()) < kDirectionDataThresh) {
    return;  // 数据还不够，先不投票
  }

  int anticlockwise = 0, clockwise = 0, stable = 0, jump = 0;
  for (std::size_t i = kAngleCompareInterval; i < direction_data_.size(); ++i) {
    // 隔几帧比一次，能放大转角，抗单帧噪声
    const double diff =
      limitRad(direction_data_[i] - direction_data_[i - kAngleCompareInterval]);
    if (std::abs(diff) > kJumpThresh) {
      ++jump;  // 跳变太大，当成解算锯齿丢掉
      continue;
    }
    if (diff > kStableThresh) {
      ++anticlockwise;
    } else if (diff < -kStableThresh) {
      ++clockwise;
    } else {
      ++stable;
    }
  }

  // 票数最多的那个方向胜出。这里要求领先得足够明显，不然就再攒一轮。
  // 判反了代价很大，宁可多等几帧。
  const int majority = std::max({anticlockwise, clockwise, stable});
  if (majority >= static_cast<int>(direction_data_.size()) * 2 / 3) {
    if (majority == anticlockwise) {
      spin_direction_ = 1;
    } else if (majority == clockwise) {
      spin_direction_ = -1;
    } else {
      spin_direction_ = 0;  // 一直不动，判不出来
    }
  } else {
    spin_direction_ = 0;  // 票数分散，再攒一轮
  }
  if (spin_direction_ != 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("outpost_tracker"), "前哨站转向判为 %s",
      spin_direction_ > 0 ? "逆时针" : "顺时针");
  }
  direction_data_.clear();  // 这一轮数据用完清掉
}

int OutpostTracker::matchArmorId(const Armor & a, const Eigen::VectorXd & x) const
{
  // 判板号只能看高度，不能看位置。三块板规格一模一样，任一时刻基本只看得到一块，
  // 按位置去对会有 120 度的三重歧义，同一块板既能算零号也能算一号，yaw 就没约束了。
  // 四板靠 armor.type 就能定板号，前哨站没这个条件，高度是最好用的抓手。
  //
  // 试过改用板朝向判板号，效果更差。解出来的朝向噪声太大，板号在相邻两块之间来回
  // 跳，每跳一次 yaw 差 120 度，跟踪状态被反复打断，tracking 从百分百掉到一半。
  const double az = a.pose.position.z;                  // 实测高度
  const double za = x[4];                               // 估计的基准高度
  const double z_of_id[3] = {za, za + x[8], za + x[9]};  // 三块板各自的理论高度

  int best_id = 0;           // 先当零号
  double best_dz = DBL_MAX;  // 当前最小的高度差
  for (int id = 0; id < kArmorsNum; ++id) {         // 三块挨个比
    const double d = std::abs(z_of_id[id] - az);    // 高度差
    if (d < best_dz) {                              // 更接近
      best_dz = d;
      best_id = id;                                 // 记下这块
    }
  }
  return best_id;  // 返回最像的那块
}

Eigen::Vector3d OutpostTracker::hArmorXYZ(const Eigen::VectorXd & x, int id) const
{
  const double angle = limitRad(x[6] + id * 2.0 * M_PI / static_cast<double>(kArmorsNum));  // 这块板的角
  const double r = kRadius;                                                                // 半径是常数
  // 板号决定高度，零号用基准，一号二号再加差值
  const double h = (id == 1) ? x[8] : ((id == 2) ? x[9] : 0.0);  // 高度偏移

  const double armor_x = x[0] - r * std::cos(angle);  // 板的 x
  const double armor_y = x[2] - r * std::sin(angle);  // 板的 y
  const double armor_z = x[4] + h;                    // 板的 z
  return Eigen::Vector3d(armor_x, armor_y, armor_z);  // 拼成位置
}

Eigen::Vector4d OutpostTracker::hArmorYPDA(const Eigen::VectorXd & x, int id) const
{
  const Eigen::Vector3d xyz = hArmorXYZ(x, id);       // 先算位置
  const Eigen::Vector3d ypd = outpostXyzToYpd(xyz);   // 转成方位俯仰斜距
  const double angle = limitRad(x[6] + id * 2.0 * M_PI / static_cast<double>(kArmorsNum));  // 板的角度
  return Eigen::Vector4d(ypd[0], ypd[1], ypd[2], angle);  // 拼成四维观测
}

Eigen::MatrixXd OutpostTracker::hJacobian(const Eigen::VectorXd & x, int id) const
{
  // 数值差分求雅可比，结构简单也不容易推错
  constexpr double kEps = 1e-6;                             // 差分步长
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, kStateDim);  // 四乘十一
  for (int i = 0; i < kStateDim; ++i) {                     // 每一维挨个求
    Eigen::VectorXd xp = x;                                 // 正扰动的副本
    Eigen::VectorXd xm = x;                                 // 负扰动的副本
    xp[i] += kEps;
    xm[i] -= kEps;
    H.col(i) = (hArmorYPDA(xp, id) - hArmorYPDA(xm, id)) / (2.0 * kEps);  // 中心差分
  }
  return H;
}

Eigen::Vector4d OutpostTracker::makeArmorYpdaMeasurement(const Armor & a) const
{
  const Eigen::Vector3d xyz(a.pose.position.x, a.pose.position.y, a.pose.position.z);  // 实测位置
  const Eigen::Vector3d ypd = outpostXyzToYpd(xyz);                                   // 转成角度
  return Eigen::Vector4d(ypd[0], ypd[1], ypd[2], getRawYaw(a.pose.orientation));      // 带上板朝向
}

Eigen::Matrix4d OutpostTracker::makeYpdaMeasurementNoiseR(const Eigen::Vector3d & armor_xyz) const
{
  // 噪声按距离放大。同样是几个像素的误差，距离越远折成角度就越离谱。只取对角，
  // 没考虑方位和俯仰的相关性，现在的精度下够用。
  const double distance = std::max(armor_xyz.norm(), 1e-3);  // 斜距
  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();               // 先清零
  R(0, 0) = r_azimuth_ * distance * distance;                // 方位角噪声
  R(1, 1) = r_pitch_ * distance * distance;                  // 俯仰角噪声
  R(2, 2) = r_distance_base_ * distance * distance;          // 斜距噪声
  // 第四维是解算出来的板朝向，这一维基本不能信。能看到的那块板正对相机，板绕
  // 自身法线的旋转不可观测，解出来是噪声，实测在负八十七度到负三十五度之间跳。
  // 按正常权重喂进去，v_yaw 会被硬压到零，所以干脆不用，方差放到极大。
  R(3, 3) = 1e6;                                             // 等于废弃这一维
  return R;
}

void OutpostTracker::fillTargetMsg(Target & msg) const
{
  msg.tracking = (tracker_state == Tracker::TRACKING ||  // 正在跟踪
                  tracker_state == Tracker::TEMP_LOST);   // 或者刚丢一下，都算在跟
  msg.id = tracked_id;                                    // 板号
  msg.armors_num = kArmorsNum;                            // 三块板

  if (target_state.size() == kStateDim) {  // 有状态才填
    msg.position.x = target_state[0];      // 圆心 x
    msg.position.y = target_state[2];      // 圆心 y
    msg.position.z = target_state[4];      // 基准高度
    msg.velocity.x = target_state[1];      // 速度 x
    msg.velocity.y = target_state[3];      // 速度 y
    msg.velocity.z = target_state[5];      // 速度 z
    msg.yaw = target_state[6];             // 旋转角
    msg.v_yaw = target_state[7];           // 角速度
    msg.radius_1 = kRadius;                // 半径是常数，不做估计
    msg.radius_2 = kRadius;                // 三块板半径相同
    msg.dz = 0.0;                          // 前哨站没有高低两组的说法
    // 三块板的高度各不相同，只靠上面那个标量 dz 表达不了，这里逐板给出来。
    // 板零用基准高度，板一和板二是加在基准上的差值。
    msg.dz_list = {0.0, target_state[8], target_state[9]};
  }
}

double OutpostTracker::limitRad(double angle)
{
  while (angle > M_PI) {      // 超过 π 就往下减
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {     // 小于负 π 就往上加
    angle += 2.0 * M_PI;
  }
  return angle;
}

double OutpostTracker::getRawYaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q;       // tf2 的四元数
  tf2::fromMsg(q, tf_q);      // 从消息转过来
  double roll = 0.0;          // 横滚只用占位
  double pitch = 0.0;         // 俯仰只用占位
  double yaw = 0.0;           // 真正要的偏航
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);  // 转成欧拉角
  return yaw;
}

}  // namespace rm_auto_aim
