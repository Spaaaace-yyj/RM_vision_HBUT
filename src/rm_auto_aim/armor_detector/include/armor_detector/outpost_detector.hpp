// 前哨站（outpost）检测器
//
// 前哨站结构：三块装甲板绕同一竖直轴旋转，轨迹圆的圆心在地面投影重合，
// 三块板中心高度分别为 1.114 / 1.216 / 1.318 m（互差 0.102 m），
// 排列间隔 120°，每块板面倾斜 15°，灯条为红色（四板靶标为蓝色）。
//
// 检测思路与普通装甲板完全一致：
//   二值化 -> 找灯条 -> 灯条配对 -> PnP
// 与普通检测器的差别只有两点：
//   1) 不限制灯条颜色（仿真里前哨站灯条过曝偏白，靠颜色分不出来）
//   2) 检测结果统一标记为 outpost（数字贴图无意义，不做数字识别）
//
// 本类不改动原 Detector 的任何行为，只在它外面包一层。

#ifndef ARMOR_DETECTOR__OUTPOST_DETECTOR_HPP_
#define ARMOR_DETECTOR__OUTPOST_DETECTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector.hpp"

namespace rm_auto_aim
{

class OutpostDetector
{
public:
  /// 前哨站装甲板被赋予的 number 字段（tracker 的 initialRadius() 会据此
  /// 返回 0.2765 m 的旋转半径）
  static constexpr const char * OUTPOST_NUMBER = "outpost";

  OutpostDetector(
    const int & binary_thres, const int & detect_color,
    const Detector::LightParams & l, const Detector::ArmorParams & a);

  /// 检测前哨站装甲板（结果已标记为 outpost）
  std::vector<Armor> detect(const cv::Mat & input);

  /// 底层通用检测器（灯条提取 / 配对判据都复用它）
  std::unique_ptr<Detector> detector;

  /// 在图像上绘制灯条与装甲板（与 Detector::drawResults 的绘制风格保持一致）
  void drawResults(cv::Mat & img);

private:
  std::vector<Armor> outpost_armors_;   // 最近一次检测结果（供绘制用）
  std::vector<Light> lights_;           // 最近一次灯条（供绘制用）

  /// 两条灯条是否满足装甲板判据（复用 Detector::isArmor 的公开版本）
  bool pickArmor(const Light & light_1, const Light & light_2, ArmorType & type);

  /// 两灯条之间是否夹着第三条灯条
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__OUTPOST_DETECTOR_HPP_
