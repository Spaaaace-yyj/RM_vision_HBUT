// 前哨站（outpost）检测器实现
//
// 复用 Detector 的预处理 / 灯条提取（这两步与普通装甲板完全一样），
// 只把"灯条配对"这一步改成不限制灯条颜色，并把结果标记为 outpost。

#include "armor_detector/outpost_detector.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace rm_auto_aim
{

namespace
{

// 同一块板的左右两条灯条高度基本一致，上下端点在竖直方向上叠得很齐。灯条被二值化
// 打碎之后拼出来的假配对往往是一上一下，竖直方向几乎不重叠，用这个条件把它们挡掉。
// 图像 y 轴朝下，两端都取最小和最大值，避免灯条倾斜时判反。
bool verticalOverlapOk(const Light & a, const Light & b, double min_ratio)
{
  const double a_top = std::min(a.top.y, a.bottom.y);
  const double a_bottom = std::max(a.top.y, a.bottom.y);
  const double b_top = std::min(b.top.y, b.bottom.y);
  const double b_bottom = std::max(b.top.y, b.bottom.y);
  const double overlap = std::min(a_bottom, b_bottom) - std::max(a_top, b_top);
  const double shorter = std::min(a_bottom - a_top, b_bottom - b_top);
  if (shorter <= 1e-6) {
    return false;
  }
  return overlap > min_ratio * shorter;
}

}  // namespace

OutpostDetector::OutpostDetector(
  const int & binary_thres, const int & detect_color,
  const Detector::LightParams & l, const Detector::ArmorParams & a)
{
  detector = std::make_unique<Detector>(binary_thres, detect_color, l, a);
}

std::vector<Armor> OutpostDetector::detect(const cv::Mat & input)
{
  std::vector<Armor> outpost_armors;
  if (input.empty()) {
    return outpost_armors;
  }

  // 1) 预处理 + 灯条提取：与普通装甲板完全相同
  const cv::Mat binary_img = detector->preprocessImage(input);
  const std::vector<Light> lights =
    detector->findLights(input, binary_img, detector->gray_img);
  lights_ = lights;

  // 2) 灯条配对：与普通装甲板唯一的区别 —— 不检查灯条颜色
  for (auto light_1 = lights.begin(); light_1 != lights.end(); ++light_1) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); ++light_2) {
      // 两灯条之间若还夹着别的灯条，说明不是同一块装甲板
      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      // 竖直方向对不齐的也不是同一块板，交给下面的长度比和中心距判断之前先挡掉
      if (!verticalOverlapOk(*light_1, *light_2, 0.5)) {
        continue;
      }

      ArmorType type = ArmorType::INVALID;
      if (!pickArmor(*light_1, *light_2, type)) {
        continue;
      }

      Armor armor(*light_1, *light_2);
      // 前哨站三块板都是小装甲板，没有大板。这里直接钉死成 small，不让它有机会
      // 被判成 large。判成 large 会按 0.23 米的板宽去解 PnP，距离立刻缩水到六成，
      // 之前跟踪器半径收敛到 0 就是这么来的。
      armor.type = ArmorType::SMALL;
      // 前哨站数字贴图无意义，直接固定 number，tracker 会据此使用
      // initialRadius() 中 outpost 分支的 0.2765 m 旋转半径
      armor.number = OUTPOST_NUMBER;
      armor.confidence = 1.0F;
      armor.classfication_result = OUTPOST_NUMBER;
      outpost_armors.emplace_back(armor);
    }
  }

  // 前哨站三块板 120° 排布，任一时刻最多只有一块板正对相机。
  // 灯条多了以后配对组合会爆炸，5 条灯条就有 10 种配对，会配出一堆假装甲板，
  // tracker 拿去做匹配就会乱追。所以这里只留一个最像真板的配对。
  //
  // 判据用的是 center_distance 最接近同板理论值。这个理论值不能再按 1.0 算了，
  // 实测小装甲板两条灯条中心间距 0.1283 m，灯条长 0.0596 m，比值是 2.15。
  // 之前按 1.0 去挑，挑中的全是灯条被阈值打碎后拼出来的假板，解出来的距离
  // 只有真实值的一半，tracker 的半径收敛到 0，前哨站就转不起来了。
  constexpr double kSamePlateCenterDistance = 2.15;
  if (outpost_armors.size() > 1) {
    const Detector::ArmorParams & a = detector->a;
    auto score = [&a](const Armor & arm) -> double {
        const double len1 = arm.left_light.length;
        const double len2 = arm.right_light.length;
        const double avg = std::max((len1 + len2) * 0.5, 1e-6);
        const double cd = cv::norm(arm.left_light.center - arm.right_light.center) / avg;
        (void)a;
        return std::abs(cd - kSamePlateCenterDistance);
      };
    auto best = std::min_element(
      outpost_armors.begin(), outpost_armors.end(),
      [&score](const Armor & x, const Armor & y) { return score(x) < score(y); });
    Armor keep = *best;
    outpost_armors.clear();
    outpost_armors.push_back(keep);
  }

  outpost_armors_ = outpost_armors;
  return outpost_armors;
}

void OutpostDetector::drawResults(cv::Mat & img)
{

  // 灯条：端点白点 + 主轴（红/蓝灯条用不同颜色，与 Detector::drawResults 一致）
  for (const auto & light : lights_) {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    const auto line_color =
      light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // 装甲板：绿色对角叉线
  for (const auto & armor : outpost_armors_) {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 1);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1);
  }

  // 分类结果（黄字）：前哨站显示 "outpost"
  for (const auto & armor : outpost_armors_) {
    cv::putText(
      img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 1);
  }
}

bool OutpostDetector::pickArmor(const Light & light_1, const Light & light_2, ArmorType & type)
{
  // 判据参数复用 Detector 的 ArmorParams（min_light_ratio / center_distance / max_angle）
  const Detector::ArmorParams & a = detector->a;

  // 两条灯条的长度比（短/长）
  const float light_length_ratio = light_1.length < light_2.length
                                     ? static_cast<float>(light_1.length / light_2.length)
                                     : static_cast<float>(light_2.length / light_1.length);
  const bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // 两灯条中心距，以"灯条平均长度"为单位
  const float avg_light_length = static_cast<float>((light_1.length + light_2.length) / 2.0);
  if (avg_light_length <= 1e-6F) {
    type = ArmorType::INVALID;
    return false;
  }
  const float center_distance =
    static_cast<float>(cv::norm(light_1.center - light_2.center) / avg_light_length);
  const bool center_distance_ok =
    (a.min_small_center_distance <= center_distance &&
     center_distance < a.max_small_center_distance) ||
    (a.min_large_center_distance <= center_distance &&
     center_distance < a.max_large_center_distance);

  // 两灯条中心连线的倾角（度）
  const cv::Point2f diff = light_1.center - light_2.center;
  const float angle =
    static_cast<float>(std::abs(std::atan(diff.y / (std::abs(diff.x) < 1e-6F ? 1e-6F : diff.x))) /
                       CV_PI * 180.0);
  const bool angle_ok = angle < a.max_angle;

  if (!(light_ratio_ok && center_distance_ok && angle_ok)) {
    type = ArmorType::INVALID;
    return false;
  }

  // 大/小装甲板：前哨站目前都是小装甲板，这里保持与原检测器一致的判据
  type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  return true;
}

bool OutpostDetector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  const auto points =
    std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  const auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) {
      continue;
    }
    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

}  // namespace rm_auto_aim
