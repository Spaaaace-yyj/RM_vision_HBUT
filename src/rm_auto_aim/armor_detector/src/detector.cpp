// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/detector.hpp"
#include "auto_aim_interfaces/msg/debug_armor.hpp"
#include "auto_aim_interfaces/msg/debug_light.hpp"

namespace rm_auto_aim
{
Detector::Detector(
  const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a)
: binary_thres(bin_thres), detect_color(color), l(l), a(a)
{
}

std::vector<Armor> Detector::detect(const cv::Mat & input)
{
  binary_img = preprocessImage(input);
  lights_ = findLights(input, binary_img, gray_img);
  armors_ = matchLights(lights_);

  if (!armors_.empty()) {
    classifier->extractNumbers(input, armors_);
    classifier->classify(armors_);
  }

  return armors_;
}

cv::Mat Detector::preprocessImage(const cv::Mat & rgb_img)
{
  //转换灰度图
  // cv::Mat gray_img;
  cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

  //二值化
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  return binary_img;
}

  cv::Point2f Detector::findLightCorner(
      const cv::Mat& gray_img,
      const cv::Point2f& center,
      const cv::Point2f& axis,
      float length,
      float width,
      int direction)
{
  const float SEARCH_START = 0.4f;
  const float SEARCH_END = 0.6f;

  float dx = axis.x * direction;
  float dy = axis.y * direction;

  float search_length = length * (SEARCH_END - SEARCH_START);

  std::vector<cv::Point2f> candidates;

  // 灯条法线方向
  cv::Point2f normal(-axis.y, axis.x);

  int half_width = std::max(2, (int)(width * 0.5f));

  for(int offset=-half_width; offset<=half_width; offset++)
  {
    cv::Point2f start(
        center.x + length * SEARCH_START * dx + normal.x * offset,
        center.y + length * SEARCH_START * dy + normal.y * offset
    );

    float max_diff = 0;
    cv::Point2f best_point;
    bool found = false;

    for(float step=0; step<search_length; step+=1.0f)
    {
      cv::Point2f cur = start + cv::Point2f(dx,dy)*step;

      int x = (int)cur.x;
      int y = (int)cur.y;

      if(x<=1 || y<=1 || x>=gray_img.cols-2 || y>=gray_img.rows-2)
        break;

      int prev = gray_img.at<uchar>(
          (int)(cur.y - dy),
          (int)(cur.x - dx));

      int curv = gray_img.at<uchar>(y,x);

      float diff = prev - curv;

      if(diff > max_diff)
      {
        max_diff = diff;
        best_point = cur - cv::Point2f(dx,dy);
        found = true;
      }
    }

    if(found && max_diff > 10)
    {
      candidates.push_back(best_point);
    }
  }

  if(candidates.empty())
    return cv::Point2f(-1,-1);

  cv::Point2f mean(0,0);

  for(auto&p:candidates)
    mean+=p;

  return mean*(1.0f/candidates.size());
}

std::vector<Light> Detector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img, const cv::Mat & gray_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  //根据二值化后图像找边框
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  cv::GaussianBlur(gray_img, gray_img, cv::Size(3,3), 0);

  vector<Light> lights;
  this->debug_lights.data.clear();

  for (const auto & contour : contours) {
    //过滤过小的边框（降噪）
    if (contour.size() < 5) continue;

    //寻找边框的最小外接矩形
    auto r_rect = cv::minAreaRect(contour);
    auto light = Light(r_rect);

    if (isLight(light)) {
      auto rect = light.boundingRect();
      //防止rect超出图像边界
      if (  // Avoid assertion failed
      0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
      0 <= rect.height && rect.y + rect.height <= rbg_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rbg_img(rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              // if point is inside contour
              sum_r += roi.at<cv::Vec3b>(i, j)[0];
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? RED : BLUE;
        //PCA优化
        //构造点云
        cv::Mat data(contour.size(), 2, CV_64F);
        for (size_t i = 0; i < contour.size(); i++)
        {
          data.at<double>(i,0) = contour[i].x;
          data.at<double>(i,1) = contour[i].y;
        }
        cv::PCA pca (data, cv::Mat(), cv::PCA::DATA_AS_ROW);

        cv::Point2f axis(pca.eigenvectors.at<double>(0,0), pca.eigenvectors.at<double>(0,1));
        if(axis.y < 0)
        {
          axis = -axis;
        }
        axis /= cv::norm(axis);
        //投影当前边框的所有点到灯条方向上，寻找最大和最小的点
        double min_proj = 1e9;
        double max_proj = -1e9;

        for(auto &p : contour)
        {
          double proj = p.x * axis.x + p.y * axis.y;

          if(proj < min_proj)
          {
            min_proj = proj;
          }

          if(proj > max_proj)
          {
            max_proj = proj;
          }
        }
        cv::Point2f center(pca.mean.at<double>(0,0), pca.mean.at<double>(0,1));
        float length = max_proj - min_proj;

        cv::Point2f rough_top = center - axis * length * 0.5;
        cv::Point2f rough_bottom = center + axis * length * 0.5;

        cv::Point2f top = findLightCorner(gray_img, center, axis, length, light.width, -1);
        cv::Point2f bottom = findLightCorner(gray_img, center, axis, length, light.width, 1);

        if(top.x < 0)
          top = rough_top;
        if(bottom.x < 0)
          bottom = rough_bottom;

        light.top = top;
        light.bottom = bottom;
        // light.top = rough_top;
        // light.bottom = rough_bottom;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

bool Detector::isLight(const Light & light)
{
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  auto_aim_interfaces::msg::DebugLight light_data;
  light_data.center_x = light.center.x;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
{
  std::vector<Armor> armors;
  this->debug_armors.data.clear();

  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != detect_color || light_2->color != detect_color) continue;

      //确认两个灯条之间不再存在可能是灯条的结构
      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

ArmorType Detector::isArmor(const Light & light_1, const Light & light_2)
{
  // Ratio of the length of 2 lights (short side / long side)
  //两个灯条的长度比值
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                             center_distance < a.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  // Fill in debug information
  auto_aim_interfaces::msg::DebugArmor armor_data;
  armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  this->debug_armors.data.emplace_back(armor_data);

  return type;
}

cv::Mat Detector::getAllNumbersImage()
{
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto & armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

void Detector::drawResults(cv::Mat & img)
{
  // Draw Lights
  for (const auto & light : lights_) {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.pca_top, 2, cv::Scalar(255, 0, 0), 1);
    cv::circle(img, light.pca_bottom, 2, cv::Scalar(255, 0, 0), 1);

    auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // Draw armors
  for (const auto & armor : armors_) {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 1);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1);
  }

  // Show numbers and confidence
  for (const auto & armor : armors_) {
    cv::putText(
      img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 1);
  }
}

}  // namespace rm_auto_aim