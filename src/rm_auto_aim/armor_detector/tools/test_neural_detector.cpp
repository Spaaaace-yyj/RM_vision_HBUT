// test_neural_detector：离线看神经网络模式的效果，不依赖 ROS 运行环境。
//
// 用法:
//   ./test_neural_detector <图片或视频> [模型路径] [输出路径]
// 例子:
//   ./test_neural_detector /path/armor.jpg
//   ./test_neural_detector /path/armor.mp4 "" /tmp/out.avi
//
// 做的事情：
//   1. 用深大模型做神经网络识别，打印每个装甲板的四角、类别、颜色、置信度；
//   2. 同时跑一遍传统视觉的找灯条，用灯条 PCA 角点对 CNN 角点做融合微调，
//      打印融合前后的角点差值（黄点=CNN 原始角点，绿框=融合后交给 PnP 的角点）；
//   3. 把结果画出来存成图片/视频，方便和传统模式对比。

#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "armor_detector/detector.hpp"
#include "armor_detector/neural_detector.hpp"

namespace
{

// 传统流程的参数，取 bringup/config/params.yaml 里实车在用的那组
rm_auto_aim::Detector makeTraditionalDetector()
{
  rm_auto_aim::Detector::LightParams light_params;
  light_params.min_ratio = 0.05;
  light_params.max_ratio = 0.8;
  light_params.max_angle = 40.0;

  rm_auto_aim::Detector::ArmorParams armor_params;
  armor_params.min_light_ratio = 0.7;
  armor_params.min_small_center_distance = 0.8;
  armor_params.max_small_center_distance = 3.2;
  armor_params.min_large_center_distance = 3.2;
  armor_params.max_large_center_distance = 5.0;
  armor_params.max_angle = 35.0;

  return rm_auto_aim::Detector(140, rm_auto_aim::BLUE, light_params, armor_params);
}

void printArmors(const std::vector<rm_auto_aim::Armor> & armors)
{
  if (armors.empty()) {
    std::printf("    没有检出装甲板\n");
    return;
  }
  for (const auto & armor : armors) {
    std::printf(
      "    [%s/%s] conf=%.3f color=%s\n", armor.number.c_str(),
      rm_auto_aim::ARMOR_TYPE_STR[static_cast<int>(armor.type)].c_str(), armor.confidence,
      armor.left_light.color == rm_auto_aim::RED ? "red" : "blue");
    std::printf(
      "      CNN 角点  左上(%.1f,%.1f) 左下(%.1f,%.1f) 右下(%.1f,%.1f) 右上(%.1f,%.1f)\n",
      armor.left_light.pca_top.x, armor.left_light.pca_top.y, armor.left_light.pca_bottom.x,
      armor.left_light.pca_bottom.y, armor.right_light.pca_bottom.x, armor.right_light.pca_bottom.y,
      armor.right_light.pca_top.x, armor.right_light.pca_top.y);
    std::printf(
      "      融合角点  左上(%.1f,%.1f) 左下(%.1f,%.1f) 右下(%.1f,%.1f) 右上(%.1f,%.1f)\n",
      armor.left_light.top.x, armor.left_light.top.y, armor.left_light.bottom.x,
      armor.left_light.bottom.y, armor.right_light.bottom.x, armor.right_light.bottom.y,
      armor.right_light.top.x, armor.right_light.top.y);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::printf(
      "用法: %s <图片或视频> [模型路径] [输出路径] [--red]\n"
      "  模型路径留空时用 model/shenzhen-0526.onnx\n"
      "  输出路径留空时: 图片存 <输入>_nn.jpg，视频存 <输入>_nn.avi\n"
      "  --red 只识别红方（默认只识别蓝方）\n",
      argv[0]);
    return 1;
  }

  const std::string input_path = argv[1];
  std::string model_path;
  std::string output_path;
  int detect_color = rm_auto_aim::BLUE;
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--red") {
      detect_color = rm_auto_aim::RED;
    } else if (model_path.empty()) {
      model_path = arg;
    } else {
      output_path = arg;
    }
  }
  if (model_path.empty()) {
    model_path = "../model/shenzhen-0526.onnx";
  }

  rm_auto_aim::NeuralDetectorParams params;
  params.conf_threshold = 0.65F;
  params.nms_threshold = 0.45F;
  params.detect_color = detect_color;

  std::unique_ptr<rm_auto_aim::NeuralDetector> neural;
  try {
    neural = std::make_unique<rm_auto_aim::NeuralDetector>(model_path, params);
  } catch (const std::exception & e) {
    std::printf("加载模型失败: %s\n", e.what());
    return 1;
  }

  auto traditional = makeTraditionalDetector();
  const rm_auto_aim::RefineParams refine_params;

  // 图片还是视频：先按图片读，读不到再按视频试
  cv::Mat first = cv::imread(input_path);
  const bool is_image = !first.empty();

  if (is_image) {
    cv::Mat rgb;
    cv::cvtColor(first, rgb, cv::COLOR_BGR2RGB);

    auto binary = traditional.preprocessImage(rgb);
    auto lights = traditional.findLights(rgb, binary, traditional.gray_img);
    auto armors = neural->detect(rgb);
    const int refined = rm_auto_aim::refineArmorCorners(armors, lights, refine_params);

    std::printf(
      "图片 %s: 传统灯条 %zu 个, 神经网络 %.1fms, 检出 %zu 个装甲板, 融合微调 %d 个\n",
      input_path.c_str(), lights.size(), neural->lastLatencyMs(), armors.size(), refined);
    printArmors(armors);

    cv::Mat vis = first.clone();
    neural->drawResults(vis, armors);
    std::string output = output_path.empty() ? input_path + "_nn.jpg" : output_path;
    cv::imwrite(output, vis);
    std::printf("  结果图: %s\n", output.c_str());
    return 0;
  }

  cv::VideoCapture capture(input_path);
  if (!capture.isOpened()) {
    std::printf("既不是图片也不是能打开的视频: %s\n", input_path.c_str());
    return 1;
  }

  std::string output = output_path.empty() ? input_path + "_nn.avi" : output_path;
  double fps = capture.get(cv::CAP_PROP_FPS);
  cv::VideoWriter writer(
    output, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps > 1.0 ? fps : 25.0,
    cv::Size(
      static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH)),
      static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT))));

  int frames = 0;
  int frames_with_armor = 0;
  int total_armors = 0;
  double total_nn_ms = 0;
  cv::Mat bgr;
  while (capture.read(bgr) && !bgr.empty()) {
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

    auto binary = traditional.preprocessImage(rgb);
    auto lights = traditional.findLights(rgb, binary, traditional.gray_img);
    auto armors = neural->detect(rgb);
    rm_auto_aim::refineArmorCorners(armors, lights, refine_params);

    ++frames;
    total_nn_ms += neural->lastLatencyMs();
    if (!armors.empty()) {
      ++frames_with_armor;
      total_armors += static_cast<int>(armors.size());
    }

    neural->drawResults(bgr, armors);
    writer.write(bgr);
  }

  std::printf(
    "视频 %s: %d 帧, 有检出的帧 %d, 装甲板总计 %d, 平均推理 %.1fms (约 %.1f FPS)\n",
    input_path.c_str(), frames, frames_with_armor, total_armors,
    frames > 0 ? total_nn_ms / frames : 0.0, frames > 0 ? 1000.0 * frames / total_nn_ms : 0.0);
  std::printf("  结果视频: %s\n", output.c_str());
  return 0;
}
