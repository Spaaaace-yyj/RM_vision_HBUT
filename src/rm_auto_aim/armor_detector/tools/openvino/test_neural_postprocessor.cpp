#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "armor_detector/neural_postprocessor.hpp"

int main() {
  try {
    const auto check = [](bool ok) {
      if (!ok) {
        throw std::runtime_error("Decoder regression");
      }
    };
    std::array<float, 22> row{10, 20, 10, 40, 30, 40, 30, 20, 10, 5, 0,
                              0,  0,  0,  0,  0,  5,  0,  0,  0,  0, 0};
    rm_auto_aim::NeuralDetectorParams params;
    const auto decode = [&] {
      return rm_auto_aim::decodeNeuralOutput(row.data(), 1, cv::Size(640, 640),
                                             cv::Size(1280, 1024), params);
    };
    auto armors = decode();
    check(armors.size() == 1 && armors[0].number == "3");
    check(std::abs(armors[0].left_light.top.x - 20) < .01F);
    check(std::abs(armors[0].left_light.top.y - 32) < .01F);
    check(std::abs(armors[0].right_light.bottom.x - 60) < .01F);
    check(armors[0].type == rm_auto_aim::ArmorType::SMALL);
    params.detect_color = rm_auto_aim::RED;
    check(decode().empty());
    params.swap_color = true;
    check(decode().size() == 1);
    params.ignore_classes = {"3"};
    check(decode().empty());
    params.ignore_classes.clear();
    row[9] = 0;
    row[11] = 8;
    check(decode().empty());  // inactive/gray rejected
    row[9] = 5;
    row[11] = 0;
    row[8] = -10;
    check(decode().empty());
    row[8] = 10;
    row[0] = std::numeric_limits<float>::quiet_NaN();
    check(decode().empty());
    row[0] = 10;
    // Class mapping includes the large hero armor and both base types.
    row[16] = 0;
    row[14] = 5;
    armors = decode();
    check(armors.size() == 1 && armors[0].number == "1" &&
          armors[0].type == rm_auto_aim::ArmorType::LARGE);
    std::array<float, 44> duplicate;
    std::copy(row.begin(), row.end(), duplicate.begin());
    std::copy(row.begin(), row.end(), duplicate.begin() + 22);
    armors = rm_auto_aim::decodeNeuralOutput(duplicate.data(), 2, {640, 640},
                                             {640, 640}, params);
    check(armors.size() == 1);
    std::cout << "PASS: corner scaling, class/color mapping, swap, ignore, "
                 "confidence, finite check, NMS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
