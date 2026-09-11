// test_buff_detector：离线测试 OnnxBuffDetector。
// 用法: ./test_buff_detector [video=buff.mp4] [model=buff.onnx] [output=out.avi]
// 读视频 → 检测 → 画 9 个关键点 → 保存输出视频，并在终端打印每帧检测信息。
#include <cstdio>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "buff_algo/detector/detector_common.hpp"
#include "buff_detector/onnx_buff_detector.hpp"

namespace {

// 0..8 号关键点颜色，8 号是 R 中心用白色
const std::array<cv::Scalar, 9> kColors = {
    cv::Scalar(0, 255, 0),    // 0
    cv::Scalar(255, 0, 0),    // 1
    cv::Scalar(0, 0, 255),    // 2
    cv::Scalar(255, 255, 0),  // 3
    cv::Scalar(0, 255, 255),  // 4
    cv::Scalar(128, 0, 255),  // 5
    cv::Scalar(255, 128, 0),  // 6
    cv::Scalar(255, 0, 128),  // 7
    cv::Scalar(255, 255, 255) // 8  R 中心
};

void draw_detection(cv::Mat& image, const buff_algo::DetectedBuff& detected) {
    const auto& detection = detected.detection;

    // 8 边形轮廓，按顺序连接 0..7 号关键点
    for (int i = 0; i < 8; ++i) {
        const cv::Point a(
            cvRound(detected.keypoints[i].x()), cvRound(detected.keypoints[i].y()));
        const cv::Point b(
            cvRound(detected.keypoints[(i + 1) % 8].x()),
            cvRound(detected.keypoints[(i + 1) % 8].y()));
        cv::line(image, a, b, cv::Scalar(0, 200, 200), 1);
    }

    // 9 个关键点圆点 + 编号
    for (std::size_t i = 0; i < detected.keypoints.size(); ++i) {
        const cv::Point point(
            cvRound(detected.keypoints[i].x()), cvRound(detected.keypoints[i].y()));
        cv::circle(image, point, 4, kColors[i], cv::FILLED, cv::LINE_AA);
        cv::putText(
            image, std::to_string(i), point + cv::Point(6, -6),
            cv::FONT_HERSHEY_SIMPLEX, 0.4, kColors[i], 1, cv::LINE_AA);
    }

    // PnP 四点 T/L/B/R 连线
    const std::vector<cv::Point> quad = {
        cv::Point(cvRound(detection.t.x()), cvRound(detection.t.y())),
        cv::Point(cvRound(detection.l.x()), cvRound(detection.l.y())),
        cv::Point(cvRound(detection.b.x()), cvRound(detection.b.y())),
        cv::Point(cvRound(detection.r.x()), cvRound(detection.r.y()))};
    cv::polylines(image, quad, true, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

    // 文字：颜色/激活/置信度
    const char* color_name = detection.color == 0 ? "BLUE" : "RED";
    const char* active_name = detection.label == 0 ? "inactive" : "active";
    cv::putText(
        image, color_name + std::string(" ") + active_name + " " +
                   std::to_string(detection.confidence).substr(0, 4),
        quad[0] + cv::Point(-30, -10), cv::FONT_HERSHEY_SIMPLEX, 0.5,
        cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string video_path = argc >= 2 ? argv[1] : "model/buff.mp4";
    const std::string model_path = argc >= 3 ? argv[2] : "model/buff.onnx";
    const std::string output_path = argc >= 4 ? argv[3] : "model/buff_detected.avi";

    buff_detector::OnnxBuffDetector detector(model_path, {0.5f, 0.4f});

    cv::VideoCapture capture(video_path);
    if (!capture.isOpened()) {
        std::fprintf(stderr, "failed to open video: %s\n", video_path.c_str());
        return 1;
    }
    const int width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    const int fps = static_cast<int>(capture.get(cv::CAP_PROP_FPS));
    cv::VideoWriter writer(
        output_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, {width, height});
    if (!writer.isOpened()) {
        std::fprintf(stderr, "failed to create output video: %s\n", output_path.c_str());
        return 1;
    }

    cv::Mat frame;
    int frame_index = 0;
    int total_detections = 0;
    while (capture.read(frame)) {
        const auto detections = detector.detect_with_keypoints(frame);

        for (const auto& detected : detections) {
            draw_detection(frame, detected);
        }
        writer.write(frame);

        if (detections.empty() || frame_index % 10 == 0) {
            std::printf(
                "frame %4d: %zu detections | ", frame_index, detections.size());
            for (const auto& detected : detections) {
                std::printf(
                    "[color=%d label=%d conf=%.3f] ", detected.detection.color,
                    detected.detection.label, detected.detection.confidence);
            }
            std::printf("\n");
        }
        total_detections += static_cast<int>(detections.size());
        ++frame_index;
    }

    std::printf("\nDONE: %d frames, %d detections total\n", frame_index, total_detections);
    return 0;
}
