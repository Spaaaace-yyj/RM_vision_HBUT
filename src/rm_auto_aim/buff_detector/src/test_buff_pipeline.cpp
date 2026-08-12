// test_buff_pipeline：算法全链路离线测试，不依赖 ROS2。
// 视频 → 检测 → 筛选 → PnP → BuffTracker 跟踪 → BuffPredictor 预测 → 打印状态 + 画打击点。
//
// 用法: ./test_buff_pipeline [video] [mode=1|2] [color=0|1] [output]
//   mode:  1=小符固定转速KF  2=大符RANSAC正弦拟合
//   color: 0=打蓝符  1=打红符；本测试视频里是蓝符，传 0
#include <cstdio>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "buff_algo/common/math_utils.hpp"
#include "buff_algo/locate/buff_pnp_solver.hpp"
#include "buff_algo/predictor/buff_predictor.hpp"
#include "buff_algo/selector/buff_selector.hpp"
#include "buff_algo/tracker/buff_tracker.hpp"
#include "buff_algo/tracker/tracker_status.hpp"
#include "buff_detector/onnx_buff_detector.hpp"

namespace {

// 离线测试用的假相机内参，实车用真实标定结果
const cv::Mat kCameraMatrix = (cv::Mat_<double>(3, 3) << 800.0, 0.0, 640.0,
                                                               0.0, 800.0, 384.0,
                                                               0.0, 0.0, 1.0);
const cv::Mat kDistortion = cv::Mat::zeros(1, 5, CV_64F);

// 把预测打击点投影回图像画个圈。
// 浙大 Pose3f 是相机系「前x 左y 上z」，图像投影 u = cx + fx*(-y)/x, v = cy + fy*(-z)/x
void draw_aim_point(cv::Mat& image, const Eigen::Vector3f& p) {
    if (p.x() <= 0.1f) return;
    const double fx = kCameraMatrix.at<double>(0, 0);
    const double fy = kCameraMatrix.at<double>(1, 1);
    const double cx = kCameraMatrix.at<double>(0, 2);
    const double cy = kCameraMatrix.at<double>(1, 2);
    const int x = cvRound(-p.y() / p.x() * fx + cx);
    const int y = cvRound(-p.z() / p.x() * fy + cy);
    cv::circle(image, {x, y}, 12, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
    cv::circle(image, {x, y}, 3, cv::Scalar(0, 255, 0), cv::FILLED);
}

const char* status_name(buff_algo::StatusType status) {
    switch (status) {
        case buff_algo::StatusType::LOST: return "LOST";
        case buff_algo::StatusType::CONVERGING: return "CONVERGING";
        case buff_algo::StatusType::TRACKING: return "TRACKING";
        case buff_algo::StatusType::TEMP_LOST: return "TEMP_LOST";
    }
    return "?";
}

}  // namespace

int main(int argc, char** argv) {
    const std::string video_path = argc >= 2 ? argv[1] : "model/buff.mp4";
    const int mode = argc >= 3 ? std::stoi(argv[2]) : 1;        // 1 小符 2 大符
    const int color = argc >= 4 ? std::stoi(argv[3]) : 0;       // 视频里是蓝符
    const std::string output_path = argc >= 5 ? argv[4] : "model/buff_pipeline.avi";

    // ---- 链路搭建，和 ROS 节点里完全一样的 5 个模块 ----
    buff_detector::OnnxBuffDetector detector("model/buff.onnx", {0.5f, 0.4f});

    buff_algo::BuffSelector selector(color);
    selector.set_camera_matrix(kCameraMatrix, kDistortion);

    buff_algo::BuffPnPSolver solver;
    solver.set_camera_matrix(kCameraMatrix, kDistortion);

    cv::FileStorage tracker_config(
        "third_party/buff_algo/config/tracker/tracker.yaml", cv::FileStorage::READ);
    buff_algo::BuffTracker tracker(tracker_config["buff_tracker"]);
    tracker.set_mode(static_cast<buff_algo::BuffMode>(mode));

    buff_algo::BuffPredictor predictor(cv::FileStorage(
        "third_party/buff_algo/config/predictor/predictor.yaml",
        cv::FileStorage::READ)["buff_predictor"]);

    // ---- 跑视频 ----
    cv::VideoCapture capture(video_path);
    if (!capture.isOpened()) {
        std::fprintf(stderr, "failed to open video: %s\n", video_path.c_str());
        return 1;
    }
    cv::VideoWriter writer(
        output_path,
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        static_cast<int>(capture.get(cv::CAP_PROP_FPS)),
        {static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH)),
         static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT))});

    cv::Mat frame;
    int frame_index = 0;
    int tracking_frames = 0;
    buff_algo::StatusType last_status = buff_algo::StatusType::LOST;

    while (capture.read(frame)) {
        // 1. 检测，网络输出 9 个关键点
        const std::vector<buff_algo::DetectedBuff> detected =
            detector.detect_with_keypoints(frame);
        std::vector<buff_algo::BuffDetection> detections;
        detections.reserve(detected.size());
        for (const auto& item : detected) detections.push_back(item.detection);

        // 2. 筛选：颜色、激活状态、夹角最小
        std::vector<buff_algo::BuffDetection> selected;
        selector.select_buffs(detections, selected);

        // 3. PnP 解位姿，IPPE 双解消歧
        const std::vector<buff_algo::Pose3f> poses = solver.solve_pnp(selected);
        if (!poses.empty()) {
            tracker.push(buff_algo::Buff(poses[0]));
        }

        // 4. 跟踪，时间戳用视频播放位置
        const double timestamp = capture.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
        tracker.update(timestamp);

        // 5. 预测当前时刻打击点，提前量由 gimbal 统一做
        const buff_algo::BuffState state = tracker.get_state();
        predictor.set_state(state, timestamp, timestamp);
        const Eigen::Vector3f aim_point = predictor.predict_position(0.0f);

        // ---- 可视化与打印 ----
        if (tracker.status() != last_status || frame_index % 30 == 0) {
            std::printf(
                "frame %4d | %-10s | roll=%7.2f° v_roll=%6.2f°/s | aim=(%6.2f, %6.2f, %6.2f) m\n",
                frame_index,
                status_name(tracker.status()),
                buff_algo::r2d(state.roll), buff_algo::r2d(state.roll_velocity),
                aim_point.x(), aim_point.y(), aim_point.z());
            last_status = tracker.status();
        }
        if (tracker.status() == buff_algo::StatusType::TRACKING) ++tracking_frames;

        draw_aim_point(frame, aim_point);
        writer.write(frame);
        ++frame_index;
    }

    std::printf(
        "\nDONE: %d frames | TRACKING in %d frames | final status: %s\n",
        frame_index, tracking_frames, status_name(tracker.status()));
    return 0;
}
