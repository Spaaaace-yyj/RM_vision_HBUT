#include "buff_algo/selector/buff_selector.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <vector>

namespace buff_algo {

BuffSelector::BuffSelector(int robot_color)
    : robot_color_(robot_color) {}

void BuffSelector::select_buffs(
    const std::vector<BuffDetection>& src,
    std::vector<BuffDetection>& dst
) {
    struct BuffInfo {
        BuffDetection buff;
        double angle;
    };
    std::vector<BuffInfo> infos;
    last_frame_buffs_info_.clear();

    static const std::vector<cv::Point3f> obj_pts = [] {
        std::vector<cv::Point3f> pts;
        for (const auto& p : consts::BUFF_POINTS) {
            pts.emplace_back(p.x(), p.y(), p.z());
        }
        return pts;
    }();

    for (const auto& buff : src) {
        // 筛选颜色和我方一致，且未激活的能量机关
        // 注意：能量机关是打我方的未激活扇叶来激活
        if (buff.color != robot_color_) continue;
        if (buff.label != static_cast<int>(BuffActivation::INACTIVATE)) continue;

        // 构造图像点集
        // 顺序对应 consts::BUFF_POINTS: Top, Left, Bottom, Right
        std::vector<cv::Point2f> image_points = {
            {buff.t.x(), buff.t.y()},
            {buff.l.x(), buff.l.y()},
            {buff.b.x(), buff.b.y()},
            {buff.r.x(), buff.r.y()}
        };

        // PnP解算
        cv::Mat rvec, tvec;
        // 使用IPPE算法，适用于平面4点PnP
        cv::solvePnP(obj_pts, image_points, camera_intrinsic_, camera_distortion_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

        // 计算与相机Z轴夹角
        // tvec是物体在相机坐标系下的位置向量 (x, y, z)
        // 相机光轴为Z轴 (0, 0, 1)
        // 夹角 tan(theta) = sqrt(x^2 + y^2) / z
        // theta = atan(sqrt(x^2 + y^2) / z)
        double x = tvec.at<double>(0);
        double y = tvec.at<double>(1);
        double z = tvec.at<double>(2);

        // 防止z为0导致除零错误（虽然实际不太可能）
        if (std::abs(z) < 1e-6) continue;

        // 保存PnP结果用于可视化
        BuffCandidateInfo info;
        info.detection = buff;
        info.position_in_camera = Eigen::Vector3f(x, y, z);
        double angle = std::atan(std::hypot(x, y) / z);
        info.angle_to_optical_axis = angle;
        last_frame_buffs_info_.push_back(info);

        infos.push_back({buff, angle});
    }

    if (infos.empty()) {
        last_selected_buffs_.clear();
        return;
    }

    // 按照夹角从小到大排序
    std::ranges::sort(infos, {}, &BuffInfo::angle);

    // 选择夹角最小的一个
    dst.push_back(infos[0].buff);
    last_selected_buffs_ = dst;
}

void BuffSelector::update_color(int robot_color) {
    robot_color_ = robot_color;
}

void BuffSelector::set_camera_matrix(const cv::Mat& intrinsic, const cv::Mat& distortion) {
    camera_intrinsic_ = intrinsic.clone();
    camera_distortion_ = distortion.clone();
}

} // namespace buff_algo
