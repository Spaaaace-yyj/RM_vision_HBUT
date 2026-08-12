#pragma once

#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "buff_algo/common/types.hpp"

namespace buff_algo {

// 单个候选能量机关的 PnP 信息（位置 + 与相机光轴的夹角），用于外部可视化。
struct BuffCandidateInfo {
    BuffDetection detection;
    Eigen::Vector3f position_in_camera = Eigen::Vector3f::Zero();
    double angle_to_optical_axis = 0.0;
};

// 从检测到的多个能量机关扇叶中选择出目标扇叶。
// 选择逻辑：
// 1. 筛选出颜色为我方且状态为未激活的扇叶（能量机关通过打击我方未激活扇叶来激活）；
// 2. 对候选扇叶做 PnP 解算，得到其在相机坐标系下的位置；
// 3. 计算每个候选扇叶位置向量与相机光轴（Z轴）的夹角；
// 4. 选择夹角最小的一个作为目标（越靠近画面中心，通常代表越容易命中）。
class BuffSelector {
public:
    explicit BuffSelector(int robot_color);

    void select_buffs(
        const std::vector<BuffDetection>& src,
        std::vector<BuffDetection>& dst
    );

    void update_color(int robot_color);

    void set_camera_matrix(const cv::Mat& intrinsic, const cv::Mat& distortion);

    // 获取上一次调用 select_buffs 时的候选信息（用于可视化）。
    const std::vector<BuffCandidateInfo>& get_last_candidates() const { return last_frame_buffs_info_; }
    const std::vector<BuffDetection>& get_last_selected() const { return last_selected_buffs_; }

private:
    int robot_color_;
    cv::Mat camera_intrinsic_, camera_distortion_;

    std::vector<BuffCandidateInfo> last_frame_buffs_info_;
    std::vector<BuffDetection> last_selected_buffs_;
};

} // namespace buff_algo
