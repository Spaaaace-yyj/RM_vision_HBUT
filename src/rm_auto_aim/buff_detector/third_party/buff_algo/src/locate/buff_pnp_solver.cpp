#include <algorithm>
#include <limits>
#include <variant>
#include <vector>

#include "buff_algo/locate/buff_pnp_solver.hpp"

namespace buff_algo {

std::vector<Pose3f> BuffPnPSolver::solve_pnp(
    const std::vector<BuffDetection>& buff_detections
) {
    last_pnp_infos_.clear();

    if (buff_detections.size() != 1) {
        has_previous_solution_ = false;
        return {};
    }

    std::vector<std::array<cv::Mat, 2>> rvecs;
    std::vector<std::array<cv::Mat, 2>> tvecs;
    std::vector<std::array<float, 2>> reprojerrs;
    solve_pnp_cv(buff_detections, rvecs, tvecs, reprojerrs);

    std::vector<std::array<Eigen::Quaternionf, 2>> rotations;
    std::vector<std::array<Eigen::Vector3f, 2>> translations;
    cvcoord_to_tfcoord(rvecs, tvecs, rotations, translations);

    // 保存候选解信息
    for (size_t i = 0; i < rotations.size(); ++i) {
        BuffPnPCandidate info;
        for (int j = 0; j < 2; ++j) {
            info.candidates[j].rotation = rotations[i][j];
            info.candidates[j].translation = translations[i][j];
        }
        info.selected_index = select_candidate(rotations[i], translations[i], reprojerrs[i]);
        info.reprojection_errors = reprojerrs[i];
        last_pnp_infos_.push_back(info);
    }

    std::vector<Pose3f> buffs_to_cam;
    for (size_t i = 0; i < rotations.size(); i++) {
        Pose3f pose;
        pose.rotation = rotations[i][last_pnp_infos_[i].selected_index];
        pose.translation = translations[i][last_pnp_infos_[i].selected_index];
        buffs_to_cam.push_back(pose);
        previous_rotation_ = rotations[i][last_pnp_infos_[i].selected_index].normalized();
        previous_translation_ = translations[i][last_pnp_infos_[i].selected_index];
        has_previous_solution_ = true;
    }
    return buffs_to_cam;
}

void BuffPnPSolver::solve_pnp_cv(
    const std::vector<BuffDetection>& buff_detections,
    std::vector<std::array<cv::Mat, 2>>& rvecs,
    std::vector<std::array<cv::Mat, 2>>& tvecs,
    std::vector<std::array<float, 2>>& reprojerrs
) const {
    static const std::vector<cv::Point3f> obj_pts = [] {
        std::vector<cv::Point3f> pts;
        for (const auto& p : consts::BUFF_POINTS) {
            pts.emplace_back(p.x(), p.y(), p.z());
        }
        return pts;
    }();

    std::for_each(buff_detections.begin(), buff_detections.end(), [&](const auto& detection) {
        const std::array<cv::Point2f, 4> img_pts {
            cv::Point2f(detection.t.x(), detection.t.y()),
            cv::Point2f(detection.l.x(), detection.l.y()),
            cv::Point2f(detection.b.x(), detection.b.y()),
            cv::Point2f(detection.r.x(), detection.r.y())
        };
        std::array<cv::Mat, 2> rvec, tvec;
        std::array<float, 2> reprojerr;
        cv::solvePnPGeneric(
            obj_pts,
            img_pts,
            camera_intrinsic_,
            camera_distortion_,
            rvec,
            tvec,
            false,
            cv::SOLVEPNP_IPPE,
            cv::noArray(),
            cv::noArray(),
            reprojerr
        );
        rvecs.emplace_back(rvec);
        tvecs.emplace_back(tvec);
        reprojerrs.emplace_back(reprojerr);
    });
}

void BuffPnPSolver::cvcoord_to_tfcoord(
    const std::vector<std::array<cv::Mat, 2>>& rvecs,
    const std::vector<std::array<cv::Mat, 2>>& tvecs,
    std::vector<std::array<Eigen::Quaternionf, 2>>& rotations,
    std::vector<std::array<Eigen::Vector3f, 2>>& translations
) const {
    const size_t len = rvecs.size();
    rotations.resize(len);
    translations.resize(len);
    // 左乘把opencv的相机系（右x，下y，前z，这是PnP得到结果默认的坐标系）转成常用的相机系（前x，左y，上z）
    const Eigen::Quaternionf cv_to_tf(-0.5, 0.5, -0.5, 0.5);
    for (size_t i = 0; i < len; i++) {
        Eigen::Vector3f tvec, rvec;
        for (int j = 0; j < 2; j++) {
            cv::cv2eigen(tvecs[i][j], tvec);
            cv::cv2eigen(rvecs[i][j], rvec);
            rotations[i][j] = cv_to_tf * Eigen::AngleAxisf(rvec.norm(), rvec.normalized());
            translations[i][j] = cv_to_tf * tvec;
        }
    }
}

void BuffPnPSolver::set_camera_matrix(const cv::Mat intrinsic, const cv::Mat distortion) {
    camera_intrinsic_ = intrinsic.clone();
    camera_distortion_ = distortion.clone();
}

std::vector<float> BuffPnPSolver::get_selected_reprojection_errors() const {
    std::vector<float> errors;
    for (const auto& info : last_pnp_infos_) {
        errors.push_back(info.reprojection_errors[info.selected_index]);
    }
    return errors;
}

int BuffPnPSolver::select_candidate(
    const std::array<Eigen::Quaternionf, 2>& rotations,
    const std::array<Eigen::Vector3f, 2>& translations,
    const std::array<float, 2>& reprojerrs
) {
    if (!has_previous_solution_) {
        return reprojerrs[1] < reprojerrs[0] ? 1 : 0;
    }

    constexpr float kTranslationWeight = 2.0f;
    constexpr float kRotationWeight = 1.0f;

    float best_score = std::numeric_limits<float>::max();
    int best_index = 0;
    for (int i = 0; i < 2; ++i) {
        const float translation_score = (translations[i] - previous_translation_).norm();
        const float rotation_score = previous_rotation_.angularDistance(rotations[i].normalized());
        const float score = kTranslationWeight * translation_score + kRotationWeight * rotation_score;
        if (score < best_score) {
            best_score = score;
            best_index = i;
        }
    }
    return best_index;
}

} // namespace buff_algo
