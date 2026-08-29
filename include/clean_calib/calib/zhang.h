#pragma once

#include <Eigen/Core>

#include <vector>

#include "clean_calib/core/camera.h"
#include "clean_calib/core/pose.h"
#include "clean_calib/util/result.h"

namespace clean_calib::calib {

struct ZhangInitialization {
    CameraIntrinsics intrinsics;
    std::vector<Pose> poses;
};

// Builds the 2N x 6 homogeneous constraint matrix from N planar-view
// homographies. At least three independent views are required.
Result<Eigen::MatrixXd> build_zhang_constraint_matrix(
    const std::vector<Eigen::Matrix3d>& homographies);

// Recovers K from the image-of-the-absolute-conic constraints H^T B H,
// where B = K^-T K^-1.
Result<CameraIntrinsics> estimate_intrinsics_from_homographies(
    const std::vector<Eigen::Matrix3d>& homographies);

// Recovers a world-to-camera pose from H = K [r1 r2 t]. The rotation is
// projected onto the nearest proper rotation matrix in SO(3).
Result<Pose> estimate_pose_from_homography(
    const Eigen::Matrix3d& homography,
    const CameraIntrinsics& intrinsics);

Result<ZhangInitialization> initialize_zhang(
    const std::vector<Eigen::Matrix3d>& homographies);

}  // namespace clean_calib::calib
