#pragma once

#include <Eigen/Core>

namespace clean_calib {

// Rigid body transform mapping WORLD points to CAMERA points:
//
//   X_camera = R * X_world + t
//
// R is a proper 3x3 rotation matrix (R^T R = I, det R = +1).
// t is a 3-vector in the camera's units (metres).
//
// Defaults to the identity pose (camera at the world origin, looking
// down +Z in world coordinates).
struct Pose {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

}  // namespace clean_calib
