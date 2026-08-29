#pragma once

#include <Eigen/Core>

#include <vector>

#include "clean_calib/core/camera.h"
#include "clean_calib/core/point.h"
#include "clean_calib/core/pose.h"
#include "clean_calib/util/result.h"

namespace clean_calib::calib {

struct RefinementOptions {
    int max_iterations = 50;
    double finite_difference_step = 1e-6;
    double step_tolerance = 1e-10;
    double cost_tolerance = 1e-12;
    double gradient_tolerance = 1e-10;
    double initial_damping = 1e-3;
};

struct RefinementResult {
    CameraModel camera;
    std::vector<Pose> poses;
    double initial_rms_reprojection_error = 0.0;
    double final_rms_reprojection_error = 0.0;
    int iterations = 0;
    bool converged = false;
};

// Residual order is view-major, then point-major, with (predicted_x-observed_x,
// predicted_y-observed_y) for every observation.
Result<Eigen::VectorXd> calibration_residuals(
    const std::vector<Point3D>& object_points,
    const std::vector<std::vector<Point2D>>& image_points,
    const CameraModel& camera,
    const std::vector<Pose>& poses);

// Central-difference Jacobian. Columns are:
//   fx fy cx cy skew k1 k2 k3 p1 p2,
// followed by one rotation-vector (rx ry rz) and translation (tx ty tz)
// block per view.
Result<Eigen::MatrixXd> numeric_calibration_jacobian(
    const std::vector<Point3D>& object_points,
    const std::vector<std::vector<Point2D>>& image_points,
    const CameraModel& camera,
    const std::vector<Pose>& poses,
    double relative_step = 1e-6);

Result<RefinementResult> refine_calibration_gauss_newton(
    const std::vector<Point3D>& object_points,
    const std::vector<std::vector<Point2D>>& image_points,
    const CameraModel& initial_camera,
    const std::vector<Pose>& initial_poses,
    const RefinementOptions& options = {});

Result<RefinementResult> refine_calibration_levenberg_marquardt(
    const std::vector<Point3D>& object_points,
    const std::vector<std::vector<Point2D>>& image_points,
    const CameraModel& initial_camera,
    const std::vector<Pose>& initial_poses,
    const RefinementOptions& options = {});

}  // namespace clean_calib::calib
