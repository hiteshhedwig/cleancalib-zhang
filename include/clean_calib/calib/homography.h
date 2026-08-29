#pragma once

#include <Eigen/Core>

#include <vector>

#include "clean_calib/core/point.h"
#include "clean_calib/util/result.h"

namespace clean_calib::calib {

// Points transformed by a Hartley similarity transform. The transform maps
// the original points to coordinates whose centroid is the origin and whose
// mean distance from the origin is sqrt(2).
struct NormalizedPoints {
    std::vector<Point2D> points;
    Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
};

Result<NormalizedPoints> normalize_points(const std::vector<Point2D>& points);

// Applies H to a point and performs the homogeneous divide. The denominator
// check is relative to H and the point, so multiplying H by a nonzero scale
// does not change whether the mapping succeeds.
Result<Point2D> apply_homography(const Eigen::Matrix3d& homography,
                                 const Point2D& point);

// Estimates a 2D-to-2D homography with normalized DLT. At least four unique,
// non-collinear correspondences are required.
Result<Eigen::Matrix3d> estimate_homography(
    const std::vector<Point2D>& source_points,
    const std::vector<Point2D>& destination_points);

Result<std::vector<double>> homography_reprojection_errors(
    const Eigen::Matrix3d& homography,
    const std::vector<Point2D>& source_points,
    const std::vector<Point2D>& destination_points);

Result<double> homography_rms_reprojection_error(
    const Eigen::Matrix3d& homography,
    const std::vector<Point2D>& source_points,
    const std::vector<Point2D>& destination_points);

}  // namespace clean_calib::calib
