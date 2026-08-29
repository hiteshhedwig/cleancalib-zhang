#include "clean_calib/calib/homography.h"

#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace clean_calib::calib {
namespace {

constexpr double kRelativeDenominatorTolerance = 1e-12;
constexpr double kMinimumShapeRatio = 1e-10;
constexpr double kDuplicateTolerance = 1e-12;

bool is_finite(const Point2D& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

bool all_finite(const Eigen::Matrix3d& matrix) {
    return matrix.array().isFinite().all();
}

double coordinate_scale(const std::vector<Point2D>& points) {
    double scale = 1.0;
    for (const Point2D& point : points) {
        scale = std::max(scale, std::abs(point.x));
        scale = std::max(scale, std::abs(point.y));
    }
    return scale;
}

bool has_duplicate_points(const std::vector<Point2D>& points) {
    const double tolerance = kDuplicateTolerance * coordinate_scale(points);
    for (std::size_t i = 0; i < points.size(); ++i) {
        for (std::size_t j = i + 1; j < points.size(); ++j) {
            if (std::hypot(points[i].x - points[j].x,
                           points[i].y - points[j].y) <= tolerance) {
                return true;
            }
        }
    }
    return false;
}

// The smaller/larger eigenvalue ratio of the centered 2D covariance is zero
// for collinear points and approaches zero for nearly collinear points.
bool has_two_dimensional_spread(const std::vector<Point2D>& points) {
    double centroid_x = 0.0;
    double centroid_y = 0.0;
    for (const Point2D& point : points) {
        centroid_x += point.x;
        centroid_y += point.y;
    }
    const double count = static_cast<double>(points.size());
    centroid_x /= count;
    centroid_y /= count;

    double xx = 0.0;
    double xy = 0.0;
    double yy = 0.0;
    for (const Point2D& point : points) {
        const double x = point.x - centroid_x;
        const double y = point.y - centroid_y;
        xx += x * x;
        xy += x * y;
        yy += y * y;
    }

    const double trace = xx + yy;
    if (!std::isfinite(trace) || trace <= std::numeric_limits<double>::min()) {
        return false;
    }

    const double discriminant = std::hypot(xx - yy, 2.0 * xy);
    const double largest = 0.5 * (trace + discriminant);
    const double smallest = 0.5 * (trace - discriminant);
    return smallest > kMinimumShapeRatio * largest;
}

Result<bool> validate_correspondences(
    const std::vector<Point2D>& source_points,
    const std::vector<Point2D>& destination_points,
    std::size_t minimum_count,
    const char* function_name) {
    if (source_points.size() != destination_points.size()) {
        return Result<bool>::failure(
            std::string(function_name) + ": source and destination sizes differ");
    }
    if (source_points.size() < minimum_count) {
        return Result<bool>::failure(
            std::string(function_name) + ": too few correspondences");
    }
    for (std::size_t i = 0; i < source_points.size(); ++i) {
        if (!is_finite(source_points[i]) || !is_finite(destination_points[i])) {
            return Result<bool>::failure(
                std::string(function_name) + ": correspondence " +
                std::to_string(i) + " contains a non-finite coordinate");
        }
    }
    return Result<bool>::success(true);
}

}  // namespace

Result<NormalizedPoints> normalize_points(const std::vector<Point2D>& points) {
    if (points.empty()) {
        return Result<NormalizedPoints>::failure(
            "normalize_points: point set is empty");
    }

    double centroid_x = 0.0;
    double centroid_y = 0.0;
    for (const Point2D& point : points) {
        if (!is_finite(point)) {
            return Result<NormalizedPoints>::failure(
                "normalize_points: point set contains a non-finite coordinate");
        }
        centroid_x += point.x;
        centroid_y += point.y;
    }
    const double count = static_cast<double>(points.size());
    centroid_x /= count;
    centroid_y /= count;

    double mean_distance = 0.0;
    for (const Point2D& point : points) {
        mean_distance += std::hypot(point.x - centroid_x,
                                    point.y - centroid_y);
    }
    mean_distance /= count;
    if (!std::isfinite(mean_distance) ||
        mean_distance <= std::numeric_limits<double>::epsilon() *
                             coordinate_scale(points)) {
        return Result<NormalizedPoints>::failure(
            "normalize_points: points have no usable spread");
    }

    const double scale = std::sqrt(2.0) / mean_distance;
    NormalizedPoints normalized;
    normalized.transform << scale, 0.0, -scale * centroid_x,
                            0.0, scale, -scale * centroid_y,
                            0.0, 0.0, 1.0;
    normalized.points.reserve(points.size());
    for (const Point2D& point : points) {
        normalized.points.push_back(
            {scale * (point.x - centroid_x),
             scale * (point.y - centroid_y)});
    }
    return Result<NormalizedPoints>::success(std::move(normalized));
}

Result<Point2D> apply_homography(const Eigen::Matrix3d& homography,
                                 const Point2D& point) {
    if (!all_finite(homography) || !is_finite(point)) {
        return Result<Point2D>::failure(
            "apply_homography: homography and point must be finite");
    }

    const Eigen::Vector3d input(point.x, point.y, 1.0);
    const Eigen::Vector3d mapped = homography * input;
    const double denominator_scale =
        homography.row(2).norm() * input.norm();
    if (!mapped.array().isFinite().all() ||
        denominator_scale <= std::numeric_limits<double>::min() ||
        std::abs(mapped.z()) <=
            kRelativeDenominatorTolerance * denominator_scale) {
        return Result<Point2D>::failure(
            "apply_homography: homogeneous denominator is zero or too small");
    }

    Point2D result{mapped.x() / mapped.z(), mapped.y() / mapped.z()};
    if (!is_finite(result)) {
        return Result<Point2D>::failure(
            "apply_homography: mapping produced a non-finite point");
    }
    return Result<Point2D>::success(result);
}

Result<Eigen::Matrix3d> estimate_homography(
    const std::vector<Point2D>& source_points,
    const std::vector<Point2D>& destination_points) {
    Result<bool> validation = validate_correspondences(
        source_points, destination_points, 4, "estimate_homography");
    if (!validation.ok) {
        return Result<Eigen::Matrix3d>::failure(validation.error);
    }
    if (has_duplicate_points(source_points) ||
        has_duplicate_points(destination_points)) {
        return Result<Eigen::Matrix3d>::failure(
            "estimate_homography: duplicate points are not allowed");
    }
    if (!has_two_dimensional_spread(source_points) ||
        !has_two_dimensional_spread(destination_points)) {
        return Result<Eigen::Matrix3d>::failure(
            "estimate_homography: points are collinear or nearly collinear");
    }

    Result<NormalizedPoints> source_normalized =
        normalize_points(source_points);
    Result<NormalizedPoints> destination_normalized =
        normalize_points(destination_points);
    if (!source_normalized.ok) {
        return Result<Eigen::Matrix3d>::failure(source_normalized.error);
    }
    if (!destination_normalized.ok) {
        return Result<Eigen::Matrix3d>::failure(destination_normalized.error);
    }

    const Eigen::Index count =
        static_cast<Eigen::Index>(source_points.size());
    Eigen::MatrixXd system(2 * count, 9);
    for (Eigen::Index i = 0; i < count; ++i) {
        const Point2D& source =
            source_normalized.value.points[static_cast<std::size_t>(i)];
        const Point2D& destination =
            destination_normalized.value.points[static_cast<std::size_t>(i)];
        system.row(2 * i) <<
            -source.x, -source.y, -1.0, 0.0, 0.0, 0.0,
            destination.x * source.x, destination.x * source.y,
            destination.x;
        system.row(2 * i + 1) <<
            0.0, 0.0, 0.0, -source.x, -source.y, -1.0,
            destination.y * source.x, destination.y * source.y,
            destination.y;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(system, Eigen::ComputeFullV);
    if (svd.matrixV().cols() != 9 || svd.singularValues().size() < 8) {
        return Result<Eigen::Matrix3d>::failure(
            "estimate_homography: SVD did not produce a usable null space");
    }

    // A valid four-point system has rank eight. For overdetermined noisy
    // systems the ninth singular value is the fitted residual, so rank is
    // assessed using the eighth singular value.
    const Eigen::VectorXd singular_values = svd.singularValues();
    const double rank_threshold =
        std::numeric_limits<double>::epsilon() *
        static_cast<double>(std::max(system.rows(), system.cols())) *
        singular_values(0);
    if (singular_values(7) <= rank_threshold) {
        return Result<Eigen::Matrix3d>::failure(
            "estimate_homography: DLT system is rank deficient");
    }

    const Eigen::VectorXd h = svd.matrixV().col(8);
    Eigen::Matrix3d normalized_homography;
    normalized_homography << h(0), h(1), h(2),
                              h(3), h(4), h(5),
                              h(6), h(7), h(8);

    Eigen::Matrix3d homography =
        destination_normalized.value.transform.inverse() *
        normalized_homography * source_normalized.value.transform;
    const double norm = homography.norm();
    if (!all_finite(homography) || !std::isfinite(norm) ||
        norm <= std::numeric_limits<double>::min()) {
        return Result<Eigen::Matrix3d>::failure(
            "estimate_homography: denormalization produced an invalid matrix");
    }
    homography /= norm;

    // Pick a deterministic representative of the scale/sign equivalence.
    Eigen::Index row = 0;
    Eigen::Index column = 0;
    homography.cwiseAbs().maxCoeff(&row, &column);
    if (homography(row, column) < 0.0) {
        homography = -homography;
    }
    return Result<Eigen::Matrix3d>::success(homography);
}

Result<std::vector<double>> homography_reprojection_errors(
    const Eigen::Matrix3d& homography,
    const std::vector<Point2D>& source_points,
    const std::vector<Point2D>& destination_points) {
    Result<bool> validation = validate_correspondences(
        source_points, destination_points, 1,
        "homography_reprojection_errors");
    if (!validation.ok) {
        return Result<std::vector<double>>::failure(validation.error);
    }
    if (!all_finite(homography)) {
        return Result<std::vector<double>>::failure(
            "homography_reprojection_errors: homography must be finite");
    }

    std::vector<double> errors;
    errors.reserve(source_points.size());
    for (std::size_t i = 0; i < source_points.size(); ++i) {
        Result<Point2D> mapped = apply_homography(homography, source_points[i]);
        if (!mapped.ok) {
            return Result<std::vector<double>>::failure(
                "homography_reprojection_errors: correspondence " +
                std::to_string(i) + ": " + mapped.error);
        }
        const double error = std::hypot(
            mapped.value.x - destination_points[i].x,
            mapped.value.y - destination_points[i].y);
        if (!std::isfinite(error)) {
            return Result<std::vector<double>>::failure(
                "homography_reprojection_errors: non-finite error");
        }
        errors.push_back(error);
    }
    return Result<std::vector<double>>::success(std::move(errors));
}

Result<double> homography_rms_reprojection_error(
    const Eigen::Matrix3d& homography,
    const std::vector<Point2D>& source_points,
    const std::vector<Point2D>& destination_points) {
    Result<std::vector<double>> errors = homography_reprojection_errors(
        homography, source_points, destination_points);
    if (!errors.ok) {
        return Result<double>::failure(errors.error);
    }

    double squared_error_sum = 0.0;
    for (double error : errors.value) {
        squared_error_sum += error * error;
    }
    const double rms =
        std::sqrt(squared_error_sum / static_cast<double>(errors.value.size()));
    if (!std::isfinite(rms)) {
        return Result<double>::failure(
            "homography_rms_reprojection_error: non-finite RMS error");
    }
    return Result<double>::success(rms);
}

}  // namespace clean_calib::calib
