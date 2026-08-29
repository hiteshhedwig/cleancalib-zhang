#include "clean_calib/calib/zhang.h"

#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace clean_calib::calib {
namespace {

bool is_finite(const Eigen::Matrix3d& matrix) {
    return matrix.array().isFinite().all();
}

bool is_finite(const CameraIntrinsics& intrinsics) {
    return std::isfinite(intrinsics.fx) &&
           std::isfinite(intrinsics.fy) &&
           std::isfinite(intrinsics.cx) &&
           std::isfinite(intrinsics.cy) &&
           std::isfinite(intrinsics.skew);
}

Eigen::Matrix3d camera_matrix(const CameraIntrinsics& intrinsics) {
    Eigen::Matrix3d matrix;
    matrix << intrinsics.fx, intrinsics.skew, intrinsics.cx,
              0.0, intrinsics.fy, intrinsics.cy,
              0.0, 0.0, 1.0;
    return matrix;
}

Eigen::Matrix<double, 6, 1> constraint_vector(
    const Eigen::Matrix3d& homography,
    int first_column,
    int second_column) {
    const Eigen::Vector3d first = homography.col(first_column);
    const Eigen::Vector3d second = homography.col(second_column);
    Eigen::Matrix<double, 6, 1> result;
    result << first(0) * second(0),
              first(0) * second(1) + first(1) * second(0),
              first(1) * second(1),
              first(2) * second(0) + first(0) * second(2),
              first(2) * second(1) + first(1) * second(2),
              first(2) * second(2);
    return result;
}

Result<bool> validate_homographies(
    const std::vector<Eigen::Matrix3d>& homographies,
    std::size_t minimum_count,
    const char* function_name) {
    if (homographies.size() < minimum_count) {
        return Result<bool>::failure(
            std::string(function_name) + ": too few homographies");
    }
    for (std::size_t i = 0; i < homographies.size(); ++i) {
        const double norm = homographies[i].norm();
        if (!is_finite(homographies[i]) || !std::isfinite(norm) ||
            norm <= std::numeric_limits<double>::min()) {
            return Result<bool>::failure(
                std::string(function_name) + ": homography " +
                std::to_string(i) + " is not a finite nonzero matrix");
        }
    }
    return Result<bool>::success(true);
}

}  // namespace

Result<Eigen::MatrixXd> build_zhang_constraint_matrix(
    const std::vector<Eigen::Matrix3d>& homographies) {
    Result<bool> validation = validate_homographies(
        homographies, 3, "build_zhang_constraint_matrix");
    if (!validation.ok) {
        return Result<Eigen::MatrixXd>::failure(validation.error);
    }

    Eigen::MatrixXd constraints(
        2 * static_cast<Eigen::Index>(homographies.size()), 6);
    for (std::size_t i = 0; i < homographies.size(); ++i) {
        // Normalizing each H independently is legal because both constraints
        // contributed by one view are homogeneous in H and scale by H^2.
        const Eigen::Matrix3d homography =
            homographies[i] / homographies[i].norm();
        const Eigen::Matrix<double, 6, 1> v12 =
            constraint_vector(homography, 0, 1);
        const Eigen::Matrix<double, 6, 1> v11 =
            constraint_vector(homography, 0, 0);
        const Eigen::Matrix<double, 6, 1> v22 =
            constraint_vector(homography, 1, 1);
        constraints.row(2 * static_cast<Eigen::Index>(i)) = v12.transpose();
        constraints.row(2 * static_cast<Eigen::Index>(i) + 1) =
            (v11 - v22).transpose();
    }
    return Result<Eigen::MatrixXd>::success(std::move(constraints));
}

Result<CameraIntrinsics> estimate_intrinsics_from_homographies(
    const std::vector<Eigen::Matrix3d>& homographies) {
    Result<Eigen::MatrixXd> constraint_result =
        build_zhang_constraint_matrix(homographies);
    if (!constraint_result.ok) {
        return Result<CameraIntrinsics>::failure(constraint_result.error);
    }

    const Eigen::MatrixXd& constraints = constraint_result.value;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(constraints, Eigen::ComputeFullV);
    if (svd.matrixV().cols() != 6 || svd.singularValues().size() < 6) {
        return Result<CameraIntrinsics>::failure(
            "estimate_intrinsics_from_homographies: SVD failed");
    }

    const Eigen::VectorXd singular_values = svd.singularValues();
    const double rank_threshold =
        std::numeric_limits<double>::epsilon() *
        static_cast<double>(std::max(constraints.rows(), constraints.cols())) *
        singular_values(0);
    if (singular_values(4) <= rank_threshold) {
        return Result<CameraIntrinsics>::failure(
            "estimate_intrinsics_from_homographies: constraint matrix is rank deficient; views lack sufficient pose diversity");
    }

    const Eigen::VectorXd b = svd.matrixV().col(5);
    Eigen::Matrix3d image_conic;
    image_conic << b(0), b(1), b(3),
                   b(1), b(2), b(4),
                   b(3), b(4), b(5);

    // B is defined only up to scale. Choose its positive-definite sign before
    // applying Zhang's closed-form formulas.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(image_conic);
    if (eigen_solver.info() != Eigen::Success) {
        return Result<CameraIntrinsics>::failure(
            "estimate_intrinsics_from_homographies: invalid image conic");
    }
    const Eigen::Vector3d eigenvalues = eigen_solver.eigenvalues();
    if (eigenvalues.maxCoeff() < 0.0) {
        image_conic = -image_conic;
    } else if (eigenvalues.minCoeff() <= 0.0) {
        return Result<CameraIntrinsics>::failure(
            "estimate_intrinsics_from_homographies: recovered image conic is not positive definite");
    }

    const double b11 = image_conic(0, 0);
    const double b12 = image_conic(0, 1);
    const double b22 = image_conic(1, 1);
    const double b13 = image_conic(0, 2);
    const double b23 = image_conic(1, 2);
    const double b33 = image_conic(2, 2);
    const double denominator = b11 * b22 - b12 * b12;
    const double conic_scale = image_conic.norm();
    if (!std::isfinite(denominator) ||
        denominator <= std::numeric_limits<double>::epsilon() *
                           conic_scale * conic_scale ||
        b11 <= std::numeric_limits<double>::epsilon() * conic_scale) {
        return Result<CameraIntrinsics>::failure(
            "estimate_intrinsics_from_homographies: singular intrinsic constraints");
    }

    const double cy = (b12 * b13 - b11 * b23) / denominator;
    const double lambda =
        b33 - (b13 * b13 + cy * (b12 * b13 - b11 * b23)) / b11;
    const double fx_squared = lambda / b11;
    const double fy_squared = lambda * b11 / denominator;
    if (!std::isfinite(lambda) || fx_squared <= 0.0 || fy_squared <= 0.0) {
        return Result<CameraIntrinsics>::failure(
            "estimate_intrinsics_from_homographies: non-physical focal lengths");
    }

    CameraIntrinsics intrinsics;
    intrinsics.fx = std::sqrt(fx_squared);
    intrinsics.fy = std::sqrt(fy_squared);
    intrinsics.skew =
        -b12 * intrinsics.fx * intrinsics.fx * intrinsics.fy / lambda;
    intrinsics.cx =
        intrinsics.skew * cy / intrinsics.fy -
        b13 * intrinsics.fx * intrinsics.fx / lambda;
    intrinsics.cy = cy;
    if (!is_finite(intrinsics) || intrinsics.fx <= 0.0 ||
        intrinsics.fy <= 0.0) {
        return Result<CameraIntrinsics>::failure(
            "estimate_intrinsics_from_homographies: invalid intrinsics");
    }
    return Result<CameraIntrinsics>::success(intrinsics);
}

Result<Pose> estimate_pose_from_homography(
    const Eigen::Matrix3d& homography,
    const CameraIntrinsics& intrinsics) {
    if (!is_finite(homography) ||
        homography.norm() <= std::numeric_limits<double>::min()) {
        return Result<Pose>::failure(
            "estimate_pose_from_homography: homography must be finite and nonzero");
    }
    if (!is_finite(intrinsics) || intrinsics.fx <= 0.0 ||
        intrinsics.fy <= 0.0) {
        return Result<Pose>::failure(
            "estimate_pose_from_homography: intrinsics are invalid");
    }

    const Eigen::Matrix3d inverse_intrinsics = camera_matrix(intrinsics).inverse();
    const Eigen::Vector3d first = inverse_intrinsics * homography.col(0);
    const Eigen::Vector3d second = inverse_intrinsics * homography.col(1);
    const Eigen::Vector3d translation = inverse_intrinsics * homography.col(2);
    const double first_norm = first.norm();
    const double second_norm = second.norm();
    if (!first.array().isFinite().all() || !second.array().isFinite().all() ||
        !translation.array().isFinite().all() ||
        first_norm <= std::numeric_limits<double>::min() ||
        second_norm <= std::numeric_limits<double>::min()) {
        return Result<Pose>::failure(
            "estimate_pose_from_homography: degenerate homography columns");
    }

    double scale = 2.0 / (first_norm + second_norm);
    if (scale * translation.z() < 0.0) {
        scale = -scale;
    }
    const Eigen::Vector3d r1 = scale * first;
    const Eigen::Vector3d r2 = scale * second;
    Eigen::Matrix3d approximate_rotation;
    approximate_rotation.col(0) = r1;
    approximate_rotation.col(1) = r2;
    approximate_rotation.col(2) = r1.cross(r2);

    Eigen::JacobiSVD<Eigen::Matrix3d> rotation_svd(
        approximate_rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) =
        (rotation_svd.matrixU() * rotation_svd.matrixV().transpose()).determinant();

    Pose pose;
    pose.R = rotation_svd.matrixU() * correction *
             rotation_svd.matrixV().transpose();
    pose.t = scale * translation;
    if (!is_finite(pose.R) || !pose.t.array().isFinite().all() ||
        pose.R.determinant() <= 0.0) {
        return Result<Pose>::failure(
            "estimate_pose_from_homography: pose recovery failed");
    }
    return Result<Pose>::success(pose);
}

Result<ZhangInitialization> initialize_zhang(
    const std::vector<Eigen::Matrix3d>& homographies) {
    Result<CameraIntrinsics> intrinsics =
        estimate_intrinsics_from_homographies(homographies);
    if (!intrinsics.ok) {
        return Result<ZhangInitialization>::failure(intrinsics.error);
    }

    ZhangInitialization initialization;
    initialization.intrinsics = intrinsics.value;
    initialization.poses.reserve(homographies.size());
    for (std::size_t i = 0; i < homographies.size(); ++i) {
        Result<Pose> pose = estimate_pose_from_homography(
            homographies[i], initialization.intrinsics);
        if (!pose.ok) {
            return Result<ZhangInitialization>::failure(
                "initialize_zhang: failed to recover pose " +
                std::to_string(i) + ": " + pose.error);
        }
        initialization.poses.push_back(pose.value);
    }
    return Result<ZhangInitialization>::success(std::move(initialization));
}

}  // namespace clean_calib::calib
