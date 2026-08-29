#include "clean_calib/calib/refinement.h"

#include "clean_calib/calib/projection.h"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace clean_calib::calib {
namespace {

constexpr Eigen::Index kCameraParameterCount = 10;
constexpr Eigen::Index kPoseParameterCount = 6;

struct CalibrationState {
    CameraModel camera;
    std::vector<Pose> poses;
};

bool is_finite(const Point2D& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

bool is_finite(const Point3D& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
}

bool is_finite(const CameraModel& camera) {
    const CameraIntrinsics& k = camera.intrinsics;
    const Distortion& d = camera.distortion;
    return std::isfinite(k.fx) && std::isfinite(k.fy) &&
           std::isfinite(k.cx) && std::isfinite(k.cy) &&
           std::isfinite(k.skew) && std::isfinite(d.k1) &&
           std::isfinite(d.k2) && std::isfinite(d.k3) &&
           std::isfinite(d.p1) && std::isfinite(d.p2);
}

Result<bool> validate_problem(
    const std::vector<Point3D>& object_points,
    const std::vector<std::vector<Point2D>>& image_points,
    const CameraModel& camera,
    const std::vector<Pose>& poses) {
    if (object_points.empty()) {
        return Result<bool>::failure(
            "calibration refinement: object point list is empty");
    }
    if (image_points.empty()) {
        return Result<bool>::failure(
            "calibration refinement: no image views were provided");
    }
    if (image_points.size() != poses.size()) {
        return Result<bool>::failure(
            "calibration refinement: image-view and pose counts differ");
    }
    if (!is_finite(camera) || camera.intrinsics.fx <= 0.0 ||
        camera.intrinsics.fy <= 0.0) {
        return Result<bool>::failure(
            "calibration refinement: camera parameters are invalid");
    }
    for (const Point3D& point : object_points) {
        if (!is_finite(point)) {
            return Result<bool>::failure(
                "calibration refinement: object point is non-finite");
        }
    }
    for (std::size_t view = 0; view < image_points.size(); ++view) {
        if (image_points[view].size() != object_points.size()) {
            return Result<bool>::failure(
                "calibration refinement: view " + std::to_string(view) +
                " has the wrong observation count");
        }
        if (!poses[view].R.array().isFinite().all() ||
            !poses[view].t.array().isFinite().all()) {
            return Result<bool>::failure(
                "calibration refinement: pose " + std::to_string(view) +
                " is non-finite");
        }
        for (const Point2D& point : image_points[view]) {
            if (!is_finite(point)) {
                return Result<bool>::failure(
                    "calibration refinement: image observation is non-finite");
            }
        }
    }
    return Result<bool>::success(true);
}

Result<bool> validate_options(const RefinementOptions& options,
                              bool require_damping) {
    if (options.max_iterations <= 0 ||
        !std::isfinite(options.finite_difference_step) ||
        options.finite_difference_step <= 0.0 ||
        !std::isfinite(options.step_tolerance) ||
        options.step_tolerance < 0.0 ||
        !std::isfinite(options.cost_tolerance) ||
        options.cost_tolerance < 0.0 ||
        !std::isfinite(options.gradient_tolerance) ||
        options.gradient_tolerance < 0.0 ||
        (require_damping &&
         (!std::isfinite(options.initial_damping) ||
          options.initial_damping <= 0.0))) {
        return Result<bool>::failure(
            "calibration refinement: optimizer options are invalid");
    }
    return Result<bool>::success(true);
}

Eigen::Vector3d rotation_vector(const Eigen::Matrix3d& rotation) {
    Eigen::AngleAxisd angle_axis(rotation);
    if (!std::isfinite(angle_axis.angle()) ||
        angle_axis.angle() <= std::numeric_limits<double>::epsilon()) {
        return Eigen::Vector3d::Zero();
    }
    return angle_axis.angle() * angle_axis.axis();
}

Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d& vector) {
    const double angle = vector.norm();
    if (angle <= std::numeric_limits<double>::epsilon()) {
        return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd(angle, vector / angle).toRotationMatrix();
}

Eigen::VectorXd pack_state(const CameraModel& camera,
                           const std::vector<Pose>& poses) {
    Eigen::VectorXd parameters(
        kCameraParameterCount +
        kPoseParameterCount * static_cast<Eigen::Index>(poses.size()));
    parameters << camera.intrinsics.fx, camera.intrinsics.fy,
                  camera.intrinsics.cx, camera.intrinsics.cy,
                  camera.intrinsics.skew,
                  camera.distortion.k1, camera.distortion.k2,
                  camera.distortion.k3, camera.distortion.p1,
                  camera.distortion.p2,
                  Eigen::VectorXd::Zero(
                      kPoseParameterCount *
                      static_cast<Eigen::Index>(poses.size()));
    for (std::size_t view = 0; view < poses.size(); ++view) {
        const Eigen::Index offset =
            kCameraParameterCount +
            kPoseParameterCount * static_cast<Eigen::Index>(view);
        parameters.segment<3>(offset) = rotation_vector(poses[view].R);
        parameters.segment<3>(offset + 3) = poses[view].t;
    }
    return parameters;
}

Result<CalibrationState> unpack_state(const Eigen::VectorXd& parameters,
                                      std::size_t view_count) {
    const Eigen::Index expected_size =
        kCameraParameterCount +
        kPoseParameterCount * static_cast<Eigen::Index>(view_count);
    if (parameters.size() != expected_size ||
        !parameters.array().isFinite().all()) {
        return Result<CalibrationState>::failure(
            "calibration refinement: parameter vector is invalid");
    }

    CalibrationState state;
    state.camera.intrinsics.fx = parameters(0);
    state.camera.intrinsics.fy = parameters(1);
    state.camera.intrinsics.cx = parameters(2);
    state.camera.intrinsics.cy = parameters(3);
    state.camera.intrinsics.skew = parameters(4);
    state.camera.distortion.k1 = parameters(5);
    state.camera.distortion.k2 = parameters(6);
    state.camera.distortion.k3 = parameters(7);
    state.camera.distortion.p1 = parameters(8);
    state.camera.distortion.p2 = parameters(9);
    if (!is_finite(state.camera) || state.camera.intrinsics.fx <= 0.0 ||
        state.camera.intrinsics.fy <= 0.0) {
        return Result<CalibrationState>::failure(
            "calibration refinement: trial focal lengths are non-physical");
    }

    state.poses.resize(view_count);
    for (std::size_t view = 0; view < view_count; ++view) {
        const Eigen::Index offset =
            kCameraParameterCount +
            kPoseParameterCount * static_cast<Eigen::Index>(view);
        state.poses[view].R =
            rotation_matrix(parameters.segment<3>(offset));
        state.poses[view].t = parameters.segment<3>(offset + 3);
    }
    return Result<CalibrationState>::success(std::move(state));
}

Result<Eigen::VectorXd> residuals_from_parameters(
    const std::vector<Point3D>& object_points,
    const std::vector<std::vector<Point2D>>& image_points,
    const Eigen::VectorXd& parameters) {
    Result<CalibrationState> state =
        unpack_state(parameters, image_points.size());
    if (!state.ok) {
        return Result<Eigen::VectorXd>::failure(state.error);
    }

    const Eigen::Index residual_count =
        2 * static_cast<Eigen::Index>(object_points.size()) *
        static_cast<Eigen::Index>(image_points.size());
    Eigen::VectorXd residuals(residual_count);
    Eigen::Index index = 0;
    for (std::size_t view = 0; view < image_points.size(); ++view) {
        for (std::size_t point = 0; point < object_points.size(); ++point) {
            Result<Point2D> projected = project_point(
                object_points[point], state.value.poses[view],
                state.value.camera);
            if (!projected.ok || !is_finite(projected.value)) {
                return Result<Eigen::VectorXd>::failure(
                    "calibration refinement: projection failed for view " +
                    std::to_string(view) + ", point " +
                    std::to_string(point));
            }
            residuals(index++) =
                projected.value.x - image_points[view][point].x;
            residuals(index++) =
                projected.value.y - image_points[view][point].y;
        }
    }
    if (!residuals.array().isFinite().all()) {
        return Result<Eigen::VectorXd>::failure(
            "calibration refinement: residual vector is non-finite");
    }
    return Result<Eigen::VectorXd>::success(std::move(residuals));
}

Result<Eigen::MatrixXd> jacobian_from_parameters(
    const std::vector<Point3D>& object_points,
    const std::vector<std::vector<Point2D>>& image_points,
    const Eigen::VectorXd& parameters,
    double relative_step) {
    if (!std::isfinite(relative_step) || relative_step <= 0.0) {
        return Result<Eigen::MatrixXd>::failure(
            "numeric_calibration_jacobian: step must be finite and positive");
    }
    Result<Eigen::VectorXd> base = residuals_from_parameters(
        object_points, image_points, parameters);
    if (!base.ok) {
        return Result<Eigen::MatrixXd>::failure(base.error);
    }

    Eigen::MatrixXd jacobian(base.value.size(), parameters.size());
    for (Eigen::Index column = 0; column < parameters.size(); ++column) {
        const double step = relative_step *
                            std::max(1.0, std::abs(parameters(column)));
        Eigen::VectorXd plus = parameters;
        Eigen::VectorXd minus = parameters;
        plus(column) += step;
        minus(column) -= step;
        Result<Eigen::VectorXd> plus_residuals = residuals_from_parameters(
            object_points, image_points, plus);
        Result<Eigen::VectorXd> minus_residuals = residuals_from_parameters(
            object_points, image_points, minus);
        if (!plus_residuals.ok || !minus_residuals.ok) {
            return Result<Eigen::MatrixXd>::failure(
                "numeric_calibration_jacobian: perturbation failed at parameter " +
                std::to_string(column));
        }
        jacobian.col(column) =
            (plus_residuals.value - minus_residuals.value) / (2.0 * step);
    }
    if (!jacobian.array().isFinite().all()) {
        return Result<Eigen::MatrixXd>::failure(
            "numeric_calibration_jacobian: result is non-finite");
    }
    return Result<Eigen::MatrixXd>::success(std::move(jacobian));
}

double rms_from_residuals(const Eigen::VectorXd& residuals) {
    const double observation_count =
        static_cast<double>(residuals.size() / 2);
    return std::sqrt(residuals.squaredNorm() / observation_count);
}

Result<RefinementResult> make_result(
    const Eigen::VectorXd& parameters,
    std::size_t view_count,
    double initial_rms,
    const Eigen::VectorXd& final_residuals,
    int iterations,
    bool converged) {
    Result<CalibrationState> state = unpack_state(parameters, view_count);
    if (!state.ok) {
        return Result<RefinementResult>::failure(state.error);
    }
    RefinementResult result;
    result.camera = state.value.camera;
    result.poses = std::move(state.value.poses);
    result.initial_rms_reprojection_error = initial_rms;
    result.final_rms_reprojection_error = rms_from_residuals(final_residuals);
    result.iterations = iterations;
    result.converged = converged;
    return Result<RefinementResult>::success(std::move(result));
}

}  // namespace

Result<Eigen::VectorXd> calibration_residuals(
    const std::vector<Point3D>& object_points,
    const std::vector<std::vector<Point2D>>& image_points,
    const CameraModel& camera,
    const std::vector<Pose>& poses) {
    Result<bool> validation =
        validate_problem(object_points, image_points, camera, poses);
    if (!validation.ok) {
        return Result<Eigen::VectorXd>::failure(validation.error);
    }
    return residuals_from_parameters(
        object_points, image_points, pack_state(camera, poses));
}

Result<Eigen::MatrixXd> numeric_calibration_jacobian(
    const std::vector<Point3D>& object_points,
    const std::vector<std::vector<Point2D>>& image_points,
    const CameraModel& camera,
    const std::vector<Pose>& poses,
    double relative_step) {
    Result<bool> validation =
        validate_problem(object_points, image_points, camera, poses);
    if (!validation.ok) {
        return Result<Eigen::MatrixXd>::failure(validation.error);
    }
    return jacobian_from_parameters(
        object_points, image_points, pack_state(camera, poses), relative_step);
}

Result<RefinementResult> refine_calibration_gauss_newton(
    const std::vector<Point3D>& object_points,
    const std::vector<std::vector<Point2D>>& image_points,
    const CameraModel& initial_camera,
    const std::vector<Pose>& initial_poses,
    const RefinementOptions& options) {
    Result<bool> problem_validation = validate_problem(
        object_points, image_points, initial_camera, initial_poses);
    if (!problem_validation.ok) {
        return Result<RefinementResult>::failure(problem_validation.error);
    }
    Result<bool> option_validation = validate_options(options, false);
    if (!option_validation.ok) {
        return Result<RefinementResult>::failure(option_validation.error);
    }

    Eigen::VectorXd parameters = pack_state(initial_camera, initial_poses);
    Result<Eigen::VectorXd> residual_result = residuals_from_parameters(
        object_points, image_points, parameters);
    if (!residual_result.ok) {
        return Result<RefinementResult>::failure(residual_result.error);
    }
    Eigen::VectorXd residuals = residual_result.value;
    const double initial_rms = rms_from_residuals(residuals);
    bool converged = false;
    int iterations = 0;

    for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
        Result<Eigen::MatrixXd> jacobian_result = jacobian_from_parameters(
            object_points, image_points, parameters,
            options.finite_difference_step);
        if (!jacobian_result.ok) {
            return Result<RefinementResult>::failure(jacobian_result.error);
        }
        const Eigen::MatrixXd& jacobian = jacobian_result.value;
        const Eigen::VectorXd gradient = jacobian.transpose() * residuals;
        if (gradient.lpNorm<Eigen::Infinity>() <=
            options.gradient_tolerance) {
            converged = true;
            break;
        }

        const Eigen::VectorXd step =
            jacobian.colPivHouseholderQr().solve(-residuals);
        if (!step.array().isFinite().all()) {
            return Result<RefinementResult>::failure(
                "refine_calibration_gauss_newton: non-finite step");
        }
        if (step.norm() <= options.step_tolerance *
                               (parameters.norm() + options.step_tolerance)) {
            converged = true;
            break;
        }

        const double old_cost = 0.5 * residuals.squaredNorm();
        Eigen::VectorXd trial_parameters = parameters + step;
        Result<Eigen::VectorXd> trial_residuals = residuals_from_parameters(
            object_points, image_points, trial_parameters);
        if (!trial_residuals.ok) {
            return Result<RefinementResult>::failure(
                "refine_calibration_gauss_newton: step left the valid parameter domain");
        }
        const double new_cost = 0.5 * trial_residuals.value.squaredNorm();
        parameters = std::move(trial_parameters);
        residuals = std::move(trial_residuals.value);
        iterations = iteration + 1;
        if (std::abs(old_cost - new_cost) <=
            options.cost_tolerance * std::max(1.0, old_cost)) {
            converged = true;
            break;
        }
    }
    return make_result(parameters, initial_poses.size(), initial_rms,
                       residuals, iterations, converged);
}

Result<RefinementResult> refine_calibration_levenberg_marquardt(
    const std::vector<Point3D>& object_points,
    const std::vector<std::vector<Point2D>>& image_points,
    const CameraModel& initial_camera,
    const std::vector<Pose>& initial_poses,
    const RefinementOptions& options) {
    Result<bool> problem_validation = validate_problem(
        object_points, image_points, initial_camera, initial_poses);
    if (!problem_validation.ok) {
        return Result<RefinementResult>::failure(problem_validation.error);
    }
    Result<bool> option_validation = validate_options(options, true);
    if (!option_validation.ok) {
        return Result<RefinementResult>::failure(option_validation.error);
    }

    Eigen::VectorXd parameters = pack_state(initial_camera, initial_poses);
    Result<Eigen::VectorXd> residual_result = residuals_from_parameters(
        object_points, image_points, parameters);
    if (!residual_result.ok) {
        return Result<RefinementResult>::failure(residual_result.error);
    }
    Eigen::VectorXd residuals = residual_result.value;
    const double initial_rms = rms_from_residuals(residuals);
    double damping = options.initial_damping;
    double rejection_multiplier = 2.0;
    bool converged = false;
    int iterations = 0;

    for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
        Result<Eigen::MatrixXd> jacobian_result = jacobian_from_parameters(
            object_points, image_points, parameters,
            options.finite_difference_step);
        if (!jacobian_result.ok) {
            return Result<RefinementResult>::failure(jacobian_result.error);
        }
        const Eigen::MatrixXd& jacobian = jacobian_result.value;
        const Eigen::MatrixXd normal = jacobian.transpose() * jacobian;
        const Eigen::VectorXd gradient = jacobian.transpose() * residuals;
        if (gradient.lpNorm<Eigen::Infinity>() <=
            options.gradient_tolerance) {
            converged = true;
            break;
        }

        Eigen::VectorXd diagonal = normal.diagonal().cwiseMax(1e-12);
        Eigen::MatrixXd damped = normal;
        damped.diagonal() += damping * diagonal;
        Eigen::LDLT<Eigen::MatrixXd> solver(damped);
        if (solver.info() != Eigen::Success) {
            return Result<RefinementResult>::failure(
                "refine_calibration_levenberg_marquardt: linear solve failed");
        }
        const Eigen::VectorXd step = solver.solve(-gradient);
        if (solver.info() != Eigen::Success ||
            !step.array().isFinite().all()) {
            return Result<RefinementResult>::failure(
                "refine_calibration_levenberg_marquardt: invalid step");
        }
        if (step.norm() <= options.step_tolerance *
                               (parameters.norm() + options.step_tolerance)) {
            converged = true;
            break;
        }

        const double old_cost = 0.5 * residuals.squaredNorm();
        const double predicted_reduction =
            -step.dot(gradient) - 0.5 * step.dot(normal * step);
        Eigen::VectorXd trial_parameters = parameters + step;
        Result<Eigen::VectorXd> trial_residuals = residuals_from_parameters(
            object_points, image_points, trial_parameters);
        double gain_ratio = -1.0;
        double new_cost = std::numeric_limits<double>::infinity();
        if (trial_residuals.ok && predicted_reduction > 0.0) {
            new_cost = 0.5 * trial_residuals.value.squaredNorm();
            gain_ratio = (old_cost - new_cost) / predicted_reduction;
        }

        if (gain_ratio > 0.0 && std::isfinite(new_cost)) {
            parameters = std::move(trial_parameters);
            residuals = std::move(trial_residuals.value);
            const double update =
                std::max(1.0 / 3.0,
                         1.0 - std::pow(2.0 * gain_ratio - 1.0, 3.0));
            damping = std::max(1e-18, damping * update);
            rejection_multiplier = 2.0;
            iterations = iteration + 1;
            if (old_cost - new_cost <=
                options.cost_tolerance * std::max(1.0, old_cost)) {
                converged = true;
                break;
            }
        } else {
            damping *= rejection_multiplier;
            rejection_multiplier *= 2.0;
            iterations = iteration + 1;
            if (!std::isfinite(damping)) {
                return Result<RefinementResult>::failure(
                    "refine_calibration_levenberg_marquardt: damping overflow");
            }
        }
    }
    return make_result(parameters, initial_poses.size(), initial_rms,
                       residuals, iterations, converged);
}

}  // namespace clean_calib::calib
