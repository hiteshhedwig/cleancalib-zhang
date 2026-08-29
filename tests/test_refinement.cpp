#include "clean_calib/calib/refinement.h"
#include "clean_calib/synthetic/checkerboard.h"
#include "clean_calib/synthetic/dataset.h"
#include "test_harness.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <vector>

using clean_calib::CameraModel;
using clean_calib::Point2D;
using clean_calib::Point3D;
using clean_calib::Pose;
using clean_calib::Result;
using clean_calib::calib::RefinementOptions;
using clean_calib::calib::RefinementResult;
using clean_calib::calib::calibration_residuals;
using clean_calib::calib::numeric_calibration_jacobian;
using clean_calib::calib::refine_calibration_gauss_newton;
using clean_calib::calib::refine_calibration_levenberg_marquardt;
using clean_calib::synthetic::project_board;

namespace {

struct SyntheticProblem {
    std::vector<Point3D> object_points;
    std::vector<std::vector<Point2D>> image_points;
    CameraModel camera;
    std::vector<Pose> poses;
};

CameraModel make_camera(bool with_distortion) {
    CameraModel camera;
    camera.intrinsics.fx = 780.0;
    camera.intrinsics.fy = 805.0;
    camera.intrinsics.cx = 321.0;
    camera.intrinsics.cy = 238.0;
    camera.intrinsics.skew = 0.8;
    if (with_distortion) {
        camera.distortion.k1 = -0.10;
        camera.distortion.k2 = 0.025;
        camera.distortion.k3 = -0.003;
        camera.distortion.p1 = 0.0015;
        camera.distortion.p2 = -0.001;
    }
    return camera;
}

Pose make_pose(double rx,
               double ry,
               double rz,
               double tx,
               double ty,
               double tz) {
    Pose pose;
    pose.R = (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX())).toRotationMatrix();
    pose.t << tx, ty, tz;
    return pose;
}

std::vector<Pose> make_poses() {
    return {
        make_pose(0.12, -0.18, 0.03, -0.12, -0.08, 0.95),
        make_pose(-0.22, 0.14, -0.08, 0.08, -0.05, 1.10),
        make_pose(0.28, 0.09, 0.12, -0.07, 0.10, 0.90),
        make_pose(-0.10, -0.31, 0.17, 0.13, 0.04, 1.20),
        make_pose(0.19, 0.25, -0.15, -0.09, 0.02, 1.02),
        make_pose(-0.27, -0.08, 0.06, 0.04, -0.09, 1.25)};
}

SyntheticProblem make_problem(bool with_distortion) {
    SyntheticProblem problem;
    problem.object_points =
        clean_calib::synthetic::generate_planar_board(7, 9, 0.04);
    problem.camera = make_camera(with_distortion);
    problem.poses = make_poses();
    for (const Pose& pose : problem.poses) {
        Result<std::vector<Point2D>> projected =
            project_board(problem.object_points, pose, problem.camera);
        if (!projected.ok) {
            return {};
        }
        problem.image_points.push_back(projected.value);
    }
    return problem;
}

CameraModel perturb_camera(const CameraModel& truth, bool include_distortion) {
    CameraModel initial = truth;
    initial.intrinsics.fx *= 1.015;
    initial.intrinsics.fy *= 0.985;
    initial.intrinsics.cx += 3.0;
    initial.intrinsics.cy -= 2.5;
    initial.intrinsics.skew += 0.4;
    if (include_distortion) {
        initial.distortion.k1 *= 0.65;
        initial.distortion.k2 *= 0.4;
        initial.distortion.k3 = 0.0;
        initial.distortion.p1 *= 0.5;
        initial.distortion.p2 *= 0.5;
    }
    return initial;
}

std::vector<Pose> perturb_poses(const std::vector<Pose>& truth,
                                double magnitude) {
    std::vector<Pose> initial = truth;
    const Eigen::Vector3d axes[] = {
        {1.0, 0.2, -0.1}, {-0.3, 1.0, 0.1}, {0.2, -0.4, 1.0}};
    for (std::size_t i = 0; i < initial.size(); ++i) {
        const Eigen::Vector3d axis = axes[i % 3].normalized();
        initial[i].R = Eigen::AngleAxisd(
                           magnitude * (i % 2 == 0 ? 1.0 : -1.0), axis)
                           .toRotationMatrix() *
                       initial[i].R;
        initial[i].t.x() += magnitude * 0.4;
        initial[i].t.y() -= magnitude * 0.25;
        initial[i].t.z() += magnitude * 0.3;
    }
    return initial;
}

double camera_parameter_error(const CameraModel& actual,
                              const CameraModel& truth) {
    Eigen::Matrix<double, 10, 1> difference;
    difference <<
        (actual.intrinsics.fx - truth.intrinsics.fx) / truth.intrinsics.fx,
        (actual.intrinsics.fy - truth.intrinsics.fy) / truth.intrinsics.fy,
        (actual.intrinsics.cx - truth.intrinsics.cx) / truth.intrinsics.fx,
        (actual.intrinsics.cy - truth.intrinsics.cy) / truth.intrinsics.fy,
        (actual.intrinsics.skew - truth.intrinsics.skew) / truth.intrinsics.fx,
        actual.distortion.k1 - truth.distortion.k1,
        actual.distortion.k2 - truth.distortion.k2,
        actual.distortion.k3 - truth.distortion.k3,
        actual.distortion.p1 - truth.distortion.p1,
        actual.distortion.p2 - truth.distortion.p2;
    return difference.norm();
}

CC_TEST(refinement_truth_has_zero_residuals_and_expected_order) {
    const SyntheticProblem problem = make_problem(true);
    Result<Eigen::VectorXd> residuals = calibration_residuals(
        problem.object_points, problem.image_points,
        problem.camera, problem.poses);
    CC_EXPECT_TRUE(residuals.ok);
    CC_EXPECT_EQ(
        residuals.value.size(),
        2 * static_cast<Eigen::Index>(problem.object_points.size()) *
            static_cast<Eigen::Index>(problem.poses.size()));
    CC_EXPECT_NEAR(residuals.value.norm(), 0.0, 1e-12);
}

CC_TEST(refinement_numeric_jacobian_has_expected_shape_and_cx_column) {
    SyntheticProblem problem = make_problem(false);
    problem.poses.resize(3);
    problem.image_points.resize(3);
    Result<Eigen::MatrixXd> jacobian = numeric_calibration_jacobian(
        problem.object_points, problem.image_points,
        problem.camera, problem.poses, 1e-6);
    CC_EXPECT_TRUE(jacobian.ok);
    const Eigen::Index expected_rows =
        2 * static_cast<Eigen::Index>(problem.object_points.size()) * 3;
    const Eigen::Index expected_columns = 10 + 6 * 3;
    CC_EXPECT_EQ(jacobian.value.rows(), expected_rows);
    CC_EXPECT_EQ(jacobian.value.cols(), expected_columns);
    CC_EXPECT_TRUE(jacobian.value.array().isFinite().all());
    for (Eigen::Index row = 0; row < expected_rows; row += 2) {
        CC_EXPECT_NEAR(jacobian.value(row, 2), 1.0, 1e-9);
        CC_EXPECT_NEAR(jacobian.value(row + 1, 2), 0.0, 1e-9);
    }
}

CC_TEST(refinement_gauss_newton_reduces_exact_reprojection_error) {
    const SyntheticProblem problem = make_problem(false);
    const CameraModel initial_camera = perturb_camera(problem.camera, false);
    const std::vector<Pose> initial_poses =
        perturb_poses(problem.poses, 0.002);
    RefinementOptions options;
    options.max_iterations = 15;
    options.cost_tolerance = 1e-14;
    Result<RefinementResult> result = refine_calibration_gauss_newton(
        problem.object_points, problem.image_points,
        initial_camera, initial_poses, options);
    CC_EXPECT_TRUE(result.ok);
    CC_EXPECT_TRUE(result.value.final_rms_reprojection_error <
                   result.value.initial_rms_reprojection_error);
    CC_EXPECT_TRUE(result.value.final_rms_reprojection_error < 1e-6);
}

CC_TEST(refinement_lm_refines_camera_distortion_and_all_poses_jointly) {
    const SyntheticProblem problem = make_problem(true);
    const CameraModel initial_camera = perturb_camera(problem.camera, true);
    const std::vector<Pose> initial_poses =
        perturb_poses(problem.poses, 0.012);
    const double initial_camera_error =
        camera_parameter_error(initial_camera, problem.camera);

    RefinementOptions options;
    options.max_iterations = 50;
    options.initial_damping = 1e-3;
    options.cost_tolerance = 1e-14;
    Result<RefinementResult> result =
        refine_calibration_levenberg_marquardt(
            problem.object_points, problem.image_points,
            initial_camera, initial_poses, options);
    CC_EXPECT_TRUE(result.ok);
    CC_EXPECT_EQ(result.value.poses.size(), problem.poses.size());
    CC_EXPECT_TRUE(result.value.final_rms_reprojection_error <
                   result.value.initial_rms_reprojection_error * 1e-3);
    CC_EXPECT_TRUE(result.value.final_rms_reprojection_error < 1e-5);
    CC_EXPECT_TRUE(camera_parameter_error(result.value.camera, problem.camera) <
                   initial_camera_error * 1e-2);
    for (std::size_t i = 0; i < problem.poses.size(); ++i) {
        CC_EXPECT_TRUE((result.value.poses[i].R - problem.poses[i].R).norm() <
                       1e-5);
        CC_EXPECT_TRUE((result.value.poses[i].t - problem.poses[i].t).norm() <
                       1e-5);
    }
}

CC_TEST(refinement_lm_reaches_noise_floor_without_overfitting_assertions) {
    SyntheticProblem problem = make_problem(true);
    const double noise[][2] = {
        {0.12, -0.08}, {-0.05, 0.09}, {0.03, -0.11},
        {-0.09, -0.02}, {0.08, 0.05}, {-0.02, 0.07},
        {0.06, -0.04}};
    for (std::size_t view = 0; view < problem.image_points.size(); ++view) {
        for (std::size_t point = 0;
             point < problem.image_points[view].size(); ++point) {
            const std::size_t index = (point + 2 * view) % 7;
            problem.image_points[view][point].x += noise[index][0];
            problem.image_points[view][point].y += noise[index][1];
        }
    }

    RefinementOptions options;
    options.max_iterations = 35;
    Result<RefinementResult> result =
        refine_calibration_levenberg_marquardt(
            problem.object_points, problem.image_points,
            perturb_camera(problem.camera, true),
            perturb_poses(problem.poses, 0.008), options);
    CC_EXPECT_TRUE(result.ok);
    CC_EXPECT_TRUE(result.value.final_rms_reprojection_error <
                   result.value.initial_rms_reprojection_error);
    CC_EXPECT_TRUE(result.value.final_rms_reprojection_error > 0.02);
    CC_EXPECT_TRUE(result.value.final_rms_reprojection_error < 0.2);
}

CC_TEST(refinement_rejects_invalid_problem_and_options) {
    const SyntheticProblem problem = make_problem(false);
    CC_EXPECT_TRUE(!calibration_residuals(
        {}, problem.image_points, problem.camera, problem.poses).ok);

    std::vector<std::vector<Point2D>> wrong_views = problem.image_points;
    wrong_views.pop_back();
    CC_EXPECT_TRUE(!calibration_residuals(
        problem.object_points, wrong_views,
        problem.camera, problem.poses).ok);

    wrong_views = problem.image_points;
    wrong_views[0].pop_back();
    CC_EXPECT_TRUE(!numeric_calibration_jacobian(
        problem.object_points, wrong_views,
        problem.camera, problem.poses).ok);

    CameraModel invalid_camera = problem.camera;
    invalid_camera.intrinsics.fx =
        std::numeric_limits<double>::quiet_NaN();
    CC_EXPECT_TRUE(!calibration_residuals(
        problem.object_points, problem.image_points,
        invalid_camera, problem.poses).ok);

    RefinementOptions invalid_options;
    invalid_options.max_iterations = 0;
    CC_EXPECT_TRUE(!refine_calibration_levenberg_marquardt(
        problem.object_points, problem.image_points,
        problem.camera, problem.poses, invalid_options).ok);
    CC_EXPECT_TRUE(!numeric_calibration_jacobian(
        problem.object_points, problem.image_points,
        problem.camera, problem.poses, 0.0).ok);
}

}  // namespace

void register_refinement_tests() {
    using clean_calib::test::registry;
    registry().push_back({"refinement.truth_has_zero_residuals",
                          refinement_truth_has_zero_residuals_and_expected_order});
    registry().push_back({"refinement.numeric_jacobian_shape_and_cx",
                          refinement_numeric_jacobian_has_expected_shape_and_cx_column});
    registry().push_back({"refinement.gauss_newton_reduces_error",
                          refinement_gauss_newton_reduces_exact_reprojection_error});
    registry().push_back({"refinement.lm_joint_camera_and_pose_recovery",
                          refinement_lm_refines_camera_distortion_and_all_poses_jointly});
    registry().push_back({"refinement.lm_noisy_data",
                          refinement_lm_reaches_noise_floor_without_overfitting_assertions});
    registry().push_back({"refinement.rejects_invalid_inputs",
                          refinement_rejects_invalid_problem_and_options});
}
