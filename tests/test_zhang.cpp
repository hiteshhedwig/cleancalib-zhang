#include "clean_calib/calib/homography.h"
#include "clean_calib/calib/zhang.h"
#include "test_harness.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <vector>

using clean_calib::CameraIntrinsics;
using clean_calib::Point2D;
using clean_calib::Pose;
using clean_calib::Result;
using clean_calib::calib::ZhangInitialization;
using clean_calib::calib::apply_homography;
using clean_calib::calib::build_zhang_constraint_matrix;
using clean_calib::calib::estimate_homography;
using clean_calib::calib::estimate_intrinsics_from_homographies;
using clean_calib::calib::estimate_pose_from_homography;
using clean_calib::calib::initialize_zhang;

namespace {

CameraIntrinsics make_intrinsics() {
    CameraIntrinsics intrinsics;
    intrinsics.fx = 800.0;
    intrinsics.fy = 830.0;
    intrinsics.cx = 318.0;
    intrinsics.cy = 242.0;
    intrinsics.skew = 1.25;
    return intrinsics;
}

Eigen::Matrix3d camera_matrix(const CameraIntrinsics& intrinsics) {
    Eigen::Matrix3d matrix;
    matrix << intrinsics.fx, intrinsics.skew, intrinsics.cx,
              0.0, intrinsics.fy, intrinsics.cy,
              0.0, 0.0, 1.0;
    return matrix;
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

std::vector<Pose> make_diverse_poses() {
    return {
        make_pose(0.12, -0.18, 0.03, -0.08, -0.04, 1.15),
        make_pose(-0.22, 0.14, -0.08, 0.06, -0.02, 1.35),
        make_pose(0.28, 0.09, 0.12, -0.03, 0.07, 1.05),
        make_pose(-0.10, -0.31, 0.17, 0.09, 0.03, 1.45),
        make_pose(0.19, 0.25, -0.15, -0.05, 0.01, 1.25),
        make_pose(-0.27, -0.08, 0.06, 0.02, -0.06, 1.55)};
}

Eigen::Matrix3d homography_from_pose(const CameraIntrinsics& intrinsics,
                                     const Pose& pose) {
    Eigen::Matrix3d plane_to_camera;
    plane_to_camera.col(0) = pose.R.col(0);
    plane_to_camera.col(1) = pose.R.col(1);
    plane_to_camera.col(2) = pose.t;
    return camera_matrix(intrinsics) * plane_to_camera;
}

std::vector<Eigen::Matrix3d> make_homographies(
    const CameraIntrinsics& intrinsics,
    const std::vector<Pose>& poses) {
    std::vector<Eigen::Matrix3d> homographies;
    homographies.reserve(poses.size());
    for (const Pose& pose : poses) {
        homographies.push_back(homography_from_pose(intrinsics, pose));
    }
    return homographies;
}

void expect_intrinsics_near(const CameraIntrinsics& actual,
                            const CameraIntrinsics& expected,
                            double tolerance,
                            std::string& _err) {
    CC_EXPECT_NEAR(actual.fx, expected.fx, tolerance);
    CC_EXPECT_NEAR(actual.fy, expected.fy, tolerance);
    CC_EXPECT_NEAR(actual.cx, expected.cx, tolerance);
    CC_EXPECT_NEAR(actual.cy, expected.cy, tolerance);
    CC_EXPECT_NEAR(actual.skew, expected.skew, tolerance);
}

void expect_pose_near(const Pose& actual,
                      const Pose& expected,
                      double tolerance,
                      std::string& _err) {
    CC_EXPECT_NEAR((actual.R - expected.R).norm(), 0.0, tolerance);
    CC_EXPECT_NEAR((actual.t - expected.t).norm(), 0.0, tolerance);
}

CC_TEST(zhang_constraint_matrix_has_expected_dimensions_and_null_vector) {
    const CameraIntrinsics intrinsics = make_intrinsics();
    const std::vector<Eigen::Matrix3d> homographies =
        make_homographies(intrinsics, make_diverse_poses());
    Result<Eigen::MatrixXd> constraints =
        build_zhang_constraint_matrix(homographies);
    CC_EXPECT_TRUE(constraints.ok);
    CC_EXPECT_EQ(constraints.value.rows(),
                 2 * static_cast<Eigen::Index>(homographies.size()));
    CC_EXPECT_EQ(constraints.value.cols(), static_cast<Eigen::Index>(6));

    const Eigen::Matrix3d inverse = camera_matrix(intrinsics).inverse();
    const Eigen::Matrix3d b_matrix = inverse.transpose() * inverse;
    Eigen::Matrix<double, 6, 1> b;
    b << b_matrix(0, 0), b_matrix(0, 1), b_matrix(1, 1),
         b_matrix(0, 2), b_matrix(1, 2), b_matrix(2, 2);
    CC_EXPECT_NEAR((constraints.value * b).norm(), 0.0, 1e-15);
}

CC_TEST(zhang_constraint_matrix_is_invariant_to_homography_scales) {
    const CameraIntrinsics intrinsics = make_intrinsics();
    std::vector<Eigen::Matrix3d> homographies =
        make_homographies(intrinsics, make_diverse_poses());
    const double scales[] = {0.5, -2.0, 7.0, -0.1, 3.5, -11.0};
    for (std::size_t i = 0; i < homographies.size(); ++i) {
        homographies[i] *= scales[i];
    }
    Result<CameraIntrinsics> estimated =
        estimate_intrinsics_from_homographies(homographies);
    CC_EXPECT_TRUE(estimated.ok);
    expect_intrinsics_near(estimated.value, intrinsics, 1e-7, _err);
}

CC_TEST(zhang_recovers_intrinsics_from_exact_homographies) {
    const CameraIntrinsics intrinsics = make_intrinsics();
    const std::vector<Eigen::Matrix3d> homographies =
        make_homographies(intrinsics, make_diverse_poses());
    Result<CameraIntrinsics> estimated =
        estimate_intrinsics_from_homographies(homographies);
    CC_EXPECT_TRUE(estimated.ok);
    expect_intrinsics_near(estimated.value, intrinsics, 1e-7, _err);
}

CC_TEST(zhang_recovers_pose_and_proper_rotation) {
    const CameraIntrinsics intrinsics = make_intrinsics();
    const Pose expected = make_diverse_poses()[2];
    const Eigen::Matrix3d homography =
        -4.2 * homography_from_pose(intrinsics, expected);
    Result<Pose> estimated =
        estimate_pose_from_homography(homography, intrinsics);
    CC_EXPECT_TRUE(estimated.ok);
    expect_pose_near(estimated.value, expected, 1e-12, _err);
    CC_EXPECT_NEAR(
        (estimated.value.R.transpose() * estimated.value.R -
         Eigen::Matrix3d::Identity()).norm(),
        0.0, 1e-12);
    CC_EXPECT_NEAR(estimated.value.R.determinant(), 1.0, 1e-12);
}

CC_TEST(zhang_projects_perturbed_rotation_to_so3) {
    const CameraIntrinsics intrinsics = make_intrinsics();
    const Pose expected = make_diverse_poses()[1];
    Eigen::Matrix3d homography = homography_from_pose(intrinsics, expected);
    homography.col(0) *= 1.0005;
    homography.col(1) += 0.0003 * homography.col(0);

    Result<Pose> estimated =
        estimate_pose_from_homography(homography, intrinsics);
    CC_EXPECT_TRUE(estimated.ok);
    CC_EXPECT_NEAR(
        (estimated.value.R.transpose() * estimated.value.R -
         Eigen::Matrix3d::Identity()).norm(),
        0.0, 1e-12);
    CC_EXPECT_NEAR(estimated.value.R.determinant(), 1.0, 1e-12);
}

CC_TEST(zhang_end_to_end_initialization_recovers_intrinsics_and_poses) {
    const CameraIntrinsics intrinsics = make_intrinsics();
    const std::vector<Pose> poses = make_diverse_poses();
    const std::vector<Eigen::Matrix3d> truth_homographies =
        make_homographies(intrinsics, poses);

    std::vector<Point2D> board_points;
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 9; ++column) {
            board_points.push_back(
                {0.03 * static_cast<double>(column),
                 0.03 * static_cast<double>(row)});
        }
    }

    std::vector<Eigen::Matrix3d> estimated_homographies;
    for (const Eigen::Matrix3d& truth : truth_homographies) {
        std::vector<Point2D> image_points;
        for (const Point2D& board_point : board_points) {
            Result<Point2D> mapped = apply_homography(truth, board_point);
            CC_EXPECT_TRUE(mapped.ok);
            image_points.push_back(mapped.value);
        }
        Result<Eigen::Matrix3d> estimated =
            estimate_homography(board_points, image_points);
        CC_EXPECT_TRUE(estimated.ok);
        estimated_homographies.push_back(estimated.value);
    }

    Result<ZhangInitialization> initialization =
        initialize_zhang(estimated_homographies);
    CC_EXPECT_TRUE(initialization.ok);
    expect_intrinsics_near(initialization.value.intrinsics,
                           intrinsics, 1e-5, _err);
    CC_EXPECT_EQ(initialization.value.poses.size(), poses.size());
    for (std::size_t i = 0; i < poses.size(); ++i) {
        expect_pose_near(initialization.value.poses[i], poses[i], 1e-8, _err);
    }
}

CC_TEST(zhang_rejects_too_few_nonfinite_and_repeated_homographies) {
    const CameraIntrinsics intrinsics = make_intrinsics();
    std::vector<Eigen::Matrix3d> homographies =
        make_homographies(intrinsics, make_diverse_poses());
    homographies.resize(2);
    CC_EXPECT_TRUE(!build_zhang_constraint_matrix(homographies).ok);
    CC_EXPECT_TRUE(!estimate_intrinsics_from_homographies(homographies).ok);

    homographies = make_homographies(intrinsics, make_diverse_poses());
    homographies[1](0, 0) = std::numeric_limits<double>::quiet_NaN();
    CC_EXPECT_TRUE(!estimate_intrinsics_from_homographies(homographies).ok);

    const Eigen::Matrix3d repeated =
        homography_from_pose(intrinsics, make_diverse_poses()[0]);
    homographies.assign(4, repeated);
    CC_EXPECT_TRUE(!estimate_intrinsics_from_homographies(homographies).ok);
}

CC_TEST(zhang_pose_recovery_rejects_invalid_inputs) {
    const CameraIntrinsics intrinsics = make_intrinsics();
    CC_EXPECT_TRUE(!estimate_pose_from_homography(
        Eigen::Matrix3d::Zero(), intrinsics).ok);

    CameraIntrinsics invalid = intrinsics;
    invalid.fx = 0.0;
    CC_EXPECT_TRUE(!estimate_pose_from_homography(
        Eigen::Matrix3d::Identity(), invalid).ok);
}

}  // namespace

void register_zhang_tests() {
    using clean_calib::test::registry;
    registry().push_back({"zhang.constraint_matrix_null_vector",
                          zhang_constraint_matrix_has_expected_dimensions_and_null_vector});
    registry().push_back({"zhang.homography_scale_invariance",
                          zhang_constraint_matrix_is_invariant_to_homography_scales});
    registry().push_back({"zhang.recovers_exact_intrinsics",
                          zhang_recovers_intrinsics_from_exact_homographies});
    registry().push_back({"zhang.recovers_pose_and_proper_rotation",
                          zhang_recovers_pose_and_proper_rotation});
    registry().push_back({"zhang.projects_rotation_to_so3",
                          zhang_projects_perturbed_rotation_to_so3});
    registry().push_back({"zhang.end_to_end_initialization",
                          zhang_end_to_end_initialization_recovers_intrinsics_and_poses});
    registry().push_back({"zhang.rejects_invalid_homography_sets",
                          zhang_rejects_too_few_nonfinite_and_repeated_homographies});
    registry().push_back({"zhang.pose_rejects_invalid_inputs",
                          zhang_pose_recovery_rejects_invalid_inputs});
}
