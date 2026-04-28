#include "clean_calib/synthetic/dataset.h"
#include "test_harness.h"

#include <vector>

using clean_calib::CameraIntrinsics;
using clean_calib::CameraModel;
using clean_calib::Distortion;
using clean_calib::Point2D;
using clean_calib::Point3D;
using clean_calib::Pose;
using clean_calib::Result;

using clean_calib::synthetic::SyntheticCalibrationDataset;
using clean_calib::synthetic::project_board;
using clean_calib::synthetic::generate_calibration_dataset;

namespace {

CameraModel make_test_camera() {
    CameraModel camera;
    camera.intrinsics.fx = 100.0;
    camera.intrinsics.fy = 100.0;
    camera.intrinsics.cx = 320.0;
    camera.intrinsics.cy = 240.0;
    camera.intrinsics.skew = 0.0;

    camera.distortion.k1 = 0.0;
    camera.distortion.k2 = 0.0;
    camera.distortion.k3 = 0.0;
    camera.distortion.p1 = 0.0;
    camera.distortion.p2 = 0.0;

    return camera;
}

Pose make_pose(double tx, double ty, double tz) {
    Pose pose;
    pose.t(0) = tx;
    pose.t(1) = ty;
    pose.t(2) = tz;
    return pose;
}

Point3D make_point3(double x, double y, double z) {
    Point3D p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

CC_TEST(synthetic_dataset_project_board_projects_all_points) {
    CameraModel camera = make_test_camera();
    Pose pose = make_pose(0.0, 0.0, 1.0);

    std::vector<Point3D> object_points = {
        make_point3(0.0, 0.0, 0.0),
        make_point3(0.1, 0.0, 0.0),
        make_point3(0.0, 0.1, 0.0)
    };

    Result<std::vector<Point2D>> projected =
        project_board(object_points, pose, camera);

    CC_EXPECT_TRUE(projected.ok);
    CC_EXPECT_EQ(projected.value.size(), object_points.size());
}

CC_TEST(synthetic_dataset_one_pose_creates_one_view) {
    CameraModel camera = make_test_camera();
    std::vector<Pose> poses = {
        make_pose(0.0, 0.0, 1.0)
    };

    Result<SyntheticCalibrationDataset> result =
        generate_calibration_dataset(
            6,
            9,
            0.025,
            camera,
            poses
        );

    CC_EXPECT_TRUE(result.ok);
    CC_EXPECT_EQ(result.value.views.size(), static_cast<std::size_t>(1));
    CC_EXPECT_EQ(result.value.views[0].image_points.size(),
                 result.value.object_points.size());
}

CC_TEST(synthetic_dataset_object_point_count_matches_board_size) {
    CameraModel camera = make_test_camera();
    std::vector<Pose> poses = {
        make_pose(0.0, 0.0, 1.0)
    };

    Result<SyntheticCalibrationDataset> result =
        generate_calibration_dataset(
            6,
            9,
            0.025,
            camera,
            poses
        );

    CC_EXPECT_TRUE(result.ok);
    CC_EXPECT_EQ(result.value.object_points.size(),
                 static_cast<std::size_t>(6 * 9));
}

CC_TEST(synthetic_dataset_centered_first_point_projects_to_principal_point) {
    CameraModel camera = make_test_camera();
    std::vector<Pose> poses = {
        make_pose(0.0, 0.0, 1.0)
    };

    Result<SyntheticCalibrationDataset> result =
        generate_calibration_dataset(
            6,
            9,
            0.025,
            camera,
            poses
        );

    CC_EXPECT_TRUE(result.ok);

    const Point2D& first = result.value.views[0].image_points[0];

    CC_EXPECT_NEAR(first.x, 320.0, 1e-12);
    CC_EXPECT_NEAR(first.y, 240.0, 1e-12);
}


CC_TEST(synthetic_dataset_translated_pose_shifts_projection) {
    CameraModel camera = make_test_camera();
    std::vector<Pose> poses = {
        make_pose(0.1, 0.0, 1.0)
    };

    Result<SyntheticCalibrationDataset> result =
        generate_calibration_dataset(
            6,
            9,
            0.025,
            camera,
            poses
        );

    CC_EXPECT_TRUE(result.ok);

    const Point2D& first = result.value.views[0].image_points[0];

    CC_EXPECT_NEAR(first.x, 330.0, 1e-12);
    CC_EXPECT_NEAR(first.y, 240.0, 1e-12);
}

CC_TEST(synthetic_dataset_invalid_pose_behind_camera_fails) {
    CameraModel camera = make_test_camera();
    std::vector<Pose> poses = {
        make_pose(0.0, 0.0, -1.0)
    };

    Result<SyntheticCalibrationDataset> result =
        generate_calibration_dataset(
            6,
            9,
            0.025,
            camera,
            poses
        );

    CC_EXPECT_TRUE(!result.ok);
}

}  // namespace

void register_synthetic_dataset_tests() {
    using clean_calib::test::registry;

    registry().push_back({"synthetic_dataset.project_board_projects_all_points",
                          synthetic_dataset_project_board_projects_all_points});

    registry().push_back({"synthetic_dataset.object_point_count_matches_board_size",
                          synthetic_dataset_object_point_count_matches_board_size});

    registry().push_back({"synthetic_dataset.one_pose_creates_one_view",
                          synthetic_dataset_one_pose_creates_one_view});

    registry().push_back({"synthetic_dataset.centered_first_point_projects_to_principal_point",
                          synthetic_dataset_centered_first_point_projects_to_principal_point});

    registry().push_back({"synthetic_dataset.translated_pose_shifts_projection",
                          synthetic_dataset_translated_pose_shifts_projection});

    registry().push_back({"synthetic_dataset.invalid_pose_behind_camera_fails",
                          synthetic_dataset_invalid_pose_behind_camera_fails});
}