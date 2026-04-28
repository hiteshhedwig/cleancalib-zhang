#include "clean_calib/synthetic/dataset.h"
#include "clean_calib/synthetic/dataset_io.h"
#include "test_harness.h"

#include <cstdio>
#include <string>
#include <vector>

using clean_calib::CameraModel;
using clean_calib::Point2D;
using clean_calib::Point3D;
using clean_calib::Pose;
using clean_calib::Result;

using clean_calib::synthetic::SyntheticCalibrationDataset;
using clean_calib::synthetic::SyntheticView;
using clean_calib::synthetic::generate_calibration_dataset;
using clean_calib::synthetic::load_dataset_txt;
using clean_calib::synthetic::save_dataset_txt;

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

Point2D make_point2(double x, double y) {
    Point2D p;
    p.x = x;
    p.y = y;
    return p;
}

CC_TEST(synthetic_dataset_io_save_load_roundtrip) {
    const std::string path = "clean_calib_test_synthetic_dataset_roundtrip.txt";
    std::remove(path.c_str());

    CameraModel camera = make_test_camera();
    std::vector<Pose> poses = {
        make_pose(0.0, 0.0, 1.0),
        make_pose(0.1, 0.0, 1.0)
    };

    Result<SyntheticCalibrationDataset> generated =
        generate_calibration_dataset(
            2,
            2,
            0.025,
            camera,
            poses
        );

    CC_EXPECT_TRUE(generated.ok);

    Result<bool> saved = save_dataset_txt(generated.value, path);
    CC_EXPECT_TRUE(saved.ok);

    Result<SyntheticCalibrationDataset> loaded = load_dataset_txt(path);
    CC_EXPECT_TRUE(loaded.ok);

    CC_EXPECT_EQ(loaded.value.object_points.size(),
                 generated.value.object_points.size());

    CC_EXPECT_EQ(loaded.value.views.size(),
                 generated.value.views.size());

    CC_EXPECT_EQ(loaded.value.views[0].image_points.size(),
                 generated.value.views[0].image_points.size());

    CC_EXPECT_NEAR(loaded.value.object_points[0].x,
                   generated.value.object_points[0].x,
                   1e-12);

    CC_EXPECT_NEAR(loaded.value.object_points[0].y,
                   generated.value.object_points[0].y,
                   1e-12);

    CC_EXPECT_NEAR(loaded.value.object_points[0].z,
                   generated.value.object_points[0].z,
                   1e-12);

    CC_EXPECT_NEAR(loaded.value.views[0].image_points[0].x,
                   generated.value.views[0].image_points[0].x,
                   1e-12);

    CC_EXPECT_NEAR(loaded.value.views[0].image_points[0].y,
                   generated.value.views[0].image_points[0].y,
                   1e-12);

    CC_EXPECT_NEAR(loaded.value.views[1].pose.t(0),
                   generated.value.views[1].pose.t(0),
                   1e-12);

    CC_EXPECT_NEAR(loaded.value.views[1].pose.t(1),
                   generated.value.views[1].pose.t(1),
                   1e-12);

    CC_EXPECT_NEAR(loaded.value.views[1].pose.t(2),
                   generated.value.views[1].pose.t(2),
                   1e-12);

    std::remove(path.c_str());
}

CC_TEST(synthetic_dataset_io_load_missing_file_fails) {
    Result<SyntheticCalibrationDataset> loaded =
        load_dataset_txt("clean_calib_this_file_should_not_exist_12345.txt");

    CC_EXPECT_TRUE(!loaded.ok);
}

CC_TEST(synthetic_dataset_io_save_empty_dataset_fails) {
    SyntheticCalibrationDataset dataset;

    Result<bool> saved =
        save_dataset_txt(dataset, "clean_calib_test_empty_dataset.txt");

    CC_EXPECT_TRUE(!saved.ok);
}

CC_TEST(synthetic_dataset_io_save_mismatched_point_count_fails) {
    const std::string path = "clean_calib_test_mismatched_dataset.txt";
    std::remove(path.c_str());

    SyntheticCalibrationDataset dataset;

    dataset.object_points = {
        make_point3(0.0, 0.0, 0.0),
        make_point3(1.0, 0.0, 0.0),
        make_point3(0.0, 1.0, 0.0),
        make_point3(1.0, 1.0, 0.0)
    };

    SyntheticView view;
    view.pose = make_pose(0.0, 0.0, 1.0);

    view.image_points = {
        make_point2(320.0, 240.0),
        make_point2(330.0, 240.0),
        make_point2(320.0, 250.0)
    };

    dataset.views.push_back(view);

    Result<bool> saved = save_dataset_txt(dataset, path);

    CC_EXPECT_TRUE(!saved.ok);

    std::remove(path.c_str());
}

}  // namespace

void register_synthetic_dataset_io_tests() {
    using clean_calib::test::registry;

    registry().push_back({"synthetic_dataset_io.save_load_roundtrip",
                          synthetic_dataset_io_save_load_roundtrip});

    registry().push_back({"synthetic_dataset_io.load_missing_file_fails",
                          synthetic_dataset_io_load_missing_file_fails});

    registry().push_back({"synthetic_dataset_io.save_empty_dataset_fails",
                          synthetic_dataset_io_save_empty_dataset_fails});

    registry().push_back({"synthetic_dataset_io.save_mismatched_point_count_fails",
                          synthetic_dataset_io_save_mismatched_point_count_fails});
}