#include "clean_calib/calib/projection.h"
#include "test_harness.h"

using clean_calib::CameraIntrinsics;
using clean_calib::Point2D;
using clean_calib::Point3D;
using clean_calib::Pose;
using clean_calib::Result;

using clean_calib::calib::camera_to_normalized;
using clean_calib::calib::normalized_to_pixel;
using clean_calib::calib::project_pinhole;
using clean_calib::calib::world_to_camera;

namespace {

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

CameraIntrinsics make_intrinsics(double fx,
                                 double fy,
                                 double cx,
                                 double cy,
                                 double skew) {
    CameraIntrinsics k;
    k.fx = fx;
    k.fy = fy;
    k.cx = cx;
    k.cy = cy;
    k.skew = skew;
    return k;
}

CC_TEST(projection_world_to_camera_identity_pose) {
    Pose pose;

    Point3D world = make_point3(1.0, 2.0, 3.0);
    Point3D camera = world_to_camera(world, pose);

    CC_EXPECT_NEAR(camera.x, 1.0, 1e-12);
    CC_EXPECT_NEAR(camera.y, 2.0, 1e-12);
    CC_EXPECT_NEAR(camera.z, 3.0, 1e-12);
}

CC_TEST(projection_world_to_camera_translation_only) {
    Pose pose;
    pose.t(0) = 10.0;
    pose.t(1) = 20.0;
    pose.t(2) = 30.0;

    Point3D world = make_point3(1.0, 2.0, 3.0);
    Point3D camera = world_to_camera(world, pose);

    CC_EXPECT_NEAR(camera.x, 11.0, 1e-12);
    CC_EXPECT_NEAR(camera.y, 22.0, 1e-12);
    CC_EXPECT_NEAR(camera.z, 33.0, 1e-12);
}

CC_TEST(projection_camera_to_normalized_divides_by_z) {
    Point3D camera = make_point3(2.0, 4.0, 2.0);

    Result<Point2D> normalized = camera_to_normalized(camera);

    CC_EXPECT_TRUE(normalized.ok);
    CC_EXPECT_NEAR(normalized.value.x, 1.0, 1e-12);
    CC_EXPECT_NEAR(normalized.value.y, 2.0, 1e-12);
}

CC_TEST(projection_camera_to_normalized_rejects_zero_z) {
    Point3D camera = make_point3(1.0, 2.0, 0.0);

    Result<Point2D> normalized = camera_to_normalized(camera);

    CC_EXPECT_TRUE(!normalized.ok);
}

CC_TEST(projection_camera_to_normalized_rejects_negative_z) {
    Point3D camera = make_point3(1.0, 2.0, -1.0);

    Result<Point2D> normalized = camera_to_normalized(camera);

    CC_EXPECT_TRUE(!normalized.ok);
}

CC_TEST(projection_normalized_to_pixel_without_skew) {
    CameraIntrinsics k = make_intrinsics(
        100.0,  // fx
        200.0,  // fy
        320.0,  // cx
        240.0,  // cy
        0.0     // skew
    );

    Point2D normalized = make_point2(1.0, 2.0);
    Point2D pixel = normalized_to_pixel(normalized, k);

    CC_EXPECT_NEAR(pixel.x, 420.0, 1e-12);
    CC_EXPECT_NEAR(pixel.y, 640.0, 1e-12);
}

CC_TEST(projection_normalized_to_pixel_with_skew) {
    CameraIntrinsics k = make_intrinsics(
        100.0,  // fx
        200.0,  // fy
        320.0,  // cx
        240.0,  // cy
        10.0    // skew
    );

    Point2D normalized = make_point2(1.0, 2.0);
    Point2D pixel = normalized_to_pixel(normalized, k);

    // u = fx*x + skew*y + cx = 100*1 + 10*2 + 320 = 440
    // v = fy*y + cy          = 200*2 + 240        = 640
    CC_EXPECT_NEAR(pixel.x, 440.0, 1e-12);
    CC_EXPECT_NEAR(pixel.y, 640.0, 1e-12);
}

CC_TEST(projection_project_pinhole_center_point) {
    Pose pose;

    CameraIntrinsics k = make_intrinsics(
        100.0,
        100.0,
        320.0,
        240.0,
        0.0
    );

    Point3D world = make_point3(0.0, 0.0, 1.0);

    Result<Point2D> pixel = project_pinhole(world, pose, k);

    CC_EXPECT_TRUE(pixel.ok);
    CC_EXPECT_NEAR(pixel.value.x, 320.0, 1e-12);
    CC_EXPECT_NEAR(pixel.value.y, 240.0, 1e-12);
}

CC_TEST(projection_project_pinhole_off_center_point) {
    Pose pose;

    CameraIntrinsics k = make_intrinsics(
        100.0,
        100.0,
        320.0,
        240.0,
        0.0
    );

    Point3D world = make_point3(1.0, 1.0, 1.0);

    Result<Point2D> pixel = project_pinhole(world, pose, k);

    CC_EXPECT_TRUE(pixel.ok);
    CC_EXPECT_NEAR(pixel.value.x, 420.0, 1e-12);
    CC_EXPECT_NEAR(pixel.value.y, 340.0, 1e-12);
}

CC_TEST(projection_project_pinhole_rejects_point_behind_camera) {
    Pose pose;

    CameraIntrinsics k = make_intrinsics(
        100.0,
        100.0,
        320.0,
        240.0,
        0.0
    );

    Point3D world = make_point3(0.0, 0.0, -1.0);

    Result<Point2D> pixel = project_pinhole(world, pose, k);

    CC_EXPECT_TRUE(!pixel.ok);
}

}  // namespace

void register_projection_tests() {
    using clean_calib::test::registry;

    registry().push_back({"projection.world_to_camera_identity_pose",
                          projection_world_to_camera_identity_pose});

    registry().push_back({"projection.world_to_camera_translation_only",
                          projection_world_to_camera_translation_only});

    registry().push_back({"projection.camera_to_normalized_divides_by_z",
                          projection_camera_to_normalized_divides_by_z});

    registry().push_back({"projection.camera_to_normalized_rejects_zero_z",
                          projection_camera_to_normalized_rejects_zero_z});

    registry().push_back({"projection.camera_to_normalized_rejects_negative_z",
                          projection_camera_to_normalized_rejects_negative_z});

    registry().push_back({"projection.normalized_to_pixel_without_skew",
                          projection_normalized_to_pixel_without_skew});

    registry().push_back({"projection.normalized_to_pixel_with_skew",
                          projection_normalized_to_pixel_with_skew});

    registry().push_back({"projection.project_pinhole_center_point",
                          projection_project_pinhole_center_point});

    registry().push_back({"projection.project_pinhole_off_center_point",
                          projection_project_pinhole_off_center_point});

    registry().push_back({"projection.project_pinhole_rejects_point_behind_camera",
                          projection_project_pinhole_rejects_point_behind_camera});
}