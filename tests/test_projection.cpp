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

using clean_calib::CameraModel;
using clean_calib::Distortion;

using clean_calib::calib::distort_normalized;
using clean_calib::calib::project_point;


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

Distortion make_distortion(double k1,
                           double k2,
                           double k3,
                           double p1,
                           double p2) {
    Distortion d;
    d.k1 = k1;
    d.k2 = k2;
    d.k3 = k3;
    d.p1 = p1;
    d.p2 = p2;
    return d;
}

CameraModel make_camera_model(const CameraIntrinsics& intrinsics,
                              const Distortion& distortion) {
    CameraModel camera;
    camera.intrinsics = intrinsics;
    camera.distortion = distortion;
    return camera;
}


CC_TEST(projection_distort_normalized_zero_distortion_changes_nothing) {
    Point2D normalized = make_point2(0.5, 0.25);
    Distortion distortion = make_distortion(0.0, 0.0, 0.0, 0.0, 0.0);

    Point2D distorted = distort_normalized(normalized, distortion);

    CC_EXPECT_NEAR(distorted.x, 0.5, 1e-12);
    CC_EXPECT_NEAR(distorted.y, 0.25, 1e-12);
}

CC_TEST(projection_distort_normalized_radial_x_axis) {
    Point2D normalized = make_point2(1.0, 0.0);
    Distortion distortion = make_distortion(
        0.1,  // k1
        0.0,  // k2
        0.0,  // k3
        0.0,  // p1
        0.0   // p2
    );

    Point2D distorted = distort_normalized(normalized, distortion);

    CC_EXPECT_NEAR(distorted.x, 1.1, 1e-12);
    CC_EXPECT_NEAR(distorted.y, 0.0, 1e-12);
}

CC_TEST(projection_distort_normalized_radial_xy) {
    Point2D normalized = make_point2(1.0, 1.0);
    Distortion distortion = make_distortion(
        0.1,  // k1
        0.0,  // k2
        0.0,  // k3
        0.0,  // p1
        0.0   // p2
    );

    Point2D distorted = distort_normalized(normalized, distortion);

    CC_EXPECT_NEAR(distorted.x, 1.2, 1e-12);
    CC_EXPECT_NEAR(distorted.y, 1.2, 1e-12);
}

CC_TEST(projection_distort_normalized_tangential_only) {
    Point2D normalized = make_point2(1.0, 1.0);
    Distortion distortion = make_distortion(
        0.0,   // k1
        0.0,   // k2
        0.0,   // k3
        0.01,  // p1
        0.02   // p2
    );

    Point2D distorted = distort_normalized(normalized, distortion);

    CC_EXPECT_NEAR(distorted.x, 1.10, 1e-12);
    CC_EXPECT_NEAR(distorted.y, 1.08, 1e-12);
}

CC_TEST(projection_project_point_zero_distortion_matches_pinhole) {
    Pose pose;

    CameraIntrinsics k = make_intrinsics(
        100.0,
        100.0,
        320.0,
        240.0,
        0.0
    );

    Distortion distortion = make_distortion(0.0, 0.0, 0.0, 0.0, 0.0);
    CameraModel camera = make_camera_model(k, distortion);

    Point3D world = make_point3(1.0, 1.0, 1.0);

    Result<Point2D> pinhole_pixel = project_pinhole(world, pose, k);
    Result<Point2D> distorted_pixel = project_point(world, pose, camera);

    CC_EXPECT_TRUE(pinhole_pixel.ok);
    CC_EXPECT_TRUE(distorted_pixel.ok);

    CC_EXPECT_NEAR(distorted_pixel.value.x, pinhole_pixel.value.x, 1e-12);
    CC_EXPECT_NEAR(distorted_pixel.value.y, pinhole_pixel.value.y, 1e-12);
}

CC_TEST(projection_project_point_with_radial_distortion) {
    Pose pose;

    CameraIntrinsics k = make_intrinsics(
        100.0,
        100.0,
        320.0,
        240.0,
        0.0
    );

    Distortion distortion = make_distortion(
        0.1,
        0.0,
        0.0,
        0.0,
        0.0
    );

    CameraModel camera = make_camera_model(k, distortion);

    Point3D world = make_point3(1.0, 0.0, 1.0);

    Result<Point2D> pixel = project_point(world, pose, camera);

    CC_EXPECT_TRUE(pixel.ok);

    // normalized = (1, 0)
    // radial = 1.1
    // distorted normalized = (1.1, 0)
    // pixel x = 100 * 1.1 + 320 = 430
    // pixel y = 240
    CC_EXPECT_NEAR(pixel.value.x, 430.0, 1e-12);
    CC_EXPECT_NEAR(pixel.value.y, 240.0, 1e-12);
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

    registry().push_back({"projection.distort_normalized_zero_distortion_changes_nothing",
                      projection_distort_normalized_zero_distortion_changes_nothing});

    registry().push_back({"projection.distort_normalized_radial_x_axis",
                        projection_distort_normalized_radial_x_axis});

    registry().push_back({"projection.distort_normalized_radial_xy",
                        projection_distort_normalized_radial_xy});

    registry().push_back({"projection.distort_normalized_tangential_only",
                        projection_distort_normalized_tangential_only});

    registry().push_back({"projection.project_point_zero_distortion_matches_pinhole",
                        projection_project_point_zero_distortion_matches_pinhole});

    registry().push_back({"projection.project_point_with_radial_distortion",
                        projection_project_point_with_radial_distortion});
                        
}