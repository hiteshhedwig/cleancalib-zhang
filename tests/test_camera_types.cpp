#include "clean_calib/core/camera.h"
#include "clean_calib/core/point.h"
#include "clean_calib/core/pose.h"
#include "test_harness.h"

using clean_calib::CameraIntrinsics;
using clean_calib::CameraModel;
using clean_calib::Distortion;
using clean_calib::Point2D;
using clean_calib::Point3D;
using clean_calib::Pose;

namespace {

CC_TEST(point_default_is_zero) {
    Point2D p2;
    Point3D p3;
    CC_EXPECT_NEAR(p2.x, 0.0, 1e-12);
    CC_EXPECT_NEAR(p2.y, 0.0, 1e-12);
    CC_EXPECT_NEAR(p3.x, 0.0, 1e-12);
    CC_EXPECT_NEAR(p3.y, 0.0, 1e-12);
    CC_EXPECT_NEAR(p3.z, 0.0, 1e-12);
}

CC_TEST(intrinsics_distortion_default_zero) {
    CameraIntrinsics k;
    Distortion d;
    CC_EXPECT_NEAR(k.fx, 0.0, 1e-12);
    CC_EXPECT_NEAR(k.skew, 0.0, 1e-12);
    CC_EXPECT_NEAR(d.k1, 0.0, 1e-12);
    CC_EXPECT_NEAR(d.p2, 0.0, 1e-12);
}

CC_TEST(pose_default_is_identity) {
    Pose p;
    CC_EXPECT_NEAR(p.R(0, 0), 1.0, 1e-12);
    CC_EXPECT_NEAR(p.R(1, 1), 1.0, 1e-12);
    CC_EXPECT_NEAR(p.R(2, 2), 1.0, 1e-12);
    CC_EXPECT_NEAR(p.R(0, 1), 0.0, 1e-12);
    CC_EXPECT_NEAR(p.t.norm(), 0.0, 1e-12);
}

CC_TEST(camera_model_holds_intrinsics_and_distortion) {
    CameraModel m;
    m.intrinsics.fx = 800.0;
    m.distortion.k1 = -0.1;
    CC_EXPECT_NEAR(m.intrinsics.fx, 800.0, 1e-12);
    CC_EXPECT_NEAR(m.distortion.k1, -0.1, 1e-12);
}

}  // namespace

void register_camera_types_tests() {
    using clean_calib::test::registry;
    registry().push_back({"camera.point_default_is_zero", point_default_is_zero});
    registry().push_back({"camera.intrinsics_distortion_default_zero",
                          intrinsics_distortion_default_zero});
    registry().push_back({"camera.pose_default_is_identity", pose_default_is_identity});
    registry().push_back({"camera.model_holds_intrinsics_and_distortion",
                          camera_model_holds_intrinsics_and_distortion});
}
