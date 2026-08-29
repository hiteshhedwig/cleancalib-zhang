#include "clean_calib/calib/homography.h"
#include "clean_calib/calib/refinement.h"
#include "clean_calib/calib/zhang.h"
#include "clean_calib/detection/checkerboard_detector.h"
#include "clean_calib/image/image_io.h"
#include "test_harness.h"

#include <Eigen/Core>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

using clean_calib::CameraModel;
using clean_calib::Point2D;
using clean_calib::Point3D;
using clean_calib::Result;
using clean_calib::calib::RefinementOptions;
using clean_calib::calib::RefinementResult;
using clean_calib::calib::ZhangInitialization;
using clean_calib::calib::estimate_homography;
using clean_calib::calib::homography_rms_reprojection_error;
using clean_calib::calib::initialize_zhang;
using clean_calib::calib::refine_calibration_levenberg_marquardt;
using clean_calib::detection::CheckerboardDetection;
using clean_calib::detection::CheckerboardDetectionOptions;
using clean_calib::detection::detect_checkerboard;

namespace {

CC_TEST(real_checkerboard_opencv_sample_set_detects_and_calibrates) {
    constexpr int rows = 7;
    constexpr int cols = 6;
    const std::filesystem::path directory =
        std::filesystem::path(CLEAN_CALIB_SOURCE_DIR) /
        "data" / "opencv_left";
    std::vector<std::filesystem::path> image_paths;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.path().extension() == ".jpg") {
            image_paths.push_back(entry.path());
        }
    }
    std::sort(image_paths.begin(), image_paths.end());
    CC_EXPECT_EQ(image_paths.size(), static_cast<std::size_t>(13));

    std::vector<Point2D> planar_points;
    std::vector<Point3D> object_points;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            planar_points.push_back(
                {static_cast<double>(col), static_cast<double>(row)});
            object_points.push_back(
                {static_cast<double>(col), static_cast<double>(row), 0.0});
        }
    }

    CheckerboardDetectionOptions detection_options;
    detection_options.rows = rows;
    detection_options.cols = cols;
    std::vector<std::vector<Point2D>> image_points;
    std::vector<Eigen::Matrix3d> homographies;
    for (const std::filesystem::path& path : image_paths) {
        Result<clean_calib::Image> loaded =
            clean_calib::image::load(path.string());
        CC_EXPECT_TRUE(loaded.ok);
        Result<CheckerboardDetection> detected =
            detect_checkerboard(loaded.value, detection_options);
        if (!detected.ok) {
            CC_FAIL(path.filename().string() + ": " + detected.error);
        }
        CC_EXPECT_EQ(detected.value.corners.size(),
                     static_cast<std::size_t>(rows * cols));
        Result<Eigen::Matrix3d> homography = estimate_homography(
            planar_points, detected.value.corners);
        CC_EXPECT_TRUE(homography.ok);
        Result<double> homography_rms = homography_rms_reprojection_error(
            homography.value, planar_points, detected.value.corners);
        CC_EXPECT_TRUE(homography_rms.ok);
        CC_EXPECT_TRUE(homography_rms.value < 1.5);
        image_points.push_back(detected.value.corners);
        homographies.push_back(homography.value);
    }

    Result<ZhangInitialization> initialization = initialize_zhang(homographies);
    CC_EXPECT_TRUE(initialization.ok);
    CameraModel initial_camera;
    initial_camera.intrinsics = initialization.value.intrinsics;
    RefinementOptions refinement_options;
    refinement_options.max_iterations = 60;
    Result<RefinementResult> refined =
        refine_calibration_levenberg_marquardt(
            object_points, image_points, initial_camera,
            initialization.value.poses, refinement_options);
    CC_EXPECT_TRUE(refined.ok);
    CC_EXPECT_TRUE(refined.value.converged);
    CC_EXPECT_TRUE(refined.value.final_rms_reprojection_error < 0.25);
    CC_EXPECT_TRUE(refined.value.camera.intrinsics.fx > 450.0);
    CC_EXPECT_TRUE(refined.value.camera.intrinsics.fx < 650.0);
    CC_EXPECT_TRUE(refined.value.camera.intrinsics.fy > 450.0);
    CC_EXPECT_TRUE(refined.value.camera.intrinsics.fy < 650.0);
    CC_EXPECT_TRUE(refined.value.camera.intrinsics.cx > 250.0);
    CC_EXPECT_TRUE(refined.value.camera.intrinsics.cx < 400.0);
    CC_EXPECT_TRUE(refined.value.camera.intrinsics.cy > 180.0);
    CC_EXPECT_TRUE(refined.value.camera.intrinsics.cy < 300.0);
}

}  // namespace

void register_real_checkerboard_tests() {
    using clean_calib::test::registry;
    registry().push_back({"real_checkerboard.opencv_samples_calibrate",
                          real_checkerboard_opencv_sample_set_detects_and_calibrates});
}
