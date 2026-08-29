#include "clean_calib/calib/homography.h"
#include "clean_calib/calib/refinement.h"
#include "clean_calib/calib/zhang.h"
#include "clean_calib/detection/checkerboard_detector.h"
#include "clean_calib/image/image_io.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr int kRows = 7;
constexpr int kCols = 6;

struct ImageMeasurement {
    std::string name;
    bool clean_found = false;
    bool opencv_found = false;
    std::vector<clean_calib::Point2D> clean_corners;
    std::vector<cv::Point2f> opencv_corners;
    double clean_detection_ms = 0.0;
    double opencv_detection_ms = 0.0;
};

struct CalibrationMetrics {
    double rms = 0.0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    std::vector<double> per_view_rms;
};

std::vector<clean_calib::Point2D> planar_points() {
    std::vector<clean_calib::Point2D> points;
    for (int row = 0; row < kRows; ++row) {
        for (int col = 0; col < kCols; ++col) {
            points.push_back({static_cast<double>(col),
                              static_cast<double>(row)});
        }
    }
    return points;
}

std::vector<clean_calib::Point3D> clean_object_points() {
    std::vector<clean_calib::Point3D> points;
    for (int row = 0; row < kRows; ++row) {
        for (int col = 0; col < kCols; ++col) {
            points.push_back({static_cast<double>(col),
                              static_cast<double>(row), 0.0});
        }
    }
    return points;
}

std::vector<cv::Point3f> opencv_object_points() {
    std::vector<cv::Point3f> points;
    for (int row = 0; row < kRows; ++row) {
        for (int col = 0; col < kCols; ++col) {
            points.emplace_back(static_cast<float>(col),
                                static_cast<float>(row), 0.0F);
        }
    }
    return points;
}

CalibrationMetrics calibrate_clean(
    const std::vector<std::vector<clean_calib::Point2D>>& image_points) {
    const std::vector<clean_calib::Point2D> planar = planar_points();
    std::vector<Eigen::Matrix3d> homographies;
    for (const auto& view : image_points) {
        const auto homography =
            clean_calib::calib::estimate_homography(planar, view);
        if (!homography.ok) {
            throw std::runtime_error(homography.error);
        }
        homographies.push_back(homography.value);
    }

    const auto initialized = clean_calib::calib::initialize_zhang(homographies);
    if (!initialized.ok) {
        throw std::runtime_error(initialized.error);
    }
    clean_calib::CameraModel initial_camera;
    initial_camera.intrinsics = initialized.value.intrinsics;
    clean_calib::calib::RefinementOptions options;
    options.max_iterations = 60;
    const auto refined =
        clean_calib::calib::refine_calibration_levenberg_marquardt(
            clean_object_points(), image_points, initial_camera,
            initialized.value.poses, options);
    if (!refined.ok || !refined.value.converged) {
        throw std::runtime_error(refined.ok ? "clean-calib did not converge"
                                            : refined.error);
    }

    CalibrationMetrics metrics;
    metrics.rms = refined.value.final_rms_reprojection_error;
    metrics.fx = refined.value.camera.intrinsics.fx;
    metrics.fy = refined.value.camera.intrinsics.fy;
    metrics.cx = refined.value.camera.intrinsics.cx;
    metrics.cy = refined.value.camera.intrinsics.cy;
    const auto residuals = clean_calib::calib::calibration_residuals(
        clean_object_points(), image_points, refined.value.camera,
        refined.value.poses);
    if (!residuals.ok) {
        throw std::runtime_error(residuals.error);
    }
    const int residuals_per_view = kRows * kCols * 2;
    for (std::size_t view = 0; view < image_points.size(); ++view) {
        const Eigen::VectorXd block = residuals.value.segment(
            static_cast<Eigen::Index>(view * residuals_per_view),
            residuals_per_view);
        metrics.per_view_rms.push_back(
            std::sqrt(block.squaredNorm() / (kRows * kCols)));
    }
    return metrics;
}

CalibrationMetrics calibrate_opencv(
    const std::vector<std::vector<cv::Point2f>>& image_points,
    const cv::Size image_size) {
    const std::vector<cv::Point3f> object = opencv_object_points();
    const std::vector<std::vector<cv::Point3f>> object_points(
        image_points.size(), object);
    cv::Mat camera_matrix;
    cv::Mat distortion;
    std::vector<cv::Mat> rotation_vectors;
    std::vector<cv::Mat> translation_vectors;
    CalibrationMetrics metrics;
    metrics.rms = cv::calibrateCamera(
        object_points, image_points, image_size, camera_matrix, distortion,
        rotation_vectors, translation_vectors);
    metrics.fx = camera_matrix.at<double>(0, 0);
    metrics.fy = camera_matrix.at<double>(1, 1);
    metrics.cx = camera_matrix.at<double>(0, 2);
    metrics.cy = camera_matrix.at<double>(1, 2);
    for (std::size_t view = 0; view < image_points.size(); ++view) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(object, rotation_vectors[view], translation_vectors[view],
                          camera_matrix, distortion, projected);
        double sum_squared = 0.0;
        for (std::size_t point = 0; point < projected.size(); ++point) {
            const cv::Point2f difference = projected[point] - image_points[view][point];
            sum_squared += difference.dot(difference);
        }
        metrics.per_view_rms.push_back(
            std::sqrt(sum_squared / static_cast<double>(projected.size())));
    }
    return metrics;
}

void write_summary_row(std::ofstream& output, const std::string& subset,
                       const std::string& implementation, int detected,
                       int total, const CalibrationMetrics& metrics) {
    output << subset << ',' << implementation << ',' << detected << ',' << total
           << ',' << 100.0 * detected / total << ',' << metrics.rms << ','
           << metrics.fx << ',' << metrics.fy << ',' << metrics.cx << ','
           << metrics.cy << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: clean_calib_opencv_benchmark IMAGE_DIR OUTPUT_DIR\n";
        return 2;
    }
    const std::filesystem::path image_directory = argv[1];
    const std::filesystem::path output_directory = argv[2];
    std::filesystem::create_directories(output_directory);

    std::vector<std::filesystem::path> paths;
    for (const auto& entry : std::filesystem::directory_iterator(image_directory)) {
        if (entry.path().extension() == ".jpg") {
            paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());
    std::vector<ImageMeasurement> measurements;
    cv::Size image_size;
    for (const auto& path : paths) {
        ImageMeasurement measurement;
        measurement.name = path.filename().string();

        const auto loaded = clean_calib::image::load(path.string());
        if (!loaded.ok) {
            throw std::runtime_error(loaded.error);
        }
        clean_calib::detection::CheckerboardDetectionOptions options;
        options.rows = kRows;
        options.cols = kCols;
        auto start = std::chrono::steady_clock::now();
        const auto detected = clean_calib::detection::detect_checkerboard(
            loaded.value, options);
        auto finish = std::chrono::steady_clock::now();
        measurement.clean_detection_ms =
            std::chrono::duration<double, std::milli>(finish - start).count();
        measurement.clean_found = detected.ok;
        if (detected.ok) {
            measurement.clean_corners = detected.value.corners;
        }

        const cv::Mat gray = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
        image_size = gray.size();
        start = std::chrono::steady_clock::now();
        measurement.opencv_found = cv::findChessboardCorners(
            gray, cv::Size(kCols, kRows), measurement.opencv_corners);
        if (measurement.opencv_found) {
            cv::cornerSubPix(
                gray, measurement.opencv_corners, cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS |
                                     cv::TermCriteria::MAX_ITER,
                                 30, 0.001));
        }
        finish = std::chrono::steady_clock::now();
        measurement.opencv_detection_ms =
            std::chrono::duration<double, std::milli>(finish - start).count();
        measurements.push_back(std::move(measurement));
    }

    std::vector<std::vector<clean_calib::Point2D>> clean_all;
    std::vector<std::vector<cv::Point2f>> opencv_all;
    std::vector<std::vector<clean_calib::Point2D>> clean_common;
    std::vector<std::vector<cv::Point2f>> opencv_common;
    std::vector<std::string> common_names;
    for (const auto& measurement : measurements) {
        if (measurement.clean_found) {
            clean_all.push_back(measurement.clean_corners);
        }
        if (measurement.opencv_found) {
            opencv_all.push_back(measurement.opencv_corners);
        }
        if (measurement.clean_found && measurement.opencv_found) {
            clean_common.push_back(measurement.clean_corners);
            opencv_common.push_back(measurement.opencv_corners);
            common_names.push_back(measurement.name);
        }
    }

    const CalibrationMetrics clean_own = calibrate_clean(clean_all);
    const CalibrationMetrics opencv_own = calibrate_opencv(opencv_all, image_size);
    const CalibrationMetrics clean_shared = calibrate_clean(clean_common);
    const CalibrationMetrics opencv_shared =
        calibrate_opencv(opencv_common, image_size);

    std::ofstream summary(output_directory / "summary.csv");
    summary << std::fixed << std::setprecision(6);
    summary << "subset,implementation,detected,total,detection_rate_percent,"
               "calibration_rms_px,fx_px,fy_px,cx_px,cy_px\n";
    write_summary_row(summary, "own_detected", "clean-calib",
                      static_cast<int>(clean_all.size()), measurements.size(),
                      clean_own);
    write_summary_row(summary, "own_detected", "OpenCV 4.8.0 classic",
                      static_cast<int>(opencv_all.size()), measurements.size(),
                      opencv_own);
    write_summary_row(summary, "common_11", "clean-calib",
                      static_cast<int>(clean_common.size()), measurements.size(),
                      clean_shared);
    write_summary_row(summary, "common_11", "OpenCV 4.8.0 classic",
                      static_cast<int>(opencv_common.size()), measurements.size(),
                      opencv_shared);

    std::ofstream detections(output_directory / "detections.csv");
    detections << std::fixed << std::setprecision(3);
    detections << "image,clean_calib_found,opencv_found,clean_calib_ms,opencv_ms\n";
    for (const auto& measurement : measurements) {
        detections << measurement.name << ',' << measurement.clean_found << ','
                   << measurement.opencv_found << ','
                   << measurement.clean_detection_ms << ','
                   << measurement.opencv_detection_ms << '\n';
    }

    std::ofstream per_view(output_directory / "common_per_view_rms.csv");
    per_view << std::fixed << std::setprecision(6);
    per_view << "image,clean_calib_rms_px,opencv_rms_px\n";
    for (std::size_t view = 0; view < common_names.size(); ++view) {
        per_view << common_names[view] << ',' << clean_shared.per_view_rms[view]
                 << ',' << opencv_shared.per_view_rms[view] << '\n';
    }

    std::cout << "Wrote benchmark results to " << output_directory << '\n';
    return 0;
}
