#include "clean_calib/calib/homography.h"
#include "test_harness.h"

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <vector>

using clean_calib::Point2D;
using clean_calib::Result;
using clean_calib::calib::NormalizedPoints;
using clean_calib::calib::apply_homography;
using clean_calib::calib::estimate_homography;
using clean_calib::calib::homography_reprojection_errors;
using clean_calib::calib::homography_rms_reprojection_error;
using clean_calib::calib::normalize_points;

namespace {

std::vector<Point2D> make_grid_points() {
    return {{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0},
            {0.0, 1.5}, {2.0, 1.5}, {4.0, 1.5},
            {0.0, 3.0}, {2.0, 3.0}, {4.0, 3.0}};
}

std::vector<Point2D> map_points(const Eigen::Matrix3d& homography,
                                const std::vector<Point2D>& points) {
    std::vector<Point2D> mapped;
    mapped.reserve(points.size());
    for (const Point2D& point : points) {
        Result<Point2D> result = apply_homography(homography, point);
        if (!result.ok) {
            return {};
        }
        mapped.push_back(result.value);
    }
    return mapped;
}

void expect_recovery(const Eigen::Matrix3d& truth,
                     const std::vector<Point2D>& source,
                     double tolerance,
                     std::string& _err) {
    const std::vector<Point2D> destination = map_points(truth, source);
    CC_EXPECT_EQ(destination.size(), source.size());

    Result<Eigen::Matrix3d> estimated =
        estimate_homography(source, destination);
    CC_EXPECT_TRUE(estimated.ok);

    Result<double> rms = homography_rms_reprojection_error(
        estimated.value, source, destination);
    CC_EXPECT_TRUE(rms.ok);
    CC_EXPECT_NEAR(rms.value, 0.0, tolerance);
}

CC_TEST(homography_normalization_has_hartley_invariants) {
    const std::vector<Point2D> points =
        {{100.0, 50.0}, {300.0, 50.0}, {300.0, 150.0}, {100.0, 150.0}};
    Result<NormalizedPoints> result = normalize_points(points);
    CC_EXPECT_TRUE(result.ok);
    CC_EXPECT_EQ(result.value.points.size(), points.size());

    double centroid_x = 0.0;
    double centroid_y = 0.0;
    double mean_distance = 0.0;
    for (const Point2D& point : result.value.points) {
        centroid_x += point.x;
        centroid_y += point.y;
        mean_distance += std::hypot(point.x, point.y);
    }
    centroid_x /= static_cast<double>(points.size());
    centroid_y /= static_cast<double>(points.size());
    mean_distance /= static_cast<double>(points.size());

    CC_EXPECT_NEAR(centroid_x, 0.0, 1e-14);
    CC_EXPECT_NEAR(centroid_y, 0.0, 1e-14);
    CC_EXPECT_NEAR(mean_distance, std::sqrt(2.0), 1e-14);

    for (std::size_t i = 0; i < points.size(); ++i) {
        Eigen::Vector3d original(points[i].x, points[i].y, 1.0);
        Eigen::Vector3d transformed = result.value.transform * original;
        CC_EXPECT_NEAR(transformed.x(), result.value.points[i].x, 1e-14);
        CC_EXPECT_NEAR(transformed.y(), result.value.points[i].y, 1e-14);
        CC_EXPECT_NEAR(transformed.z(), 1.0, 1e-14);
    }
}

CC_TEST(homography_normalization_rejects_empty_coincident_and_nonfinite) {
    CC_EXPECT_TRUE(!normalize_points({}).ok);
    CC_EXPECT_TRUE(!normalize_points({{2.0, 3.0}, {2.0, 3.0}}).ok);
    CC_EXPECT_TRUE(!normalize_points(
        {{0.0, 0.0}, {std::numeric_limits<double>::infinity(), 1.0}}).ok);
}

CC_TEST(homography_apply_known_matrix_and_scale_invariance) {
    Eigen::Matrix3d homography;
    homography << 2.0, 0.5, 10.0,
                  0.0, 3.0, -4.0,
                  0.01, -0.02, 1.0;
    const Point2D point{4.0, 2.0};
    Result<Point2D> mapped = apply_homography(homography, point);
    Result<Point2D> scaled = apply_homography(-7.0 * homography, point);
    CC_EXPECT_TRUE(mapped.ok);
    CC_EXPECT_TRUE(scaled.ok);
    CC_EXPECT_NEAR(mapped.value.x, 19.0, 1e-12);
    CC_EXPECT_NEAR(mapped.value.y, 2.0, 1e-12);
    CC_EXPECT_NEAR(scaled.value.x, mapped.value.x, 1e-12);
    CC_EXPECT_NEAR(scaled.value.y, mapped.value.y, 1e-12);
}

CC_TEST(homography_apply_rejects_zero_denominator_and_nonfinite_input) {
    Eigen::Matrix3d zero_denominator = Eigen::Matrix3d::Identity();
    zero_denominator.row(2) << 1.0, 0.0, -2.0;
    CC_EXPECT_TRUE(!apply_homography(zero_denominator, {2.0, 4.0}).ok);

    Eigen::Matrix3d invalid = Eigen::Matrix3d::Identity();
    invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
    CC_EXPECT_TRUE(!apply_homography(invalid, {1.0, 1.0}).ok);
}

CC_TEST(homography_recovers_identity_from_four_points) {
    const std::vector<Point2D> source =
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {0.0, 1.0}};
    expect_recovery(Eigen::Matrix3d::Identity(), source, 1e-11, _err);
}

CC_TEST(homography_recovers_translation_scaling_and_shear) {
    Eigen::Matrix3d homography;
    homography << 2.5, 0.3, 120.0,
                  -0.2, 1.7, -35.0,
                  0.0, 0.0, 1.0;
    expect_recovery(homography, make_grid_points(), 1e-10, _err);
}

CC_TEST(homography_recovers_genuine_projective_mapping) {
    Eigen::Matrix3d homography;
    homography << 1.2, 0.15, 320.0,
                  -0.08, 0.95, 240.0,
                  0.0015, -0.0008, 1.0;
    expect_recovery(homography, make_grid_points(), 1e-10, _err);
}

CC_TEST(homography_normalized_dlt_handles_large_offsets_and_scale_difference) {
    const std::vector<Point2D> source = {
        {1.0e9, -2.0e9}, {1.0e9 + 1000.0, -2.0e9},
        {1.0e9 + 2000.0, -2.0e9 + 500.0},
        {1.0e9, -2.0e9 + 1000.0},
        {1.0e9 + 1000.0, -2.0e9 + 1500.0},
        {1.0e9 + 2500.0, -2.0e9 + 1800.0}};
    Eigen::Matrix3d homography;
    homography << 0.002, 0.0003, -1500.0,
                  -0.0004, 0.0015, 2750.0,
                  2.0e-10, -1.0e-10, 1.0;
    expect_recovery(homography, source, 1e-5, _err);
}

CC_TEST(homography_noisy_overdetermined_fit_has_small_finite_error) {
    const std::vector<Point2D> source = make_grid_points();
    Eigen::Matrix3d truth;
    truth << 1.2, 0.15, 320.0,
             -0.08, 0.95, 240.0,
             0.0015, -0.0008, 1.0;
    std::vector<Point2D> destination = map_points(truth, source);
    const double noise[][2] = {
        {0.10, -0.08}, {-0.06, 0.04}, {0.03, 0.07},
        {-0.09, -0.02}, {0.05, 0.01}, {-0.02, -0.06},
        {0.07, 0.03}, {-0.04, 0.08}, {0.01, -0.05}};
    for (std::size_t i = 0; i < destination.size(); ++i) {
        destination[i].x += noise[i][0];
        destination[i].y += noise[i][1];
    }

    Result<Eigen::Matrix3d> estimated =
        estimate_homography(source, destination);
    CC_EXPECT_TRUE(estimated.ok);
    Result<double> rms = homography_rms_reprojection_error(
        estimated.value, source, destination);
    CC_EXPECT_TRUE(rms.ok);
    CC_EXPECT_TRUE(rms.value > 0.0);
    CC_EXPECT_TRUE(rms.value < 0.15);
}

CC_TEST(homography_estimation_rejects_bad_correspondence_inputs) {
    const std::vector<Point2D> four =
        {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    const std::vector<Point2D> three =
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}};
    CC_EXPECT_TRUE(!estimate_homography(four, three).ok);
    CC_EXPECT_TRUE(!estimate_homography(three, three).ok);

    std::vector<Point2D> nonfinite = four;
    nonfinite[2].x = std::numeric_limits<double>::quiet_NaN();
    CC_EXPECT_TRUE(!estimate_homography(nonfinite, four).ok);
}

CC_TEST(homography_estimation_rejects_duplicate_and_collinear_points) {
    const std::vector<Point2D> valid =
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
    const std::vector<Point2D> duplicate =
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 0.0}, {0.0, 2.0}};
    const std::vector<Point2D> collinear =
        {{0.0, 0.0}, {1.0, 2.0}, {2.0, 4.0}, {3.0, 6.0}};
    const std::vector<Point2D> nearly_collinear =
        {{0.0, 0.0}, {1.0, 1.0e-8},
         {2.0, -1.0e-8}, {3.0, 2.0e-8}};

    CC_EXPECT_TRUE(!estimate_homography(duplicate, valid).ok);
    CC_EXPECT_TRUE(!estimate_homography(valid, duplicate).ok);
    CC_EXPECT_TRUE(!estimate_homography(collinear, collinear).ok);
    CC_EXPECT_TRUE(!estimate_homography(nearly_collinear,
                                        nearly_collinear).ok);
}

CC_TEST(homography_reprojection_metrics_report_expected_values) {
    const std::vector<Point2D> source =
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}};
    const std::vector<Point2D> destination =
        {{3.0, 4.0}, {1.0, 0.0}, {0.0, 1.0}};
    Result<std::vector<double>> errors = homography_reprojection_errors(
        Eigen::Matrix3d::Identity(), source, destination);
    CC_EXPECT_TRUE(errors.ok);
    CC_EXPECT_EQ(errors.value.size(), static_cast<std::size_t>(3));
    CC_EXPECT_NEAR(errors.value[0], 5.0, 1e-12);
    CC_EXPECT_NEAR(errors.value[1], 0.0, 1e-12);
    CC_EXPECT_NEAR(errors.value[2], 0.0, 1e-12);

    Result<double> rms = homography_rms_reprojection_error(
        Eigen::Matrix3d::Identity(), source, destination);
    CC_EXPECT_TRUE(rms.ok);
    CC_EXPECT_NEAR(rms.value, 5.0 / std::sqrt(3.0), 1e-12);
}

CC_TEST(homography_reprojection_metrics_reject_invalid_inputs) {
    const std::vector<Point2D> one = {{0.0, 0.0}};
    CC_EXPECT_TRUE(!homography_reprojection_errors(
        Eigen::Matrix3d::Identity(), one, {}).ok);
    CC_EXPECT_TRUE(!homography_reprojection_errors(
        Eigen::Matrix3d::Identity(), {}, {}).ok);

    Eigen::Matrix3d zero_denominator = Eigen::Matrix3d::Identity();
    zero_denominator.row(2).setZero();
    CC_EXPECT_TRUE(!homography_rms_reprojection_error(
        zero_denominator, one, one).ok);
}

}  // namespace

void register_homography_tests() {
    using clean_calib::test::registry;
    registry().push_back({"homography.normalization_hartley_invariants",
                          homography_normalization_has_hartley_invariants});
    registry().push_back({"homography.normalization_rejects_invalid_inputs",
                          homography_normalization_rejects_empty_coincident_and_nonfinite});
    registry().push_back({"homography.apply_and_scale_invariance",
                          homography_apply_known_matrix_and_scale_invariance});
    registry().push_back({"homography.apply_rejects_invalid_mapping",
                          homography_apply_rejects_zero_denominator_and_nonfinite_input});
    registry().push_back({"homography.recovers_identity_four_points",
                          homography_recovers_identity_from_four_points});
    registry().push_back({"homography.recovers_affine_transform",
                          homography_recovers_translation_scaling_and_shear});
    registry().push_back({"homography.recovers_projective_transform",
                          homography_recovers_genuine_projective_mapping});
    registry().push_back({"homography.handles_large_coordinate_offsets",
                          homography_normalized_dlt_handles_large_offsets_and_scale_difference});
    registry().push_back({"homography.noisy_overdetermined_fit",
                          homography_noisy_overdetermined_fit_has_small_finite_error});
    registry().push_back({"homography.rejects_bad_correspondence_inputs",
                          homography_estimation_rejects_bad_correspondence_inputs});
    registry().push_back({"homography.rejects_degenerate_points",
                          homography_estimation_rejects_duplicate_and_collinear_points});
    registry().push_back({"homography.reprojection_metrics",
                          homography_reprojection_metrics_report_expected_values});
    registry().push_back({"homography.reprojection_metrics_reject_invalid",
                          homography_reprojection_metrics_reject_invalid_inputs});
}
