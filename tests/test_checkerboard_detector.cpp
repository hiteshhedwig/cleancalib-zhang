#include "clean_calib/detection/checkerboard_detector.h"
#include "test_harness.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using clean_calib::Image;
using clean_calib::Point2D;
using clean_calib::Result;
using clean_calib::detection::CheckerboardDetection;
using clean_calib::detection::CheckerboardDetectionOptions;
using clean_calib::detection::CornerCandidate;
using clean_calib::detection::HarrisResponse;
using clean_calib::detection::detect_checkerboard;
using clean_calib::detection::fit_checkerboard_grid;
using clean_calib::detection::refine_corners_subpixel;
using clean_calib::detection::suppress_harris_nonmaxima;

namespace {

std::size_t index_of(int width, int x, int y) {
    return static_cast<std::size_t>(y * width + x);
}

Point2D project_grid_point(int row, int col) {
    const double x = static_cast<double>(col);
    const double y = static_cast<double>(row);
    const double denominator = 1.0 + 0.015 * x - 0.01 * y;
    return {(24.0 + 15.0 * x + 2.5 * y) / denominator,
            (18.0 + 1.5 * x + 13.0 * y) / denominator};
}

std::vector<CornerCandidate> make_projective_candidates(int rows, int cols) {
    std::vector<CornerCandidate> candidates;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            candidates.push_back(
                {project_grid_point(row, col),
                 100.0 + static_cast<double>((row * cols + col) % 5)});
        }
    }
    std::reverse(candidates.begin(), candidates.end());
    return candidates;
}

Image render_checkerboard(int inner_rows,
                          int inner_cols,
                          int square_size,
                          int margin) {
    Image image;
    image.width = 2 * margin + (inner_cols + 1) * square_size;
    image.height = 2 * margin + (inner_rows + 1) * square_size;
    image.channels = 1;
    image.data.assign(
        static_cast<std::size_t>(image.width * image.height), 127);
    for (int square_row = 0; square_row < inner_rows + 1; ++square_row) {
        for (int square_col = 0; square_col < inner_cols + 1; ++square_col) {
            const unsigned char value =
                (square_row + square_col) % 2 == 0 ? 0 : 255;
            for (int y = margin + square_row * square_size;
                 y < margin + (square_row + 1) * square_size; ++y) {
                for (int x = margin + square_col * square_size;
                     x < margin + (square_col + 1) * square_size; ++x) {
                    image.data[index_of(image.width, x, y)] = value;
                }
            }
        }
    }
    return image;
}

CC_TEST(checkerboard_detector_nms_keeps_local_peaks_and_sorts_by_response) {
    HarrisResponse response;
    response.width = 9;
    response.height = 7;
    response.values.assign(63, 0.0);
    response.values[index_of(response.width, 2, 2)] = 10.0;
    response.values[index_of(response.width, 3, 2)] = 7.0;
    response.values[index_of(response.width, 7, 4)] = 15.0;
    response.values[index_of(response.width, 1, 5)] = 2.0;

    Result<std::vector<CornerCandidate>> result =
        suppress_harris_nonmaxima(response, 0.1, 1);
    CC_EXPECT_TRUE(result.ok);
    CC_EXPECT_EQ(result.value.size(), static_cast<std::size_t>(3));
    CC_EXPECT_NEAR(result.value[0].point.x, 7.0, 1e-12);
    CC_EXPECT_NEAR(result.value[0].point.y, 4.0, 1e-12);
    CC_EXPECT_NEAR(result.value[1].point.x, 2.0, 1e-12);
    CC_EXPECT_NEAR(result.value[1].point.y, 2.0, 1e-12);
    CC_EXPECT_NEAR(result.value[2].point.x, 1.0, 1e-12);
    CC_EXPECT_NEAR(result.value[2].point.y, 5.0, 1e-12);
}

CC_TEST(checkerboard_detector_nms_breaks_plateau_ties_deterministically) {
    HarrisResponse response;
    response.width = 5;
    response.height = 5;
    response.values.assign(25, 0.0);
    response.values[index_of(response.width, 2, 2)] = 8.0;
    response.values[index_of(response.width, 3, 2)] = 8.0;
    Result<std::vector<CornerCandidate>> result =
        suppress_harris_nonmaxima(response, 0.5, 1);
    CC_EXPECT_TRUE(result.ok);
    CC_EXPECT_EQ(result.value.size(), static_cast<std::size_t>(1));
    CC_EXPECT_NEAR(result.value[0].point.x, 2.0, 1e-12);
    CC_EXPECT_NEAR(result.value[0].point.y, 2.0, 1e-12);
}

CC_TEST(checkerboard_detector_grid_fit_orders_projective_grid_canonically) {
    constexpr int rows = 4;
    constexpr int cols = 6;
    std::vector<CornerCandidate> candidates =
        make_projective_candidates(rows, cols);
    candidates.push_back({{5.0, 80.0}, 1.0});
    candidates.push_back({{110.0, 7.0}, 2.0});

    Result<CheckerboardDetection> result = fit_checkerboard_grid(
        candidates, rows, cols, 3.0, 0.04);
    if (!result.ok) {
        CC_FAIL(result.error);
    }
    CC_EXPECT_EQ(result.value.corners.size(),
                 static_cast<std::size_t>(rows * cols));
    CC_EXPECT_TRUE(result.value.grid_line_error < 1e-10);
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const Point2D expected = project_grid_point(row, col);
            const Point2D actual =
                result.value.corners[static_cast<std::size_t>(row * cols + col)];
            CC_EXPECT_NEAR(actual.x, expected.x, 1e-10);
            CC_EXPECT_NEAR(actual.y, expected.y, 1e-10);
        }
    }
}

CC_TEST(checkerboard_detector_grid_fit_rejects_bad_geometry) {
    std::vector<CornerCandidate> collapsed;
    for (int i = 0; i < 12; ++i) {
        collapsed.push_back(
            {{static_cast<double>(i), 2.0 * static_cast<double>(i)}, 10.0});
    }
    CC_EXPECT_TRUE(!fit_checkerboard_grid(collapsed, 3, 4).ok);

    std::vector<CornerCandidate> scrambled = {
        {{0.0, 0.0}, 10.0}, {{10.0, 1.0}, 10.0}, {{25.0, 0.0}, 10.0},
        {{1.0, 10.0}, 10.0}, {{17.0, 18.0}, 10.0}, {{24.0, 9.0}, 10.0},
        {{0.0, 24.0}, 10.0}, {{11.0, 19.0}, 10.0}, {{27.0, 30.0}, 10.0}};
    CC_EXPECT_TRUE(!fit_checkerboard_grid(
        scrambled, 3, 3, 3.0, 0.03).ok);
}

CC_TEST(checkerboard_detector_grid_fit_tolerates_noisy_hull_edges) {
    constexpr int rows = 4;
    constexpr int cols = 6;
    std::vector<CornerCandidate> candidates =
        make_projective_candidates(rows, cols);
    const double noise[][2] = {
        {0.18, -0.12}, {-0.09, 0.15}, {0.06, 0.08},
        {-0.14, -0.04}, {0.11, 0.03}};
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        candidates[i].point.x += noise[i % 5][0];
        candidates[i].point.y += noise[i % 5][1];
    }
    Result<CheckerboardDetection> result = fit_checkerboard_grid(
        candidates, rows, cols, 3.0, 0.05);
    if (!result.ok) {
        CC_FAIL(result.error);
    }
    CC_EXPECT_EQ(result.value.corners.size(),
                 static_cast<std::size_t>(rows * cols));
    CC_EXPECT_TRUE(result.value.grid_line_error < 0.03);
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const Point2D expected = project_grid_point(row, col);
            const Point2D actual =
                result.value.corners[static_cast<std::size_t>(row * cols + col)];
            CC_EXPECT_TRUE(std::hypot(actual.x - expected.x,
                                      actual.y - expected.y) < 0.3);
        }
    }
}

CC_TEST(checkerboard_detector_subpixel_quadratic_recovers_known_peak) {
    HarrisResponse response;
    response.width = 10;
    response.height = 10;
    response.values.resize(100);
    const double peak_x = 4.3;
    const double peak_y = 5.7;
    for (int y = 0; y < response.height; ++y) {
        for (int x = 0; x < response.width; ++x) {
            const double dx = static_cast<double>(x) - peak_x;
            const double dy = static_cast<double>(y) - peak_y;
            response.values[index_of(response.width, x, y)] =
                100.0 - 7.0 * dx * dx - 5.0 * dy * dy + 0.8 * dx * dy;
        }
    }
    Result<std::vector<Point2D>> refined =
        refine_corners_subpixel(response, {{4.0, 6.0}});
    CC_EXPECT_TRUE(refined.ok);
    CC_EXPECT_EQ(refined.value.size(), static_cast<std::size_t>(1));
    CC_EXPECT_NEAR(refined.value[0].x, peak_x, 1e-12);
    CC_EXPECT_NEAR(refined.value[0].y, peak_y, 1e-12);
}

CC_TEST(checkerboard_detector_subpixel_falls_back_for_border_or_saddle) {
    HarrisResponse response;
    response.width = 5;
    response.height = 5;
    response.values.assign(25, 1.0);
    Result<std::vector<Point2D>> refined =
        refine_corners_subpixel(response, {{0.0, 0.0}, {2.0, 2.0}});
    CC_EXPECT_TRUE(refined.ok);
    CC_EXPECT_NEAR(refined.value[0].x, 0.0, 1e-12);
    CC_EXPECT_NEAR(refined.value[0].y, 0.0, 1e-12);
    CC_EXPECT_NEAR(refined.value[1].x, 2.0, 1e-12);
    CC_EXPECT_NEAR(refined.value[1].y, 2.0, 1e-12);
}

CC_TEST(checkerboard_detector_end_to_end_finds_rendered_inner_corners) {
    constexpr int rows = 4;
    constexpr int cols = 5;
    constexpr int square_size = 12;
    constexpr int margin = 12;
    const Image image = render_checkerboard(rows, cols, square_size, margin);
    CheckerboardDetectionOptions options;
    options.rows = rows;
    options.cols = cols;
    options.harris.window_radius = 2;
    options.response_threshold_relative = 0.02;
    options.nonmaximum_radius = 5;
    options.candidate_pool_multiplier = 3;
    options.minimum_spacing_pixels = 7.0;
    options.maximum_grid_line_error = 0.08;
    Result<CheckerboardDetection> result =
        detect_checkerboard(image, options);
    CC_EXPECT_TRUE(result.ok);
    CC_EXPECT_EQ(result.value.corners.size(),
                 static_cast<std::size_t>(rows * cols));
    CC_EXPECT_TRUE(result.value.grid_line_error < 0.02);
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const Point2D& corner =
                result.value.corners[static_cast<std::size_t>(row * cols + col)];
            const double expected_x =
                static_cast<double>(margin + (col + 1) * square_size) - 0.5;
            const double expected_y =
                static_cast<double>(margin + (row + 1) * square_size) - 0.5;
            CC_EXPECT_NEAR(corner.x, expected_x, 1.0);
            CC_EXPECT_NEAR(corner.y, expected_y, 1.0);
        }
    }
}

CC_TEST(checkerboard_detector_rejects_missing_board_and_invalid_options) {
    Image flat;
    flat.width = 80;
    flat.height = 60;
    flat.channels = 1;
    flat.data.assign(4800, 127);
    CheckerboardDetectionOptions options;
    options.rows = 4;
    options.cols = 5;
    CC_EXPECT_TRUE(!detect_checkerboard(flat, options).ok);

    options.rows = 1;
    CC_EXPECT_TRUE(!detect_checkerboard(flat, options).ok);

    HarrisResponse invalid_response;
    invalid_response.width = 2;
    invalid_response.height = 2;
    invalid_response.values = {1.0};
    CC_EXPECT_TRUE(!suppress_harris_nonmaxima(
        invalid_response, 0.1, 1).ok);

    std::vector<CornerCandidate> nonfinite =
        make_projective_candidates(3, 3);
    nonfinite[0].point.x = std::numeric_limits<double>::infinity();
    CC_EXPECT_TRUE(!fit_checkerboard_grid(nonfinite, 3, 3).ok);
}

}  // namespace

void register_checkerboard_detector_tests() {
    using clean_calib::test::registry;
    registry().push_back({"checkerboard_detector.nms_peaks",
                          checkerboard_detector_nms_keeps_local_peaks_and_sorts_by_response});
    registry().push_back({"checkerboard_detector.nms_plateau_tie",
                          checkerboard_detector_nms_breaks_plateau_ties_deterministically});
    registry().push_back({"checkerboard_detector.grid_projective_order",
                          checkerboard_detector_grid_fit_orders_projective_grid_canonically});
    registry().push_back({"checkerboard_detector.grid_rejects_geometry",
                          checkerboard_detector_grid_fit_rejects_bad_geometry});
    registry().push_back({"checkerboard_detector.grid_noisy_hull",
                          checkerboard_detector_grid_fit_tolerates_noisy_hull_edges});
    registry().push_back({"checkerboard_detector.subpixel_quadratic",
                          checkerboard_detector_subpixel_quadratic_recovers_known_peak});
    registry().push_back({"checkerboard_detector.subpixel_fallback",
                          checkerboard_detector_subpixel_falls_back_for_border_or_saddle});
    registry().push_back({"checkerboard_detector.end_to_end_rendered_board",
                          checkerboard_detector_end_to_end_finds_rendered_inner_corners});
    registry().push_back({"checkerboard_detector.rejects_invalid_inputs",
                          checkerboard_detector_rejects_missing_board_and_invalid_options});
}
