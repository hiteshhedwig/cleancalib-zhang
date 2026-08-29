#pragma once

#include <vector>

#include "clean_calib/core/image.h"
#include "clean_calib/core/point.h"
#include "clean_calib/detection/harris.h"
#include "clean_calib/util/result.h"

namespace clean_calib::detection {

struct CornerCandidate {
    Point2D point;
    double response = 0.0;
};

struct CheckerboardDetectionOptions {
    int rows = 0;
    int cols = 0;
    HarrisOptions harris;
    double response_threshold_relative = 0.001;
    int nonmaximum_radius = 4;
    int candidate_pool_multiplier = 4;
    double minimum_spacing_pixels = 3.0;
    double maximum_grid_line_error = 0.12;
};

struct CheckerboardDetection {
    // Inner corners in row-major image order: top-to-bottom rows and
    // left-to-right points within each row.
    std::vector<Point2D> corners;
    std::vector<CornerCandidate> candidates;
    double grid_line_error = 0.0;
};

Result<std::vector<CornerCandidate>> suppress_harris_nonmaxima(
    const HarrisResponse& response,
    double relative_threshold,
    int radius,
    std::size_t maximum_candidates = 0);

Result<std::vector<CornerCandidate>> rank_checkerboard_candidates(
    const Image& image,
    const std::vector<CornerCandidate>& candidates,
    int sampling_radius);

// Selects the strongest rows*cols candidates, evaluates projective hull and
// PCA axis hypotheses, and returns the geometrically best row-major grid.
Result<CheckerboardDetection> fit_checkerboard_grid(
    const std::vector<CornerCandidate>& candidates,
    int rows,
    int cols,
    double minimum_spacing_pixels = 3.0,
    double maximum_grid_line_error = 0.12);

// Fits a local quadratic response surface around each input point and moves
// to its stationary maximum. If a stable local maximum cannot be fitted, the
// original point is retained.
Result<std::vector<Point2D>> refine_corners_subpixel(
    const HarrisResponse& response,
    const std::vector<Point2D>& corners);

Result<std::vector<Point2D>> refine_checkerboard_corners_subpixel(
    const Image& image,
    const std::vector<Point2D>& corners,
    int window_radius = 5,
    int maximum_iterations = 20,
    double convergence_tolerance = 1e-3);

Result<CheckerboardDetection> detect_checkerboard(
    const Image& image,
    const CheckerboardDetectionOptions& options);

}  // namespace clean_calib::detection
