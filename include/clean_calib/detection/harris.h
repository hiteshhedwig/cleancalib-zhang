#pragma once

#include <string>
#include <vector>

#include "clean_calib/core/image.h"
#include "clean_calib/util/result.h"

namespace clean_calib::detection {

struct ImageGradients {
    int width = 0;
    int height = 0;
    std::vector<double> x;
    std::vector<double> y;
};

struct HarrisOptions {
    int window_radius = 2;
    double sensitivity = 0.04;
};

struct HarrisResponse {
    int width = 0;
    int height = 0;
    std::vector<double> values;
};

// Converts supported 1/3/4-channel images to grayscale, then applies normalized
// 3x3 Sobel filters. A linear intensity ramp therefore retains its slope at
// interior pixels.
Result<ImageGradients> compute_sobel_gradients(const Image& image);

// Computes det(M) - k * trace(M)^2 using a square integration window over the
// gradient products. Pixels without a complete window receive response zero.
Result<HarrisResponse> compute_harris_response(
    const Image& image,
    const HarrisOptions& options = {});

// Maps positive responses linearly to [0,255]. Non-positive responses are
// black, making the result suitable for quick corner-response inspection.
Result<Image> harris_response_to_image(const HarrisResponse& response);

Result<bool> save_harris_response_image(
    const std::string& path,
    const HarrisResponse& response);

}  // namespace clean_calib::detection
