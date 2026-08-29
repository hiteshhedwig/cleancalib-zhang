#include "clean_calib/detection/harris.h"

#include "clean_calib/image/image_io.h"
#include "clean_calib/image/image_utils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace clean_calib::detection {
namespace {

std::size_t index_of(int width, int x, int y) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
           static_cast<std::size_t>(x);
}

Result<bool> validate_image(const Image& image) {
    if (image.width < 3 || image.height < 3) {
        return Result<bool>::failure(
            "compute_sobel_gradients: image must be at least 3x3");
    }
    if (image.channels != 1 && image.channels != 3 && image.channels != 4) {
        return Result<bool>::failure(
            "compute_sobel_gradients: image must have 1, 3, or 4 channels");
    }
    const std::size_t expected_size =
        static_cast<std::size_t>(image.width) *
        static_cast<std::size_t>(image.height) *
        static_cast<std::size_t>(image.channels);
    if (image.data.size() != expected_size) {
        return Result<bool>::failure(
            "compute_sobel_gradients: image buffer size is inconsistent");
    }
    return Result<bool>::success(true);
}

double gray_at_clamped(const Image& grayscale, int x, int y) {
    const int clamped_x = std::clamp(x, 0, grayscale.width - 1);
    const int clamped_y = std::clamp(y, 0, grayscale.height - 1);
    return static_cast<double>(
        grayscale.data[index_of(grayscale.width, clamped_x, clamped_y)]);
}

Result<bool> validate_response(const HarrisResponse& response) {
    if (response.width <= 0 || response.height <= 0) {
        return Result<bool>::failure(
            "harris_response_to_image: response dimensions are invalid");
    }
    const std::size_t expected_size =
        static_cast<std::size_t>(response.width) *
        static_cast<std::size_t>(response.height);
    if (response.values.size() != expected_size) {
        return Result<bool>::failure(
            "harris_response_to_image: response buffer size is inconsistent");
    }
    for (double value : response.values) {
        if (!std::isfinite(value)) {
            return Result<bool>::failure(
                "harris_response_to_image: response contains a non-finite value");
        }
    }
    return Result<bool>::success(true);
}

}  // namespace

Result<ImageGradients> compute_sobel_gradients(const Image& image) {
    Result<bool> validation = validate_image(image);
    if (!validation.ok) {
        return Result<ImageGradients>::failure(validation.error);
    }
    const Image grayscale = to_grayscale(image);
    if (grayscale.empty()) {
        return Result<ImageGradients>::failure(
            "compute_sobel_gradients: grayscale conversion failed");
    }

    ImageGradients gradients;
    gradients.width = grayscale.width;
    gradients.height = grayscale.height;
    gradients.x.resize(grayscale.pixel_count());
    gradients.y.resize(grayscale.pixel_count());

    for (int y = 0; y < grayscale.height; ++y) {
        for (int x = 0; x < grayscale.width; ++x) {
            const double top_left = gray_at_clamped(grayscale, x - 1, y - 1);
            const double top = gray_at_clamped(grayscale, x, y - 1);
            const double top_right = gray_at_clamped(grayscale, x + 1, y - 1);
            const double left = gray_at_clamped(grayscale, x - 1, y);
            const double right = gray_at_clamped(grayscale, x + 1, y);
            const double bottom_left = gray_at_clamped(grayscale, x - 1, y + 1);
            const double bottom = gray_at_clamped(grayscale, x, y + 1);
            const double bottom_right = gray_at_clamped(grayscale, x + 1, y + 1);

            const std::size_t index = index_of(grayscale.width, x, y);
            gradients.x[index] =
                (-top_left + top_right - 2.0 * left + 2.0 * right -
                 bottom_left + bottom_right) /
                8.0;
            gradients.y[index] =
                (-top_left - 2.0 * top - top_right + bottom_left +
                 2.0 * bottom + bottom_right) /
                8.0;
        }
    }
    return Result<ImageGradients>::success(std::move(gradients));
}

Result<HarrisResponse> compute_harris_response(
    const Image& image,
    const HarrisOptions& options) {
    if (options.window_radius < 1 ||
        !std::isfinite(options.sensitivity) ||
        options.sensitivity <= 0.0 || options.sensitivity >= 0.25) {
        return Result<HarrisResponse>::failure(
            "compute_harris_response: options are invalid");
    }
    Result<ImageGradients> gradient_result = compute_sobel_gradients(image);
    if (!gradient_result.ok) {
        return Result<HarrisResponse>::failure(gradient_result.error);
    }
    const ImageGradients& gradients = gradient_result.value;
    if (gradients.width < 2 * options.window_radius + 1 ||
        gradients.height < 2 * options.window_radius + 1) {
        return Result<HarrisResponse>::failure(
            "compute_harris_response: image is smaller than the integration window");
    }

    HarrisResponse response;
    response.width = gradients.width;
    response.height = gradients.height;
    response.values.assign(
        static_cast<std::size_t>(response.width) *
            static_cast<std::size_t>(response.height),
        0.0);

    const int radius = options.window_radius;
    for (int y = radius; y < response.height - radius; ++y) {
        for (int x = radius; x < response.width - radius; ++x) {
            double sum_xx = 0.0;
            double sum_xy = 0.0;
            double sum_yy = 0.0;
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    const std::size_t index =
                        index_of(response.width, x + dx, y + dy);
                    const double gradient_x = gradients.x[index];
                    const double gradient_y = gradients.y[index];
                    sum_xx += gradient_x * gradient_x;
                    sum_xy += gradient_x * gradient_y;
                    sum_yy += gradient_y * gradient_y;
                }
            }
            const double determinant = sum_xx * sum_yy - sum_xy * sum_xy;
            const double trace = sum_xx + sum_yy;
            const double value =
                determinant - options.sensitivity * trace * trace;
            if (!std::isfinite(value)) {
                return Result<HarrisResponse>::failure(
                    "compute_harris_response: response became non-finite");
            }
            response.values[index_of(response.width, x, y)] = value;
        }
    }
    return Result<HarrisResponse>::success(std::move(response));
}

Result<Image> harris_response_to_image(const HarrisResponse& response) {
    Result<bool> validation = validate_response(response);
    if (!validation.ok) {
        return Result<Image>::failure(validation.error);
    }

    double maximum_positive = 0.0;
    for (double value : response.values) {
        maximum_positive = std::max(maximum_positive, value);
    }

    Image image;
    image.width = response.width;
    image.height = response.height;
    image.channels = 1;
    image.data.resize(response.values.size(), 0);
    if (maximum_positive <= std::numeric_limits<double>::min()) {
        return Result<Image>::success(std::move(image));
    }

    for (std::size_t i = 0; i < response.values.size(); ++i) {
        const double normalized =
            std::clamp(response.values[i] / maximum_positive, 0.0, 1.0);
        image.data[i] = static_cast<unsigned char>(
            std::lround(255.0 * normalized));
    }
    return Result<Image>::success(std::move(image));
}

Result<bool> save_harris_response_image(
    const std::string& path,
    const HarrisResponse& response) {
    Result<Image> image = harris_response_to_image(response);
    if (!image.ok) {
        return Result<bool>::failure(image.error);
    }
    Result<bool> saved = clean_calib::image::save(path, image.value);
    if (!saved.ok) {
        return Result<bool>::failure(
            "save_harris_response_image: " + saved.error);
    }
    return Result<bool>::success(true);
}

}  // namespace clean_calib::detection
