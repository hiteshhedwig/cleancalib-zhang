#include "clean_calib/image/image_utils.h"

#include <algorithm>
#include <cmath>

namespace clean_calib {

bool in_bounds(const Image& image, int x, int y) {
    return x >= 0 &&
           y >= 0 &&
           x < image.width &&
           y < image.height;
}

std::pair<int, int> clamp_to_image(const Image& image, int x, int y) {
    if (image.width <= 0 || image.height <= 0) {
        return {0, 0};
    }

    const int clamped_x = std::clamp(x, 0, image.width - 1);
    const int clamped_y = std::clamp(y, 0, image.height - 1);

    return {clamped_x, clamped_y};
}

std::size_t pixel_offset(const Image& image, int x, int y, int channel) {
    return static_cast<std::size_t>(
        (y * image.width + x) * image.channels + channel
    );
}

Image to_grayscale(const Image& image) {
    Image gray;
    gray.width = image.width;
    gray.height = image.height;
    gray.channels = 1;

    if (image.width <= 0 || image.height <= 0 || image.data.empty()) {
        return {};
    }

    const int input_channels = image.channels;

    if (input_channels != 1 && input_channels != 3 && input_channels != 4) {
        return {};
    }

    const std::size_t pixel_count =
        static_cast<std::size_t>(image.width) *
        static_cast<std::size_t>(image.height);

    gray.data.resize(pixel_count);

    if (input_channels == 1) {
        gray.data = image.data;
        return gray;
    }

    for (int y = 0; y < image.height; ++y) {
        for (int x = 0; x < image.width; ++x) {
            const std::size_t src = pixel_offset(image, x, y, 0);
            const std::size_t dst = static_cast<std::size_t>(y * image.width + x);

            const double r = static_cast<double>(image.data[src + 0]);
            const double g = static_cast<double>(image.data[src + 1]);
            const double b = static_cast<double>(image.data[src + 2]);

            const int value = static_cast<int>(
                std::round(0.299 * r + 0.587 * g + 0.114 * b)
            );

            gray.data[dst] = static_cast<unsigned char>(
                std::clamp(value, 0, 255)
            );
        }
    }

    return gray;
}

} // namespace clean_calib