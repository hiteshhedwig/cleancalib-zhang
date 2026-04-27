#pragma once

#include <cstddef>
#include <vector>

namespace clean_calib {

// A simple, owning, interleaved 8-bit image.
//
// Pixel layout: row-major, channels interleaved.
//   data[(y * width + x) * channels + c]
//
// channels:
//   1 = grayscale
//   3 = RGB
//   4 = RGBA
//
// An "empty" image has width == 0 && height == 0 && data.empty().
struct Image {
    int width = 0;
    int height = 0;
    int channels = 0;
    std::vector<unsigned char> data;

    bool empty() const { return data.empty(); }
    std::size_t pixel_count() const {
        return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    }
};

}  // namespace clean_calib
