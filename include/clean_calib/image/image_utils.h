#pragma once

#include "clean_calib/core/image.h"

#include <cstddef>
#include <utility>

namespace clean_calib {

bool in_bounds(const Image& image, int x, int y);

std::pair<int, int> clamp_to_image(const Image& image, int x, int y);

std::size_t pixel_offset(const Image& image, int x, int y, int channel);

Image to_grayscale(const Image& image);

} // namespace clean_calib