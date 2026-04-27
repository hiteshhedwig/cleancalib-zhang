#pragma once

#include <string>

#include "clean_calib/core/image.h"
#include "clean_calib/util/result.h"

namespace clean_calib::image {

// Load an image from disk. The number of channels is whatever the file
// has (1, 3, or 4); we do not force a conversion.
Result<Image> load(const std::string& path);

// Save an image to disk. The format is inferred from the extension:
//   .png  -> PNG
//   .jpg / .jpeg -> JPEG (quality 95)
//   .bmp  -> BMP
// Returns ok=true on success.
Result<bool> save(const std::string& path, const Image& img);

}  // namespace clean_calib::image
