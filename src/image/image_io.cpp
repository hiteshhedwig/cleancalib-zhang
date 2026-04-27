#include "clean_calib/image/image_io.h"

#include <algorithm>
#include <cctype>
#include <string>

// stb is a header-only library; exactly one translation unit must
// define the implementation macros. We do that here.
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image.h"
#include "stb/stb_image_write.h"

namespace clean_calib::image {

namespace {

std::string lower_ext(const std::string& path) {
    auto dot = path.find_last_of('.');
    if (dot == std::string::npos) return {};
    std::string ext = path.substr(dot + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return ext;
}

}  // namespace

Result<Image> load(const std::string& path) {
    int w = 0, h = 0, c = 0;
    unsigned char* raw = stbi_load(path.c_str(), &w, &h, &c, 0);
    if (!raw) {
        return Result<Image>::failure(
            "stbi_load failed for '" + path + "': " + stbi_failure_reason());
    }
    Image img;
    img.width = w;
    img.height = h;
    img.channels = c;
    const std::size_t n = static_cast<std::size_t>(w) * h * c;
    img.data.assign(raw, raw + n);
    stbi_image_free(raw);
    return Result<Image>::success(std::move(img));
}

Result<bool> save(const std::string& path, const Image& img) {
    if (img.empty() || img.width <= 0 || img.height <= 0 || img.channels <= 0) {
        return Result<bool>::failure("save: refusing to write an empty image");
    }
    const std::string ext = lower_ext(path);
    int rc = 0;
    const int stride = img.width * img.channels;

    if (ext == "png") {
        rc = stbi_write_png(path.c_str(), img.width, img.height,
                            img.channels, img.data.data(), stride);
    } else if (ext == "jpg" || ext == "jpeg") {
        rc = stbi_write_jpg(path.c_str(), img.width, img.height,
                            img.channels, img.data.data(), 95);
    } else if (ext == "bmp") {
        rc = stbi_write_bmp(path.c_str(), img.width, img.height,
                            img.channels, img.data.data());
    } else if (ext == "tga") {
        rc = stbi_write_tga(path.c_str(), img.width, img.height,
                            img.channels, img.data.data());
    } else {
        return Result<bool>::failure(
            "save: unsupported extension '" + ext + "' for path '" + path + "'");
    }

    if (rc == 0) {
        return Result<bool>::failure("save: stbi_write_* failed for '" + path + "'");
    }
    return Result<bool>::success(true);
}

}  // namespace clean_calib::image
