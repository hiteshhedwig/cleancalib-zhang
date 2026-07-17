#pragma once

#include "clean_calib/core/image.h"
#include "clean_calib/image/image_io.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace clean_calib::demo {

struct Color {
    unsigned char r;
    unsigned char g;
    unsigned char b;
};

constexpr Color white{255, 255, 255};
constexpr Color black{20, 24, 31};
constexpr Color gray{170, 176, 186};
constexpr Color light_gray{225, 228, 234};
constexpr Color red{225, 55, 65};
constexpr Color green{45, 165, 90};
constexpr Color blue{55, 105, 225};
constexpr Color cyan{35, 175, 205};
constexpr Color magenta{205, 55, 180};
constexpr Color orange{240, 145, 35};

inline Image make_rgb_image(int width, int height, Color background = white) {
    Image image;
    image.width = width;
    image.height = height;
    image.channels = 3;
    image.data.resize(static_cast<std::size_t>(width) * height * 3);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::size_t i = static_cast<std::size_t>(y * width + x) * 3;
            image.data[i] = background.r;
            image.data[i + 1] = background.g;
            image.data[i + 2] = background.b;
        }
    }
    return image;
}

inline void set_pixel(Image& image, int x, int y, Color color) {
    if (image.channels != 3 || x < 0 || y < 0 ||
        x >= image.width || y >= image.height) {
        return;
    }
    const std::size_t i = static_cast<std::size_t>(y * image.width + x) * 3;
    image.data[i] = color.r;
    image.data[i + 1] = color.g;
    image.data[i + 2] = color.b;
}

inline void fill_rect(Image& image, int x0, int y0, int x1, int y1, Color color) {
    const int left = std::max(0, std::min(x0, x1));
    const int right = std::min(image.width - 1, std::max(x0, x1));
    const int top = std::max(0, std::min(y0, y1));
    const int bottom = std::min(image.height - 1, std::max(y0, y1));
    for (int y = top; y <= bottom; ++y) {
        for (int x = left; x <= right; ++x) set_pixel(image, x, y, color);
    }
}

inline void draw_line(Image& image, int x0, int y0, int x1, int y1, Color color) {
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;

    while (true) {
        set_pixel(image, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        const int twice_error = 2 * error;
        if (twice_error >= dy) {
            error += dy;
            x0 += sx;
        }
        if (twice_error <= dx) {
            error += dx;
            y0 += sy;
        }
    }
}

inline void draw_disk(Image& image, int cx, int cy, int radius, Color color) {
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy <= radius * radius) {
                set_pixel(image, cx + dx, cy + dy, color);
            }
        }
    }
}

inline void draw_rect_outline(Image& image, int x0, int y0, int x1, int y1,
                              Color color) {
    draw_line(image, x0, y0, x1, y0, color);
    draw_line(image, x1, y0, x1, y1, color);
    draw_line(image, x1, y1, x0, y1, color);
    draw_line(image, x0, y1, x0, y0, color);
}

inline void paste_rgb(const Image& source, Image& destination, int offset_x,
                      int offset_y) {
    if (source.channels != 3 || destination.channels != 3) return;
    for (int y = 0; y < source.height; ++y) {
        for (int x = 0; x < source.width; ++x) {
            const std::size_t i = static_cast<std::size_t>(y * source.width + x) * 3;
            set_pixel(destination, offset_x + x, offset_y + y,
                      Color{source.data[i], source.data[i + 1], source.data[i + 2]});
        }
    }
}

inline Image gray_to_rgb(const Image& gray_image) {
    if (gray_image.channels != 1) return {};
    Image rgb = make_rgb_image(gray_image.width, gray_image.height);
    for (int y = 0; y < gray_image.height; ++y) {
        for (int x = 0; x < gray_image.width; ++x) {
            const unsigned char value =
                gray_image.data[static_cast<std::size_t>(y * gray_image.width + x)];
            set_pixel(rgb, x, y, Color{value, value, value});
        }
    }
    return rgb;
}

inline Image image_to_rgb(const Image& image) {
    if (image.channels == 1) return gray_to_rgb(image);
    if (image.channels != 3 && image.channels != 4) return {};
    Image rgb = make_rgb_image(image.width, image.height);
    for (int y = 0; y < image.height; ++y) {
        for (int x = 0; x < image.width; ++x) {
            const std::size_t source =
                static_cast<std::size_t>(y * image.width + x) * image.channels;
            set_pixel(rgb, x, y,
                      Color{image.data[source], image.data[source + 1],
                            image.data[source + 2]});
        }
    }
    return rgb;
}

inline Image make_color_test_image(int width = 480, int height = 300) {
    Image image = make_rgb_image(width, height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const double nx = static_cast<double>(x) / (width - 1);
            const double ny = static_cast<double>(y) / (height - 1);
            set_pixel(image, x, y,
                      Color{static_cast<unsigned char>(35 + 180 * nx),
                            static_cast<unsigned char>(35 + 180 * ny),
                            static_cast<unsigned char>(210 - 130 * nx)});
        }
    }

    const int margin = 24;
    const int top = height * 2 / 3;
    const int gap = 10;
    const int block_width = (width - 2 * margin - 4 * gap) / 5;
    const Color colors[] = {red, green, blue, orange, magenta};
    for (int i = 0; i < 5; ++i) {
        const int x0 = margin + i * (block_width + gap);
        fill_rect(image, x0, top, x0 + block_width, height - margin, colors[i]);
        draw_rect_outline(image, x0, top, x0 + block_width, height - margin, black);
    }
    draw_line(image, margin, margin, width - margin, height / 2, white);
    draw_line(image, margin, height / 2, width - margin, margin, black);
    return image;
}

inline std::string output_path(const std::string& filename) {
    const std::filesystem::path directory =
        std::filesystem::path(CLEAN_CALIB_SOURCE_DIR) / "examples" / "output";
    std::filesystem::create_directories(directory);
    return (directory / filename).string();
}

inline std::string input_path(const std::string& filename) {
    return (std::filesystem::path(CLEAN_CALIB_SOURCE_DIR) / "examples" / "images" /
            filename)
        .string();
}

inline void save_png_or_throw(const Image& image, const std::string& filename) {
    const std::string path = output_path(filename);
    const Result<bool> result = image::save(path, image);
    if (!result.ok) throw std::runtime_error(result.error);
}

inline void write_text_or_throw(const std::string& filename,
                                const std::string& contents) {
    const std::string path = output_path(filename);
    std::ofstream output(path);
    if (!output) throw std::runtime_error("failed to open " + path);
    output << contents;
    if (!output) throw std::runtime_error("failed to write " + path);
}

inline int rounded(double value) {
    return static_cast<int>(std::lround(value));
}

}  // namespace clean_calib::demo
