#include "demo_draw.h"

#include "clean_calib/image/image_utils.h"

#include <iostream>

int main() {
    using namespace clean_calib::demo;

    try {
        const auto loaded = clean_calib::image::load(input_path("leena.png"));
        if (!loaded.ok) throw std::runtime_error(loaded.error);
        const clean_calib::Image& original = loaded.value;
        const clean_calib::Image gray = clean_calib::to_grayscale(original);
        if (gray.empty()) throw std::runtime_error("grayscale conversion failed");

        const clean_calib::Image original_rgb = image_to_rgb(original);
        const clean_calib::Image gray_rgb = gray_to_rgb(gray);
        clean_calib::Image comparison =
            make_rgb_image(original.width * 2 + 12, original.height, light_gray);
        paste_rgb(original_rgb, comparison, 0, 0);
        paste_rgb(gray_rgb, comparison, original.width + 12, 0);
        fill_rect(comparison, original.width, 0, original.width + 11,
                  original.height - 1, black);

        save_png_or_throw(original, "m02_original.png");
        save_png_or_throw(gray, "m02_grayscale.png");
        save_png_or_throw(comparison, "m02_comparison.png");

        std::cout << "Milestone 2 — RGB to grayscale\n"
                  << "source: examples/images/leena.png\n"
                  << "left: original color image\n"
                  << "right: grayscale using 0.299R + 0.587G + 0.114B\n"
                  << "comparison: " << output_path("m02_comparison.png") << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "demo_m02_grayscale: " << error.what() << "\n";
        return 1;
    }
}
