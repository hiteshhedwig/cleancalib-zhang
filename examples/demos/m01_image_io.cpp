#include "demo_draw.h"

#include "clean_calib/image/image_io.h"

#include <sstream>

int main() {
    using clean_calib::Image;
    using clean_calib::demo::input_path;
    using clean_calib::demo::output_path;
    using clean_calib::demo::save_png_or_throw;
    using clean_calib::demo::write_text_or_throw;

    try {
        const auto source = clean_calib::image::load(input_path("leena.png"));
        if (!source.ok) throw std::runtime_error(source.error);
        const Image& input = source.value;
        save_png_or_throw(input, "m01_input.png");

        const auto loaded = clean_calib::image::load(output_path("m01_input.png"));
        if (!loaded.ok) throw std::runtime_error(loaded.error);
        save_png_or_throw(loaded.value, "m01_copy.png");

        const auto copied = clean_calib::image::load(output_path("m01_copy.png"));
        if (!copied.ok) throw std::runtime_error(copied.error);

        const bool metadata_equal =
            input.width == copied.value.width &&
            input.height == copied.value.height &&
            input.channels == copied.value.channels;
        const bool pixels_equal = input.data == copied.value.data;

        std::ostringstream report;
        report << "Milestone 1 — lossless PNG round trip\n"
               << "source: examples/images/leena.png\n"
               << "dimensions: " << input.width << " x " << input.height << "\n"
               << "channels: " << input.channels << "\n"
               << "bytes: " << input.data.size() << "\n"
               << "metadata equal: " << (metadata_equal ? "yes" : "no") << "\n"
               << "pixel bytes equal: " << (pixels_equal ? "yes" : "no") << "\n";
        write_text_or_throw("m01_report.txt", report.str());

        std::cout << report.str()
                  << "input:  " << output_path("m01_input.png") << "\n"
                  << "copy:   " << output_path("m01_copy.png") << "\n";
        return metadata_equal && pixels_equal ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "demo_m01_image_io: " << error.what() << "\n";
        return 1;
    }
}
