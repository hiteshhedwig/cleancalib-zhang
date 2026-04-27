#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "clean_calib/image/image_io.h"
#include "clean_calib/synthetic/checkerboard.h"
#include "clean_calib/image/image_utils.h"

using clean_calib::Image;
using clean_calib::Point3D;

namespace {

void print_usage() {
    std::cout <<
        "clean_calib — minimal CLI\n"
        "\n"
        "Usage:\n"
        "  clean_calib image-info <path>\n"
        "  clean_calib image-copy <input> <output>\n"
        "  clean_calib generate-board --rows R --cols C --square-size S\n"
        "  clean_calib grayscale <input> <output>\n"
        "\n"
        "Notes:\n"
        "  * generate-board prints inner-corner coordinates on Z=0,\n"
        "    one 'x y z' triple per line.\n";
}

int cmd_image_info(int argc, char** argv) {
    if (argc < 1) {
        std::cerr << "image-info: missing <path>\n";
        return 2;
    }
    const std::string path = argv[0];
    auto r = clean_calib::image::load(path);
    if (!r.ok) {
        std::cout << "load: FAILED\n"
                  << "path: " << path << "\n"
                  << "error: " << r.error << "\n";
        return 1;
    }
    const Image& img = r.value;
    std::cout << "load: OK\n"
              << "path: " << path << "\n"
              << "width: " << img.width << "\n"
              << "height: " << img.height << "\n"
              << "channels: " << img.channels << "\n";
    return 0;
}

int cmd_image_copy(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "image-copy: usage: image-copy <input> <output>\n";
        return 2;
    }
    auto r = clean_calib::image::load(argv[0]);
    if (!r.ok) {
        std::cerr << "image-copy: load failed: " << r.error << "\n";
        return 1;
    }
    auto w = clean_calib::image::save(argv[1], r.value);
    if (!w.ok) {
        std::cerr << "image-copy: save failed: " << w.error << "\n";
        return 1;
    }
    std::cout << "image-copy: OK (" << r.value.width << "x"
              << r.value.height << ", " << r.value.channels << " ch)\n";
    return 0;
}

int cmd_grayscale(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "grayscale: usage: grayscale <input-image> <output-image>\n";
        return 2;
    }

    const std::string input_path = argv[0];
    const std::string output_path = argv[1];

    auto image_result = clean_calib::image::load(input_path);
    if (!image_result.ok) {
        std::cerr << "Failed to load image: " << input_path << "\n";
        std::cerr << image_result.error << "\n";
        return 1;
    }

    clean_calib::Image gray = clean_calib::to_grayscale(image_result.value);

    if (gray.data.empty()) {
        std::cerr << "Failed to convert image to grayscale.\n";
        std::cerr << "Supported input channels: 1, 3, 4\n";
        return 1;
    }

    auto save_result = clean_calib::image::save(output_path, gray);
    if (!save_result.ok) {
        std::cerr << "Failed to save grayscale image: " << output_path << "\n";
        std::cerr << save_result.error << "\n";
        return 1;
    }

    std::cout << "Loaded: " << input_path << "\n";
    std::cout << "Input: "
              << image_result.value.width << " x "
              << image_result.value.height << " x "
              << image_result.value.channels << "\n";

    std::cout << "Wrote grayscale image: " << output_path << "\n";
    std::cout << "Output: "
              << gray.width << " x "
              << gray.height << " x "
              << gray.channels << "\n";

    return 0;

}

int cmd_generate_board(int argc, char** argv) {
    int rows = 0, cols = 0;
    double square_size = 0.0;
    for (int i = 0; i < argc; ++i) {
        std::string a = argv[i];
        auto need_next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "generate-board: missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };
        if (a == "--rows") {
            const char* v = need_next("--rows");
            if (!v) return 2;
            rows = std::atoi(v);
        } else if (a == "--cols") {
            const char* v = need_next("--cols");
            if (!v) return 2;
            cols = std::atoi(v);
        } else if (a == "--square-size") {
            const char* v = need_next("--square-size");
            if (!v) return 2;
            square_size = std::atof(v);
        } else {
            std::cerr << "generate-board: unknown option '" << a << "'\n";
            return 2;
        }
    }
    if (rows <= 0 || cols <= 0 || square_size <= 0.0) {
        std::cerr << "generate-board: --rows, --cols, --square-size are required and positive\n";
        return 2;
    }
    auto pts = clean_calib::synthetic::generate_planar_board(rows, cols, square_size);
    for (const Point3D& p : pts) {
        std::cout << p.x << " " << p.y << " " << p.z << "\n";
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage();
        return 0;
    }
    const std::string cmd = argv[1];
    if (cmd == "-h" || cmd == "--help" || cmd == "help") {
        print_usage();
        return 0;
    }
    if (cmd == "image-info") return cmd_image_info(argc - 2, argv + 2);
    if (cmd == "image-copy") return cmd_image_copy(argc - 2, argv + 2);
    if (cmd == "generate-board") return cmd_generate_board(argc - 2, argv + 2);
    if (cmd == "grayscale") return cmd_grayscale(argc - 2, argv + 2);

    std::cerr << "unknown command: " << cmd << "\n\n";
    print_usage();
    return 2;
}
