#include "demo_draw.h"

#include "clean_calib/calib/projection.h"

#include <iomanip>
#include <sstream>

namespace {

clean_calib::Point2D map_point(const clean_calib::Point2D& point,
                               const clean_calib::CameraIntrinsics& intrinsics,
                               const clean_calib::Distortion* distortion) {
    const clean_calib::Point2D normalized =
        distortion ? clean_calib::calib::distort_normalized(point, *distortion)
                   : point;
    return clean_calib::calib::normalized_to_pixel(normalized, intrinsics);
}

void draw_grid(clean_calib::Image& image, int offset_x,
               const clean_calib::CameraIntrinsics& intrinsics,
               const clean_calib::Distortion* distortion,
               clean_calib::demo::Color color) {
    using namespace clean_calib::demo;
    constexpr int lines = 9;
    constexpr int samples = 60;
    constexpr double extent = 0.72;
    for (int line = 0; line < lines; ++line) {
        const double fixed = -extent + 2.0 * extent * line / (lines - 1);
        for (int sample = 0; sample < samples; ++sample) {
            const double a = -extent + 2.0 * extent * sample / (samples - 1);
            const double b = -extent + 2.0 * extent * (sample + 1) / (samples - 1);
            const auto horizontal_a = map_point({a, fixed}, intrinsics, distortion);
            const auto horizontal_b = map_point({b, fixed}, intrinsics, distortion);
            const auto vertical_a = map_point({fixed, a}, intrinsics, distortion);
            const auto vertical_b = map_point({fixed, b}, intrinsics, distortion);
            draw_line(image, rounded(horizontal_a.x) + offset_x,
                      rounded(horizontal_a.y), rounded(horizontal_b.x) + offset_x,
                      rounded(horizontal_b.y), color);
            draw_line(image, rounded(vertical_a.x) + offset_x,
                      rounded(vertical_a.y), rounded(vertical_b.x) + offset_x,
                      rounded(vertical_b.y), color);
        }
    }
}

}  // namespace

int main() {
    using namespace clean_calib::demo;

    try {
        constexpr int panel = 450;
        clean_calib::CameraIntrinsics intrinsics;
        intrinsics.fx = 270.0;
        intrinsics.fy = 270.0;
        intrinsics.cx = panel / 2.0;
        intrinsics.cy = panel / 2.0;

        clean_calib::Distortion distortion;
        distortion.k1 = -0.38;
        distortion.k2 = 0.12;
        distortion.p1 = 0.018;
        distortion.p2 = -0.012;

        clean_calib::Image canvas =
            make_rgb_image(panel * 2, panel, Color{248, 249, 252});
        draw_grid(canvas, 0, intrinsics, nullptr, blue);
        draw_grid(canvas, panel, intrinsics, nullptr, light_gray);
        draw_grid(canvas, panel, intrinsics, &distortion, red);
        draw_rect_outline(canvas, 2, 2, panel - 3, panel - 3, blue);
        draw_rect_outline(canvas, panel + 2, 2, 2 * panel - 3, panel - 3, red);
        draw_disk(canvas, rounded(intrinsics.cx), rounded(intrinsics.cy), 6, black);
        draw_disk(canvas, panel + rounded(intrinsics.cx), rounded(intrinsics.cy), 6,
                  black);
        save_png_or_throw(canvas, "m06_distortion_grid.png");

        std::ostringstream report;
        report << std::fixed << std::setprecision(4)
               << "Milestone 6 — Brown-Conrady distortion\n"
               << "left: ideal normalized grid (blue)\n"
               << "right: ideal grid (light gray) + distorted grid (red)\n"
               << "coefficients are exaggerated for visibility\n"
               << "k1=" << distortion.k1 << " k2=" << distortion.k2
               << " k3=" << distortion.k3 << " p1=" << distortion.p1
               << " p2=" << distortion.p2 << "\n";
        write_text_or_throw("m06_distortion_grid.txt", report.str());
        std::cout << report.str()
                  << "image: " << output_path("m06_distortion_grid.png") << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "demo_m06_distortion: " << error.what() << "\n";
        return 1;
    }
}
