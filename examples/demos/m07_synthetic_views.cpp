#include "demo_draw.h"

#include "clean_calib/synthetic/dataset.h"

#include <Eigen/Geometry>

#include <array>
#include <iomanip>
#include <sstream>

namespace {

clean_calib::Pose make_pose(double rx_degrees, double ry_degrees,
                            const Eigen::Vector3d& board_center,
                            const Eigen::Vector3d& desired_center) {
    constexpr double pi = 3.14159265358979323846;
    const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(rx_degrees * pi / 180.0, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(ry_degrees * pi / 180.0, Eigen::Vector3d::UnitY()))
            .toRotationMatrix();
    clean_calib::Pose pose;
    pose.R = rotation;
    pose.t = desired_center - rotation * board_center;
    return pose;
}

void draw_view(clean_calib::Image& image,
               const std::vector<clean_calib::Point2D>& points,
               int rows, int cols, int offset_x, int offset_y,
               clean_calib::demo::Color color) {
    using namespace clean_calib::demo;
    auto screen = [&](int row, int col) {
        const auto& p = points[static_cast<std::size_t>(row * cols + col)];
        return std::pair<int, int>{rounded(p.x) + offset_x,
                                   rounded(p.y) + offset_y};
    };
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col + 1 < cols; ++col) {
            const auto a = screen(row, col);
            const auto b = screen(row, col + 1);
            draw_line(image, a.first, a.second, b.first, b.second, color);
        }
    }
    for (int col = 0; col < cols; ++col) {
        for (int row = 0; row + 1 < rows; ++row) {
            const auto a = screen(row, col);
            const auto b = screen(row + 1, col);
            draw_line(image, a.first, a.second, b.first, b.second, color);
        }
    }
    for (const auto& p : points) {
        draw_disk(image, rounded(p.x) + offset_x, rounded(p.y) + offset_y, 3, color);
    }
    draw_disk(image, rounded(points.front().x) + offset_x,
              rounded(points.front().y) + offset_y, 6, red);
}

}  // namespace

int main() {
    using namespace clean_calib::demo;

    try {
        constexpr int rows = 6;
        constexpr int cols = 9;
        constexpr double square_size = 0.025;
        constexpr int panel_width = 300;
        constexpr int panel_height = 230;
        constexpr int gap = 12;

        clean_calib::CameraModel camera;
        camera.intrinsics.fx = 525.0;
        camera.intrinsics.fy = 515.0;
        camera.intrinsics.cx = panel_width / 2.0;
        camera.intrinsics.cy = panel_height / 2.0;
        camera.distortion.k1 = -0.045;
        camera.distortion.k2 = 0.012;

        const Eigen::Vector3d center((cols - 1) * square_size / 2.0,
                                     (rows - 1) * square_size / 2.0, 0.0);
        const std::vector<clean_calib::Pose> poses = {
            make_pose(0.0, 0.0, center, {0.0, 0.0, 0.82}),
            make_pose(-18.0, 12.0, center, {-0.035, 0.015, 0.88}),
            make_pose(16.0, -24.0, center, {0.04, -0.015, 0.92}),
            make_pose(-28.0, -15.0, center, {-0.02, -0.02, 0.78}),
            make_pose(24.0, 25.0, center, {0.02, 0.02, 1.02}),
            make_pose(8.0, -32.0, center, {0.0, -0.025, 0.86})
        };
        const auto dataset = clean_calib::synthetic::generate_calibration_dataset(
            rows, cols, square_size, camera, poses);
        if (!dataset.ok) throw std::runtime_error(dataset.error);

        const int canvas_width = 3 * panel_width + 4 * gap;
        const int canvas_height = 2 * panel_height + 3 * gap;
        clean_calib::Image canvas =
            make_rgb_image(canvas_width, canvas_height, Color{238, 241, 246});
        const std::array<Color, 6> colors =
            {cyan, orange, magenta, green, blue, Color{120, 75, 190}};
        std::ostringstream report;
        report << std::fixed << std::setprecision(5)
               << "Milestone 7 — synthetic calibration views\n"
               << "shared intrinsics: fx=" << camera.intrinsics.fx
               << " fy=" << camera.intrinsics.fy
               << " cx=" << camera.intrinsics.cx
               << " cy=" << camera.intrinsics.cy << "\n"
               << "red point in every panel: object_points[0]\n\n";

        for (std::size_t i = 0; i < dataset.value.views.size(); ++i) {
            const int panel_col = static_cast<int>(i % 3);
            const int panel_row = static_cast<int>(i / 3);
            const int ox = gap + panel_col * (panel_width + gap);
            const int oy = gap + panel_row * (panel_height + gap);
            draw_rect_outline(canvas, ox, oy, ox + panel_width - 1,
                              oy + panel_height - 1, colors[i]);
            draw_view(canvas, dataset.value.views[i].image_points, rows, cols,
                      ox, oy, colors[i]);
            report << "view " << i << "\nR:\n" << poses[i].R
                   << "\nt: " << poses[i].t.transpose() << "\n\n";
        }

        save_png_or_throw(canvas, "m07_synthetic_views.png");
        write_text_or_throw("m07_synthetic_views.txt", report.str());
        std::cout << report.str()
                  << "image: " << output_path("m07_synthetic_views.png") << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "demo_m07_synthetic_views: " << error.what() << "\n";
        return 1;
    }
}
