#include "demo_draw.h"

#include "clean_calib/synthetic/checkerboard.h"
#include "clean_calib/synthetic/dataset.h"

#include <Eigen/Geometry>

#include <array>
#include <iomanip>
#include <sstream>
#include <vector>

namespace {

clean_calib::Pose make_centered_pose(const Eigen::Matrix3d& rotation,
                                     const Eigen::Vector3d& board_center,
                                     const Eigen::Vector3d& desired_center) {
    clean_calib::Pose pose;
    pose.R = rotation;
    pose.t = desired_center - rotation * board_center;
    return pose;
}

void draw_grid(clean_calib::Image& image,
               const std::vector<clean_calib::Point2D>& points,
               int rows, int cols, int offset_x,
               clean_calib::demo::Color color) {
    using namespace clean_calib::demo;
    auto point = [&](int row, int col) -> std::pair<int, int> {
        const auto& p = points[static_cast<std::size_t>(row * cols + col)];
        return {rounded(p.x) + offset_x, rounded(p.y)};
    };
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col + 1 < cols; ++col) {
            const auto a = point(row, col);
            const auto b = point(row, col + 1);
            draw_line(image, a.first, a.second, b.first, b.second, color);
        }
    }
    for (int col = 0; col < cols; ++col) {
        for (int row = 0; row + 1 < rows; ++row) {
            const auto a = point(row, col);
            const auto b = point(row + 1, col);
            draw_line(image, a.first, a.second, b.first, b.second, color);
        }
    }
    for (const auto& p : points) {
        draw_disk(image, rounded(p.x) + offset_x, rounded(p.y), 4, color);
    }
    const auto& first = points.front();
    draw_disk(image, rounded(first.x) + offset_x, rounded(first.y), 7, red);
}

}  // namespace

int main() {
    using namespace clean_calib::demo;

    try {
        constexpr int rows = 6;
        constexpr int cols = 9;
        constexpr double square_size = 0.025;
        constexpr int panel_width = 320;
        constexpr int panel_height = 260;
        constexpr double pi = 3.14159265358979323846;

        clean_calib::CameraModel camera;
        camera.intrinsics.fx = 540.0;
        camera.intrinsics.fy = 530.0;
        camera.intrinsics.cx = panel_width / 2.0;
        camera.intrinsics.cy = panel_height / 2.0;

        const Eigen::Vector3d center((cols - 1) * square_size / 2.0,
                                     (rows - 1) * square_size / 2.0, 0.0);
        const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d tilted =
            (Eigen::AngleAxisd(-22.0 * pi / 180.0, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(28.0 * pi / 180.0, Eigen::Vector3d::UnitY()))
                .toRotationMatrix();

        const std::array<clean_calib::Pose, 3> poses = {
            make_centered_pose(identity, center, Eigen::Vector3d(0.0, 0.0, 0.82)),
            make_centered_pose(identity, center, Eigen::Vector3d(0.055, -0.025, 0.98)),
            make_centered_pose(tilted, center, Eigen::Vector3d(-0.025, 0.015, 0.86))
        };
        const std::array<Color, 3> colors = {cyan, orange, magenta};
        const auto board =
            clean_calib::synthetic::generate_planar_board(rows, cols, square_size);

        clean_calib::Image canvas =
            make_rgb_image(panel_width * 3, panel_height, Color{247, 248, 251});
        std::ostringstream report;
        report << std::fixed << std::setprecision(5)
               << "Milestone 5 — pinhole projection\n"
               << "panels: fronto-parallel | translated/deeper | tilted\n"
               << "red point: board origin\n\n";

        for (std::size_t i = 0; i < poses.size(); ++i) {
            const auto projected =
                clean_calib::synthetic::project_board(board, poses[i], camera);
            if (!projected.ok) throw std::runtime_error(projected.error);
            const int offset = static_cast<int>(i) * panel_width;
            draw_rect_outline(canvas, offset + 2, 2, offset + panel_width - 3,
                              panel_height - 3, colors[i]);
            draw_grid(canvas, projected.value, rows, cols, offset, colors[i]);

            double min_x = projected.value.front().x;
            double max_x = min_x;
            double min_y = projected.value.front().y;
            double max_y = min_y;
            for (const auto& p : projected.value) {
                min_x = std::min(min_x, p.x);
                max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y);
                max_y = std::max(max_y, p.y);
            }
            report << "view " << i << "\nR:\n" << poses[i].R
                   << "\nt: " << poses[i].t.transpose()
                   << "\npixel bounds: [" << min_x << ", " << max_x << "] x ["
                   << min_y << ", " << max_y << "]\n\n";
        }

        save_png_or_throw(canvas, "m05_projected_board.png");
        write_text_or_throw("m05_projected_board.txt", report.str());
        std::cout << report.str()
                  << "image: " << output_path("m05_projected_board.png") << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "demo_m05_projection: " << error.what() << "\n";
        return 1;
    }
}
