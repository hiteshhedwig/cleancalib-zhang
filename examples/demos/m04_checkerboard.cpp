#include "demo_draw.h"

#include "clean_calib/synthetic/checkerboard.h"

#include <iomanip>
#include <sstream>

int main() {
    using namespace clean_calib::demo;

    try {
        constexpr int rows = 6;
        constexpr int cols = 9;
        constexpr double square_size = 0.025;
        const auto points =
            clean_calib::synthetic::generate_planar_board(rows, cols, square_size);

        clean_calib::Image canvas = make_rgb_image(800, 560, Color{248, 249, 252});
        const int margin_x = 80;
        const int margin_y = 75;
        const double sx = 640.0 / ((cols - 1) * square_size);
        const double sy = 400.0 / ((rows - 1) * square_size);
        const double scale = std::min(sx, sy);
        auto screen = [&](int row, int col) {
            const auto& p = points[static_cast<std::size_t>(row * cols + col)];
            return std::pair<int, int>{rounded(margin_x + p.x * scale),
                                       rounded(margin_y + p.y * scale)};
        };

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col + 1 < cols; ++col) {
                const auto a = screen(row, col);
                const auto b = screen(row, col + 1);
                draw_line(canvas, a.first, a.second, b.first, b.second, cyan);
            }
        }
        for (int col = 0; col < cols; ++col) {
            for (int row = 0; row + 1 < rows; ++row) {
                const auto a = screen(row, col);
                const auto b = screen(row + 1, col);
                draw_line(canvas, a.first, a.second, b.first, b.second, blue);
            }
        }
        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                const auto p = screen(row, col);
                draw_disk(canvas, p.first, p.second, 5, black);
            }
        }
        const auto first = screen(0, 0);
        const auto last = screen(rows - 1, cols - 1);
        draw_disk(canvas, first.first, first.second, 10, red);
        draw_disk(canvas, last.first, last.second, 10, magenta);

        fill_rect(canvas, 80, 510, 110, 525, cyan);
        fill_rect(canvas, 140, 510, 170, 525, blue);
        fill_rect(canvas, 200, 510, 230, 525, red);
        fill_rect(canvas, 260, 510, 290, 525, magenta);
        save_png_or_throw(canvas, "m04_board_points.png");

        std::ostringstream report;
        report << std::fixed << std::setprecision(3)
               << "Milestone 4 — checkerboard object points\n"
               << "grid: " << rows << " rows x " << cols << " cols\n"
               << "square size: " << square_size << " m\n"
               << "cyan=horizontal neighbors, blue=vertical neighbors\n"
               << "red=first point, magenta=last point\n"
               << "first [0]: " << points.front().x << " " << points.front().y
               << " " << points.front().z << "\n"
               << "last [" << points.size() - 1 << "]: " << points.back().x
               << " " << points.back().y << " " << points.back().z << "\n";
        write_text_or_throw("m04_board_points.txt", report.str());
        std::cout << report.str()
                  << "image: " << output_path("m04_board_points.png") << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "demo_m04_checkerboard: " << error.what() << "\n";
        return 1;
    }
}
