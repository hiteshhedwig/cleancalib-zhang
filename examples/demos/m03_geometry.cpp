#include "demo_draw.h"

#include "clean_calib/calib/projection.h"

#include <Eigen/Geometry>

#include <iomanip>
#include <sstream>

namespace {

std::pair<int, int> view_point(const Eigen::Vector3d& point, int origin_x,
                               int origin_y, double scale) {
    return {
        clean_calib::demo::rounded(origin_x + scale * (point.x() + 0.45 * point.z())),
        clean_calib::demo::rounded(origin_y + scale * (point.y() - 0.35 * point.z()))
    };
}

void draw_axes(clean_calib::Image& image, int ox, int oy, double scale) {
    using namespace clean_calib::demo;
    const auto origin = view_point(Eigen::Vector3d::Zero(), ox, oy, scale);
    const auto x = view_point(Eigen::Vector3d::UnitX(), ox, oy, scale);
    const auto y = view_point(Eigen::Vector3d::UnitY(), ox, oy, scale);
    const auto z = view_point(Eigen::Vector3d::UnitZ(), ox, oy, scale);
    draw_line(image, origin.first, origin.second, x.first, x.second, red);
    draw_line(image, origin.first, origin.second, y.first, y.second, green);
    draw_line(image, origin.first, origin.second, z.first, z.second, blue);
    draw_disk(image, origin.first, origin.second, 5, black);
    draw_disk(image, x.first, x.second, 5, red);
    draw_disk(image, y.first, y.second, 5, green);
    draw_disk(image, z.first, z.second, 5, blue);
}

}  // namespace

int main() {
    using namespace clean_calib::demo;

    try {
        constexpr double pi = 3.14159265358979323846;
        clean_calib::Pose pose;
        pose.R = (Eigen::AngleAxisd(20.0 * pi / 180.0, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(-12.0 * pi / 180.0, Eigen::Vector3d::UnitX()))
                     .toRotationMatrix();
        pose.t = Eigen::Vector3d(0.35, 0.15, 0.45);

        const clean_calib::Point3D world{0.75, 0.45, 0.35};
        const clean_calib::Point3D camera =
            clean_calib::calib::world_to_camera(world, pose);
        const Eigen::Vector3d camera_center = -pose.R.transpose() * pose.t;

        clean_calib::Image canvas = make_rgb_image(900, 430, Color{247, 248, 251});
        draw_rect_outline(canvas, 20, 20, 435, 410, gray);
        draw_rect_outline(canvas, 465, 20, 880, 410, gray);
        draw_axes(canvas, 170, 240, 125.0);
        draw_axes(canvas, 615, 240, 125.0);

        const auto world_screen =
            view_point(Eigen::Vector3d(world.x, world.y, world.z), 170, 240, 125.0);
        const auto center_screen = view_point(camera_center, 170, 240, 125.0);
        const auto camera_screen =
            view_point(Eigen::Vector3d(camera.x, camera.y, camera.z), 615, 240, 125.0);
        draw_disk(canvas, world_screen.first, world_screen.second, 8, orange);
        draw_disk(canvas, center_screen.first, center_screen.second, 8, magenta);
        draw_line(canvas, center_screen.first, center_screen.second,
                  world_screen.first, world_screen.second, light_gray);
        draw_disk(canvas, camera_screen.first, camera_screen.second, 8, orange);

        fill_rect(canvas, 45, 45, 65, 60, red);
        fill_rect(canvas, 75, 45, 95, 60, green);
        fill_rect(canvas, 105, 45, 125, 60, blue);
        fill_rect(canvas, 135, 45, 155, 60, orange);
        fill_rect(canvas, 165, 45, 185, 60, magenta);
        save_png_or_throw(canvas, "m03_coordinate_frames.png");

        std::ostringstream report;
        report << std::fixed << std::setprecision(4)
               << "Milestone 3 — coordinate-frame visualization\n"
               << "left panel: world frame; right panel: camera frame\n"
               << "red=x axis, green=y axis, blue=z axis\n"
               << "orange=the same physical point, magenta=camera center in world\n\n"
               << "world point: " << world.x << " " << world.y << " " << world.z << "\n"
               << "camera center C: " << camera_center.transpose() << "\n"
               << "extrinsic t: " << pose.t.transpose() << "\n"
               << "camera point R*X+t: " << camera.x << " " << camera.y << " " << camera.z << "\n"
               << "R:\n" << pose.R << "\n";
        write_text_or_throw("m03_coordinate_frames.txt", report.str());
        std::cout << report.str()
                  << "image: " << output_path("m03_coordinate_frames.png") << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "demo_m03_geometry: " << error.what() << "\n";
        return 1;
    }
}
