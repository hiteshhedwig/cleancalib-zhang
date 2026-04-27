#include "clean_calib/synthetic/checkerboard.h"

namespace clean_calib::synthetic {

std::vector<Point3D> generate_planar_board(int rows, int cols, double square_size) {
    std::vector<Point3D> pts;
    if (rows <= 0 || cols <= 0 || square_size <= 0.0) {
        return pts;
    }
    pts.reserve(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            Point3D p;
            p.x = static_cast<double>(j) * square_size;
            p.y = static_cast<double>(i) * square_size;
            p.z = 0.0;
            pts.push_back(p);
        }
    }
    return pts;
}

}  // namespace clean_calib::synthetic
