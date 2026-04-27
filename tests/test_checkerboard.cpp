#include "clean_calib/synthetic/checkerboard.h"
#include "test_harness.h"

using clean_calib::Point3D;
using clean_calib::synthetic::generate_planar_board;

namespace {

CC_TEST(board_count_matches_rows_times_cols) {
    auto pts = generate_planar_board(6, 9, 0.025);
    CC_EXPECT_EQ(pts.size(), static_cast<std::size_t>(6 * 9));
}

CC_TEST(board_first_point_is_origin) {
    auto pts = generate_planar_board(6, 9, 0.025);
    CC_EXPECT_NEAR(pts.front().x, 0.0, 1e-12);
    CC_EXPECT_NEAR(pts.front().y, 0.0, 1e-12);
    CC_EXPECT_NEAR(pts.front().z, 0.0, 1e-12);
}

CC_TEST(board_is_row_major_with_correct_spacing) {
    const int rows = 4, cols = 5;
    const double s = 0.03;
    auto pts = generate_planar_board(rows, cols, s);
    // Index (i,j) -> i*cols + j
    auto at = [&](int i, int j) { return pts[i * cols + j]; };
    CC_EXPECT_NEAR(at(0, 1).x, s, 1e-12);
    CC_EXPECT_NEAR(at(0, 1).y, 0.0, 1e-12);
    CC_EXPECT_NEAR(at(1, 0).x, 0.0, 1e-12);
    CC_EXPECT_NEAR(at(1, 0).y, s, 1e-12);
    CC_EXPECT_NEAR(at(rows - 1, cols - 1).x, (cols - 1) * s, 1e-12);
    CC_EXPECT_NEAR(at(rows - 1, cols - 1).y, (rows - 1) * s, 1e-12);
}

CC_TEST(board_all_points_are_planar_z_zero) {
    auto pts = generate_planar_board(3, 7, 0.01);
    for (const Point3D& p : pts) {
        CC_EXPECT_NEAR(p.z, 0.0, 1e-12);
    }
}

CC_TEST(board_invalid_inputs_return_empty) {
    CC_EXPECT_TRUE(generate_planar_board(0, 5, 0.025).empty());
    CC_EXPECT_TRUE(generate_planar_board(5, 0, 0.025).empty());
    CC_EXPECT_TRUE(generate_planar_board(5, 5, 0.0).empty());
    CC_EXPECT_TRUE(generate_planar_board(-1, 5, 0.025).empty());
}

}  // namespace

void register_checkerboard_tests() {
    using clean_calib::test::registry;
    registry().push_back({"checkerboard.count_matches_rows_times_cols",
                          board_count_matches_rows_times_cols});
    registry().push_back({"checkerboard.first_point_is_origin",
                          board_first_point_is_origin});
    registry().push_back({"checkerboard.row_major_with_correct_spacing",
                          board_is_row_major_with_correct_spacing});
    registry().push_back({"checkerboard.all_points_z_zero",
                          board_all_points_are_planar_z_zero});
    registry().push_back({"checkerboard.invalid_inputs_return_empty",
                          board_invalid_inputs_return_empty});
}
