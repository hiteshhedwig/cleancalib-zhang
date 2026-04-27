#pragma once

#include <vector>

#include "clean_calib/core/point.h"

namespace clean_calib::synthetic {

// Generate inner-corner object points for a planar checkerboard lying
// on the Z = 0 plane in its own "board" coordinate frame.
//
// A board with `rows` x `cols` squares has (rows-1) x (cols-1) inner
// corners. We use the convention that `rows` and `cols` here refer to
// the inner-corner grid directly — i.e. you pass the number of corners
// you want along each axis, not the number of squares.
//
// Output ordering is row-major: corner (i, j) (row i, column j) is
// at index i * cols + j, with coordinates:
//
//   x = j * square_size
//   y = i * square_size
//   z = 0
//
// `square_size` is in metres (or whatever world unit you commit to).
std::vector<Point3D> generate_planar_board(int rows, int cols, double square_size);

}  // namespace clean_calib::synthetic
