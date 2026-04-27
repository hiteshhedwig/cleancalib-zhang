#pragma once

namespace clean_calib {

// Pinhole intrinsics.
//
// Projects a camera-space point (Xc, Yc, Zc) to pixels (u, v) via:
//
//   x = Xc / Zc
//   y = Yc / Zc
//   u = fx * x + skew * y + cx
//   v = fy * y + cy
//
// `skew` is almost always 0 for modern sensors but is part of the
// general Zhang formulation, so we keep it.
struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double skew = 0.0;
};

// Brown-Conrady distortion model.
//
// Given a normalised camera-plane coordinate (x, y) with r^2 = x^2 + y^2:
//
//   radial   = 1 + k1*r^2 + k2*r^4 + k3*r^6
//   x_d = x * radial + 2*p1*x*y + p2*(r^2 + 2*x^2)
//   y_d = y * radial + p1*(r^2 + 2*y^2) + 2*p2*x*y
//
// All coefficients default to zero (i.e. no distortion).
struct Distortion {
    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
};

// A complete camera model: intrinsics + distortion.
// Extrinsics live separately in `Pose` because they are per-view.
struct CameraModel {
    CameraIntrinsics intrinsics;
    Distortion distortion;
};

}  // namespace clean_calib
