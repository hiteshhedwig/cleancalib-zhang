#pragma once

#include "clean_calib/core/point.h"
#include "clean_calib/core/camera.h"
#include "clean_calib/core/pose.h"
#include "clean_calib/util/result.h"

namespace clean_calib::calib {

// Applies the world-to-camera pose convention:
//
//     P_camera = R * P_world + t
//
// Pose is assumed to map world coordinates into camera coordinates.
Point3D world_to_camera(const Point3D& world_point,
                        const Pose& pose);

// Converts camera coordinates to normalized image coordinates:
//
//     x = X / Z
//     y = Y / Z
//
// Fails if Z <= 0, because the point is on/behind the camera plane.
Result<Point2D> camera_to_normalized(const Point3D& camera_point);

// Applies camera intrinsics:
//
//     u = fx * x + skew * y + cx
//     v = fy * y + cy
//
// Input is normalized camera coordinates.
// Output is pixel coordinates, still stored as double because projected
// points are usually subpixel.
Point2D normalized_to_pixel(const Point2D& normalized_point,
                            const CameraIntrinsics& intrinsics);

// Full pinhole projection without distortion:
//
//     world -> camera -> normalized -> pixel
//
// Fails if the point cannot be projected, e.g. Z <= 0.
Result<Point2D> project_pinhole(const Point3D& world_point,
                                const Pose& pose,
                                const CameraIntrinsics& intrinsics);

} // namespace clean_calib::calib