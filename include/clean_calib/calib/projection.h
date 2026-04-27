#pragma once

#include "clean_calib/core/point.h"
#include "clean_calib/core/camera.h"
#include "clean_calib/core/pose.h"
#include "clean_calib/util/result.h"

namespace clean_calib::calib {

Point3D world_to_camera(const Point3D& world_point,
                        const Pose& pose);

Result<Point2D> camera_to_normalized(const Point3D& camera_point);

Point2D normalized_to_pixel(const Point2D& normalized_point,
                            const CameraIntrinsics& intrinsics);

Point2D distort_normalized(const Point2D& normalized_point,
                           const Distortion& distortion);

Result<Point2D> project_point(const Point3D& world_point,
                              const Pose& pose,
                              const CameraModel& camera);

Result<Point2D> project_pinhole(const Point3D& world_point,
                                const Pose& pose,
                                const CameraIntrinsics& intrinsics) ;
} // namespace clean_calib::calib