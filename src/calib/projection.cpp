#include "clean_calib/calib/projection.h"

#include <algorithm>
#include <cmath>

namespace clean_calib::calib {

Point3D world_to_camera(const Point3D& world_point,
                        const Pose& pose) {
                        
        Eigen::Vector3d p_world(
            world_point.x,
            world_point.y,
            world_point.z
        );

        Eigen::Vector3d p_camera = pose.R * p_world + pose.t;

        Point3D result;
        result.x = p_camera.x();
        result.y = p_camera.y();
        result.z = p_camera.z();

        return result;

    }


Result<Point2D> camera_to_normalized(const Point3D& camera_point) {
    if (camera_point.z <= 0.0) {
        return Result<Point2D>::failure(
            "camera_to_normalized: point is on or behind the camera"
        );
    }

    Point2D normalized;
    normalized.x = camera_point.x / camera_point.z;
    normalized.y = camera_point.y / camera_point.z;

    return Result<Point2D>::success(normalized);
}

Point2D normalized_to_pixel(const Point2D& normalized_point,
                            const CameraIntrinsics& intrinsics) {
    
    Point2D pixel ;
    pixel.x = intrinsics.fx * normalized_point.x 
              + intrinsics.skew * normalized_point.y 
              + intrinsics.cx;

    pixel.y = intrinsics.fy * normalized_point.y 
              + intrinsics.cy;

    return pixel;
}

Result<Point2D> project_pinhole(const Point3D& world_point,
                                const Pose& pose,
                                const CameraIntrinsics& intrinsics) {
    Point3D camera_point = world_to_camera(world_point, pose);

    Result<Point2D> normalized_result = camera_to_normalized(camera_point);
    if (!normalized_result.ok) {
        return Result<Point2D>::failure(normalized_result.error);
    }

    Point2D pixel = normalized_to_pixel(normalized_result.value, intrinsics);

    return Result<Point2D>::success(pixel);
}

}