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


Point2D distort_normalized(const Point2D& normalized_point,
                           const Distortion& distortion) {

        double x = normalized_point.x;
        double y = normalized_point.y;

        double r2 = x*x + y*y ;
        double r4 = r2 * r2 ;
        double r6 = r4 * r2 ;

        double radial = 1 + distortion.k1 * r2 
                         + distortion.k2 * r4  
                         + distortion.k3 * r6 ;
        
        double x_tangential =  2.0 * distortion.p1 * x * y
                            + distortion.p2 * (r2 + 2.0 * x * x);

        double y_tangential = distortion.p1 * (r2 + 2 * y * y) +
                                2 * distortion.p2 * x * y  ;

        double x_distorted = x * radial + x_tangential;
        double y_distorted = y * radial + y_tangential;

        return Point2D{x_distorted, y_distorted};
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

Result<Point2D> project_point(const Point3D& world_point,
                                const Pose& pose,
                                const CameraModel& camera) {
    Point3D camera_point = world_to_camera(world_point, pose);

    Result<Point2D> normalized_result = camera_to_normalized(camera_point);
    if (!normalized_result.ok) {
        return Result<Point2D>::failure(normalized_result.error);
    }

    Point2D distored_norm = distort_normalized(normalized_result.value, camera.distortion);

    Point2D pixel = normalized_to_pixel(distored_norm, camera.intrinsics);

    return Result<Point2D>::success(pixel);
}

}