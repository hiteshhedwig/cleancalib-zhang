#pragma once

#include "clean_calib/core/camera.h"
#include "clean_calib/core/point.h"
#include "clean_calib/core/pose.h"
#include "clean_calib/util/result.h"

#include <vector>

namespace clean_calib::synthetic {

struct SyntheticView {
    Pose pose;
    std::vector<Point2D> image_points;
};


struct SyntheticCalibrationDataset {
    std::vector<Point3D> object_points;
    std::vector<SyntheticView> views;
};

Result<std::vector<Point2D>> project_board(
    const std::vector<Point3D>& object_points,
    const Pose& pose,
    const CameraModel& camera
);


Result<SyntheticCalibrationDataset> generate_calibration_dataset(
    int rows,
    int cols,
    double square_size,
    const CameraModel& camera,
    const std::vector<Pose>& poses
);

}  // namespace clean_calib::synthetic