#include "clean_calib/synthetic/dataset.h"

#include "clean_calib/calib/projection.h"
#include "clean_calib/synthetic/checkerboard.h"

#include <string>

namespace clean_calib::synthetic {

Result<std::vector<Point2D>> project_board(
    const std::vector<Point3D>& object_points,
    const Pose& pose,
    const CameraModel& camera
) {
    if (object_points.empty()) {
        return Result<std::vector<Point2D>>::failure(
            "project_board: object points list is empty"
        );
    }

    std::vector<Point2D> image_points;
    image_points.reserve(object_points.size());

    for (const Point3D& object_point : object_points) {
        Result<Point2D> projected = clean_calib::calib::project_point(
            object_point,
            pose,
            camera
        );

        if (!projected.ok) {
            return Result<std::vector<Point2D>>::failure(
                "project_board: failed to project point: " + projected.error
            );
        }

        image_points.push_back(projected.value);
    }

    return Result<std::vector<Point2D>>::success(image_points);
}

Result<SyntheticCalibrationDataset> generate_calibration_dataset(
    int rows,
    int cols,
    double square_size,
    const CameraModel& camera,
    const std::vector<Pose>& poses
) {
    if (rows <= 0) {
        return Result<SyntheticCalibrationDataset>::failure(
            "generate_calibration_dataset: rows must be positive"
        );
    }

    if (cols <= 0) {
        return Result<SyntheticCalibrationDataset>::failure(
            "generate_calibration_dataset: cols must be positive"
        );
    }

    if (square_size <= 0.0) {
        return Result<SyntheticCalibrationDataset>::failure(
            "generate_calibration_dataset: square_size must be positive"
        );
    }

    if (poses.empty()) {
        return Result<SyntheticCalibrationDataset>::failure(
            "generate_calibration_dataset: poses list is empty"
        );
    }

    SyntheticCalibrationDataset dataset;
    dataset.object_points = generate_planar_board(rows, cols, square_size);

    if (dataset.object_points.empty()) {
        return Result<SyntheticCalibrationDataset>::failure(
            "generate_calibration_dataset: failed to generate checkerboard object points"
        );
    }

    dataset.views.reserve(poses.size());

    for (std::size_t i = 0; i < poses.size(); ++i) {
        const Pose& pose = poses[i];

        Result<std::vector<Point2D>> projected = project_board(
            dataset.object_points,
            pose,
            camera
        );

        if (!projected.ok) {
            return Result<SyntheticCalibrationDataset>::failure(
                "generate_calibration_dataset: failed to project view " +
                std::to_string(i) +
                ": " +
                projected.error
            );
        }

        SyntheticView view;
        view.pose = pose;
        view.image_points = projected.value;

        dataset.views.push_back(view);
    }

    return Result<SyntheticCalibrationDataset>::success(dataset);
}

}