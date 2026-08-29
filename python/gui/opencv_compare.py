"""OpenCV reference pipeline used only by the local comparison GUI."""

import math
import time
from pathlib import Path


def _imports():
    try:
        import cv2
        import numpy as np
    except (ImportError, AttributeError) as error:
        raise RuntimeError(
            "OpenCV comparison is unavailable. Use a Python environment with "
            "compatible cv2 and NumPy installations."
        ) from error
    return cv2, np


def calibrate_opencv(image_paths, rows, cols, square_size):
    cv2, np = _imports()
    object_points_template = np.zeros((rows * cols, 3), np.float32)
    object_points_template[:, :2] = (
        np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size
    )
    object_points = []
    image_points = []
    records = []
    image_size = None
    detection_seconds = 0.0

    for image_path in image_paths:
        record = {"name": Path(image_path).name, "found": False, "corner_count": 0}
        gray = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            record["error"] = "OpenCV could not decode the image"
            records.append(record)
            continue
        current_size = (int(gray.shape[1]), int(gray.shape[0]))
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            record["error"] = (
                f"image size {current_size} differs from the first image {image_size}"
            )
            records.append(record)
            continue

        start = time.perf_counter()
        found, corners = cv2.findChessboardCorners(gray, (cols, rows))
        if found:
            corners = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
        detection_seconds += time.perf_counter() - start
        if not found:
            record["error"] = "checkerboard not found"
            records.append(record)
            continue

        record["found"] = True
        record["corner_count"] = int(len(corners))
        record["_view_index"] = len(image_points)
        records.append(record)
        object_points.append(object_points_template.copy())
        image_points.append(corners)

    result = {
        "implementation": f"OpenCV {cv2.__version__} classic",
        "images_total": len(image_paths),
        "detected": len(image_points),
        "detection_rate_percent": 100.0 * len(image_points) / len(image_paths),
        "detection_time_ms": detection_seconds * 1000.0,
        "images": records,
    }
    if len(image_points) < 3:
        result.update(
            success=False,
            error="at least 3 successfully detected checkerboard views are required",
        )
        for record in records:
            record.pop("_view_index", None)
        return result

    calibration_start = time.perf_counter()
    rms, camera_matrix, distortion, rotation_vectors, translation_vectors = (
        cv2.calibrateCamera(
            object_points, image_points, image_size, None, None
        )
    )
    calibration_seconds = time.perf_counter() - calibration_start
    for record in records:
        view_index = record.pop("_view_index", None)
        if view_index is None:
            continue
        projected, _ = cv2.projectPoints(
            object_points[view_index],
            rotation_vectors[view_index],
            translation_vectors[view_index],
            camera_matrix,
            distortion,
        )
        difference = projected.reshape(-1, 2) - image_points[view_index].reshape(-1, 2)
        record["rms_px"] = math.sqrt(
            float(np.sum(difference * difference)) / len(difference)
        )

    coefficients = distortion.reshape(-1)
    result.update(
        success=True,
        rms_px=float(rms),
        converged=True,
        calibration_time_ms=calibration_seconds * 1000.0,
        intrinsics={
            "fx": float(camera_matrix[0, 0]),
            "fy": float(camera_matrix[1, 1]),
            "cx": float(camera_matrix[0, 2]),
            "cy": float(camera_matrix[1, 2]),
            "skew": 0.0,
        },
        distortion={
            "k1": float(coefficients[0]),
            "k2": float(coefficients[1]),
            "p1": float(coefficients[2]),
            "p2": float(coefficients[3]),
            "k3": float(coefficients[4]),
        },
    )
    return result
