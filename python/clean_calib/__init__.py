"""Python interface to the dependency-free clean-calib C++ implementation."""

from ._clean_calib_native import calibrate as _native_calibrate


def calibrate(image_paths, rows, cols, square_size=1.0):
    """Detect a checkerboard and calibrate a camera from image paths.

    Args:
        image_paths: Sequence of filesystem image paths.
        rows: Number of checkerboard inner-corner rows.
        cols: Number of checkerboard inner-corner columns.
        square_size: Physical checker square size. Intrinsics are independent
            of this scale; translations use the same unit.

    Returns:
        A dictionary containing detections, intrinsics, distortion, and RMS
        reprojection metrics.
    """
    paths = [str(path) for path in image_paths]
    if not paths:
        raise ValueError("image_paths must contain at least one image")
    return _native_calibrate(paths, int(rows), int(cols), float(square_size))


__all__ = ["calibrate"]
