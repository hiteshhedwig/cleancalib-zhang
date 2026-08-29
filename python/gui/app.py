"""Local Flask application for interactive camera-calibration comparison."""

import os
import sys
import tempfile
import time
from pathlib import Path

from flask import Flask, jsonify, render_template, request
from werkzeug.utils import secure_filename


SOURCE_ROOT = Path(__file__).resolve().parents[2]
SUPPORTED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def _load_binding():
    candidates = []
    configured = os.environ.get("CLEAN_CALIB_PYTHON_PATH")
    if configured:
        candidates.append(Path(configured))
    candidates.extend(
        [SOURCE_ROOT / "build-python" / "python", SOURCE_ROOT / "build" / "python"]
    )
    for candidate in candidates:
        if (candidate / "clean_calib").is_dir():
            sys.path.insert(0, str(candidate))
            break
    try:
        import clean_calib
    except ImportError as error:
        searched = ", ".join(str(path) for path in candidates)
        raise RuntimeError(
            "clean-calib Python binding was not found. Build with "
            "-DCLEAN_CALIB_BUILD_PYTHON=ON or set CLEAN_CALIB_PYTHON_PATH. "
            f"Searched: {searched}"
        ) from error
    return clean_calib


clean_calib = _load_binding()

try:
    from .opencv_compare import calibrate_opencv
except ImportError:
    from opencv_compare import calibrate_opencv


app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = 512 * 1024 * 1024


def _positive_int(form, name, default):
    try:
        value = int(form.get(name, default))
    except ValueError as error:
        raise ValueError(f"{name} must be an integer") from error
    if value < 2:
        raise ValueError(f"{name} must be at least 2")
    return value


def _positive_float(form, name, default):
    try:
        value = float(form.get(name, default))
    except ValueError as error:
        raise ValueError(f"{name} must be a number") from error
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


def _successful_names(result):
    return {image["name"] for image in result.get("images", []) if image["found"]}


@app.get("/")
def index():
    return render_template("index.html")


@app.get("/api/health")
def health():
    try:
        import cv2
        opencv = {"available": True, "version": cv2.__version__}
    except (ImportError, AttributeError) as error:
        opencv = {"available": False, "error": str(error)}
    return jsonify({"clean_calib": True, "opencv": opencv})


@app.post("/api/calibrate")
def calibrate_api():
    try:
        rows = _positive_int(request.form, "rows", 7)
        cols = _positive_int(request.form, "cols", 6)
        square_size = _positive_float(request.form, "square_size", 1.0)
    except ValueError as error:
        return jsonify({"error": str(error)}), 400

    uploads = [file for file in request.files.getlist("images") if file.filename]
    if not uploads:
        return jsonify({"error": "select at least one image"}), 400
    if len(uploads) > 200:
        return jsonify({"error": "at most 200 images can be processed at once"}), 400

    with tempfile.TemporaryDirectory(prefix="clean-calib-web-") as temporary:
        upload_directory = Path(temporary)
        paths = []
        for index, upload in enumerate(uploads):
            original_name = Path(upload.filename).name
            suffix = Path(original_name).suffix.lower()
            if suffix not in SUPPORTED_EXTENSIONS:
                continue
            safe_name = secure_filename(original_name) or f"image{index}{suffix}"
            target_directory = upload_directory / f"{index:04d}"
            target_directory.mkdir()
            target = target_directory / safe_name
            upload.save(target)
            paths.append(target)
        if not paths:
            return jsonify({"error": "no supported image files were selected"}), 400

        clean_started = time.perf_counter()
        try:
            clean_result = clean_calib.calibrate(paths, rows, cols, square_size)
            clean_result["success"] = True
        except (RuntimeError, ValueError) as error:
            clean_result = {
                "implementation": "clean-calib",
                "success": False,
                "error": str(error),
                "images_total": len(paths),
                "detected": 0,
                "images": [],
            }
        clean_result["total_time_ms"] = (time.perf_counter() - clean_started) * 1000.0

        opencv_started = time.perf_counter()
        try:
            opencv_result = calibrate_opencv(paths, rows, cols, square_size)
        except RuntimeError as error:
            opencv_result = {
                "implementation": "OpenCV classic",
                "success": False,
                "error": str(error),
                "images_total": len(paths),
                "detected": 0,
                "images": [],
            }
        opencv_result["total_time_ms"] = (
            time.perf_counter() - opencv_started
        ) * 1000.0

        comparison = {"shared_images": 0}
        if clean_result.get("success") and opencv_result.get("success"):
            common_names = _successful_names(clean_result) & _successful_names(opencv_result)
            comparison["shared_images"] = len(common_names)
            if len(common_names) >= 3:
                common_paths = [path for path in paths if path.name in common_names]
                if (len(common_names) == clean_result["detected"] ==
                        opencv_result["detected"]):
                    clean_common = clean_result
                    opencv_common = opencv_result
                else:
                    clean_common = clean_calib.calibrate(
                        common_paths, rows, cols, square_size
                    )
                    opencv_common = calibrate_opencv(
                        common_paths, rows, cols, square_size
                    )
                comparison.update(
                    clean_calib_rms_px=clean_common["rms_px"],
                    opencv_rms_px=opencv_common["rms_px"],
                )

        return jsonify(
            {
                "dataset": {
                    "images": len(paths),
                    "rows": rows,
                    "cols": cols,
                    "square_size": square_size,
                },
                "clean_calib": clean_result,
                "opencv": opencv_result,
                "comparison": comparison,
            }
        )


@app.errorhandler(413)
def too_large(_error):
    return jsonify({"error": "upload exceeds the 512 MB request limit"}), 413


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=int(os.environ.get("PORT", "5000")), debug=False)
