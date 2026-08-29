#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "clean_calib/calib/homography.h"
#include "clean_calib/calib/refinement.h"
#include "clean_calib/calib/zhang.h"
#include "clean_calib/detection/checkerboard_detector.h"
#include "clean_calib/image/image_io.h"

#include <Eigen/Core>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

class PyOwned {
public:
    explicit PyOwned(PyObject* value = nullptr) : value_(value) {}
    ~PyOwned() { Py_XDECREF(value_); }
    PyOwned(const PyOwned&) = delete;
    PyOwned& operator=(const PyOwned&) = delete;
    PyOwned(PyOwned&& other) noexcept : value_(other.value_) {
        other.value_ = nullptr;
    }
    PyObject* get() const { return value_; }
    PyObject* release() {
        PyObject* value = value_;
        value_ = nullptr;
        return value;
    }

private:
    PyObject* value_;
};

void dict_set(PyObject* dictionary, const char* key, PyObject* value) {
    if (value == nullptr || PyDict_SetItemString(dictionary, key, value) != 0) {
        Py_XDECREF(value);
        throw std::runtime_error("failed to construct Python result");
    }
    Py_DECREF(value);
}

void dict_set_double(PyObject* dictionary, const char* key, double value) {
    dict_set(dictionary, key, PyFloat_FromDouble(value));
}

void dict_set_int(PyObject* dictionary, const char* key, long value) {
    dict_set(dictionary, key, PyLong_FromLong(value));
}

void dict_set_bool(PyObject* dictionary, const char* key, bool value) {
    dict_set(dictionary, key, PyBool_FromLong(value ? 1 : 0));
}

void dict_set_string(PyObject* dictionary, const char* key,
                     const std::string& value) {
    dict_set(dictionary, key,
             PyUnicode_FromStringAndSize(value.data(),
                                         static_cast<Py_ssize_t>(value.size())));
}

std::vector<std::string> parse_paths(PyObject* paths_object) {
    PyOwned sequence(PySequence_Fast(paths_object,
                                     "image_paths must be a sequence"));
    if (sequence.get() == nullptr) {
        throw std::runtime_error("image_paths must be a sequence");
    }
    const Py_ssize_t count = PySequence_Fast_GET_SIZE(sequence.get());
    if (count == 0) {
        throw std::invalid_argument("image_paths must not be empty");
    }
    std::vector<std::string> paths;
    paths.reserve(static_cast<std::size_t>(count));
    PyObject** items = PySequence_Fast_ITEMS(sequence.get());
    for (Py_ssize_t index = 0; index < count; ++index) {
        const char* path = PyUnicode_AsUTF8(items[index]);
        if (path == nullptr) {
            throw std::invalid_argument("every image path must be a string");
        }
        paths.emplace_back(path);
    }
    return paths;
}

std::vector<clean_calib::Point2D> make_planar_points(int rows, int cols,
                                                      double square_size) {
    std::vector<clean_calib::Point2D> points;
    points.reserve(static_cast<std::size_t>(rows * cols));
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            points.push_back({col * square_size, row * square_size});
        }
    }
    return points;
}

std::vector<clean_calib::Point3D> make_object_points(int rows, int cols,
                                                      double square_size) {
    std::vector<clean_calib::Point3D> points;
    points.reserve(static_cast<std::size_t>(rows * cols));
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            points.push_back({col * square_size, row * square_size, 0.0});
        }
    }
    return points;
}

struct DetectionRecord {
    std::string path;
    bool found = false;
    std::string error;
    int corner_count = 0;
    double grid_line_error = 0.0;
    double homography_rms = 0.0;
    double final_rms = 0.0;
    std::vector<clean_calib::Point2D> corners;
};

PyObject* calibration_to_python(const std::vector<std::string>& paths,
                                int rows, int cols, double square_size) {
    const std::vector<clean_calib::Point2D> planar_points =
        make_planar_points(rows, cols, square_size);
    const std::vector<clean_calib::Point3D> object_points =
        make_object_points(rows, cols, square_size);
    std::vector<DetectionRecord> records;
    std::vector<std::vector<clean_calib::Point2D>> image_points;
    std::vector<Eigen::Matrix3d> homographies;

    for (const std::string& path : paths) {
        DetectionRecord record;
        record.path = path;
        const auto loaded = clean_calib::image::load(path);
        if (!loaded.ok) {
            record.error = loaded.error;
            records.push_back(std::move(record));
            continue;
        }
        clean_calib::detection::CheckerboardDetectionOptions options;
        options.rows = rows;
        options.cols = cols;
        const auto detected = clean_calib::detection::detect_checkerboard(
            loaded.value, options);
        if (!detected.ok) {
            record.error = detected.error;
            records.push_back(std::move(record));
            continue;
        }
        const auto homography = clean_calib::calib::estimate_homography(
            planar_points, detected.value.corners);
        if (!homography.ok) {
            record.error = homography.error;
            records.push_back(std::move(record));
            continue;
        }
        const auto homography_rms =
            clean_calib::calib::homography_rms_reprojection_error(
                homography.value, planar_points, detected.value.corners);
        if (!homography_rms.ok) {
            record.error = homography_rms.error;
            records.push_back(std::move(record));
            continue;
        }
        record.found = true;
        record.corner_count = static_cast<int>(detected.value.corners.size());
        record.grid_line_error = detected.value.grid_line_error;
        record.homography_rms = homography_rms.value;
        record.corners = detected.value.corners;
        image_points.push_back(record.corners);
        homographies.push_back(homography.value);
        records.push_back(std::move(record));
    }

    if (image_points.size() < 3) {
        throw std::runtime_error(
            "at least 3 successfully detected checkerboard views are required");
    }
    const auto initialized = clean_calib::calib::initialize_zhang(homographies);
    if (!initialized.ok) {
        throw std::runtime_error(initialized.error);
    }
    clean_calib::CameraModel initial_camera;
    initial_camera.intrinsics = initialized.value.intrinsics;
    clean_calib::calib::RefinementOptions refinement_options;
    refinement_options.max_iterations = 60;
    const auto refined =
        clean_calib::calib::refine_calibration_levenberg_marquardt(
            object_points, image_points, initial_camera,
            initialized.value.poses, refinement_options);
    if (!refined.ok) {
        throw std::runtime_error(refined.error);
    }

    const auto residuals = clean_calib::calib::calibration_residuals(
        object_points, image_points, refined.value.camera, refined.value.poses);
    if (!residuals.ok) {
        throw std::runtime_error(residuals.error);
    }
    const int residual_count_per_view = rows * cols * 2;
    std::size_t successful_view = 0;
    for (DetectionRecord& record : records) {
        if (!record.found) {
            continue;
        }
        const Eigen::VectorXd view_residuals = residuals.value.segment(
            static_cast<Eigen::Index>(successful_view * residual_count_per_view),
            residual_count_per_view);
        record.final_rms = std::sqrt(
            view_residuals.squaredNorm() / static_cast<double>(rows * cols));
        ++successful_view;
    }

    PyOwned result(PyDict_New());
    if (result.get() == nullptr) {
        throw std::runtime_error("failed to allocate result dictionary");
    }
    dict_set_string(result.get(), "implementation", "clean-calib");
    dict_set_int(result.get(), "images_total", static_cast<long>(paths.size()));
    dict_set_int(result.get(), "detected",
                 static_cast<long>(image_points.size()));
    dict_set_double(result.get(), "detection_rate_percent",
                    100.0 * image_points.size() / paths.size());
    dict_set_double(result.get(), "initial_rms_px",
                    refined.value.initial_rms_reprojection_error);
    dict_set_double(result.get(), "rms_px",
                    refined.value.final_rms_reprojection_error);
    dict_set_bool(result.get(), "converged", refined.value.converged);
    dict_set_int(result.get(), "iterations", refined.value.iterations);

    const auto& intrinsics = refined.value.camera.intrinsics;
    PyOwned intrinsics_dict(PyDict_New());
    dict_set_double(intrinsics_dict.get(), "fx", intrinsics.fx);
    dict_set_double(intrinsics_dict.get(), "fy", intrinsics.fy);
    dict_set_double(intrinsics_dict.get(), "cx", intrinsics.cx);
    dict_set_double(intrinsics_dict.get(), "cy", intrinsics.cy);
    dict_set_double(intrinsics_dict.get(), "skew", intrinsics.skew);
    dict_set(result.get(), "intrinsics", intrinsics_dict.release());

    const auto& distortion = refined.value.camera.distortion;
    PyOwned distortion_dict(PyDict_New());
    dict_set_double(distortion_dict.get(), "k1", distortion.k1);
    dict_set_double(distortion_dict.get(), "k2", distortion.k2);
    dict_set_double(distortion_dict.get(), "k3", distortion.k3);
    dict_set_double(distortion_dict.get(), "p1", distortion.p1);
    dict_set_double(distortion_dict.get(), "p2", distortion.p2);
    dict_set(result.get(), "distortion", distortion_dict.release());

    PyOwned images(PyList_New(static_cast<Py_ssize_t>(records.size())));
    if (images.get() == nullptr) {
        throw std::runtime_error("failed to allocate image result list");
    }
    for (std::size_t index = 0; index < records.size(); ++index) {
        const DetectionRecord& record = records[index];
        PyOwned image(PyDict_New());
        dict_set_string(image.get(), "name",
                        std::filesystem::path(record.path).filename().string());
        dict_set_bool(image.get(), "found", record.found);
        dict_set_int(image.get(), "corner_count", record.corner_count);
        if (record.found) {
            dict_set_double(image.get(), "grid_line_error",
                            record.grid_line_error);
            dict_set_double(image.get(), "homography_rms_px",
                            record.homography_rms);
            dict_set_double(image.get(), "rms_px", record.final_rms);
        } else {
            dict_set_string(image.get(), "error", record.error);
        }
        PyList_SET_ITEM(images.get(), static_cast<Py_ssize_t>(index),
                        image.release());
    }
    dict_set(result.get(), "images", images.release());
    return result.release();
}

PyObject* py_calibrate(PyObject*, PyObject* args, PyObject* kwargs) {
    PyObject* paths_object = nullptr;
    int rows = 0;
    int cols = 0;
    double square_size = 1.0;
    const char* keywords[] = {"image_paths", "rows", "cols", "square_size",
                              nullptr};
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "Oiid", const_cast<char**>(keywords),
                                     &paths_object, &rows, &cols, &square_size)) {
        return nullptr;
    }
    try {
        if (rows < 2 || cols < 2) {
            throw std::invalid_argument("rows and cols must both be at least 2");
        }
        if (!std::isfinite(square_size) || square_size <= 0.0) {
            throw std::invalid_argument("square_size must be finite and positive");
        }
        return calibration_to_python(parse_paths(paths_object), rows, cols,
                                     square_size);
    } catch (const std::invalid_argument& error) {
        PyErr_SetString(PyExc_ValueError, error.what());
    } catch (const std::exception& error) {
        PyErr_SetString(PyExc_RuntimeError, error.what());
    }
    return nullptr;
}

PyMethodDef module_methods[] = {
    {"calibrate", reinterpret_cast<PyCFunction>(py_calibrate),
     METH_VARARGS | METH_KEYWORDS,
     "Detect checkerboards and calibrate a camera from image paths."},
    {nullptr, nullptr, 0, nullptr},
};

PyModuleDef module_definition = {
    PyModuleDef_HEAD_INIT,
    "_clean_calib_native",
    "Native bindings for clean-calib.",
    -1,
    module_methods,
};

}  // namespace

PyMODINIT_FUNC PyInit__clean_calib_native() {
    return PyModule_Create(&module_definition);
}
