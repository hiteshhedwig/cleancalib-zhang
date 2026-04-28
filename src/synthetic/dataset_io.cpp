#include "clean_calib/synthetic/dataset_io.h"

#include <cstddef>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace clean_calib::synthetic {
namespace {

constexpr const char* kHeader = "CLEAN_CALIB_SYNTHETIC_DATASET_V1";

Result<bool> validate_dataset_for_save(const SyntheticCalibrationDataset& dataset) {
    if (dataset.object_points.empty()) {
        return Result<bool>::failure(
            "save_dataset_txt: dataset has no object points"
        );
    }

    if (dataset.views.empty()) {
        return Result<bool>::failure(
            "save_dataset_txt: dataset has no views"
        );
    }

    for (std::size_t i = 0; i < dataset.views.size(); ++i) {
        const SyntheticView& view = dataset.views[i];

        if (view.image_points.size() != dataset.object_points.size()) {
            return Result<bool>::failure(
                "save_dataset_txt: view " +
                std::to_string(i) +
                " has " +
                std::to_string(view.image_points.size()) +
                " image points, expected " +
                std::to_string(dataset.object_points.size())
            );
        }
    }

    return Result<bool>::success(true);
}

bool read_expected_token(std::istream& in, const std::string& expected) {
    std::string token;
    if (!(in >> token)) {
        return false;
    }
    return token == expected;
}

}  // namespace

Result<bool> save_dataset_txt(const SyntheticCalibrationDataset& dataset,
                              const std::string& path) {
    Result<bool> validation = validate_dataset_for_save(dataset);
    if (!validation.ok) {
        return validation;
    }

    std::ofstream out(path);
    if (!out) {
        return Result<bool>::failure(
            "save_dataset_txt: failed to open output file: " + path
        );
    }

    out << std::setprecision(17);

    out << kHeader << "\n";

    out << "object_points " << dataset.object_points.size() << "\n";
    for (const Point3D& p : dataset.object_points) {
        out << p.x << " " << p.y << " " << p.z << "\n";
    }

    out << "views " << dataset.views.size() << "\n";

    for (std::size_t view_index = 0; view_index < dataset.views.size(); ++view_index) {
        const SyntheticView& view = dataset.views[view_index];

        out << "view " << view_index << "\n";

        out << "pose\n";
        for (int r = 0; r < 3; ++r) {
            out << view.pose.R(r, 0) << " "
                << view.pose.R(r, 1) << " "
                << view.pose.R(r, 2) << "\n";
        }

        out << "t "
            << view.pose.t(0) << " "
            << view.pose.t(1) << " "
            << view.pose.t(2) << "\n";

        out << "image_points " << view.image_points.size() << "\n";
        for (const Point2D& p : view.image_points) {
            out << p.x << " " << p.y << "\n";
        }

        out << "end_view\n";
    }

    if (!out) {
        return Result<bool>::failure(
            "save_dataset_txt: failed while writing output file: " + path
        );
    }

    return Result<bool>::success(true);
}

Result<SyntheticCalibrationDataset> load_dataset_txt(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        return Result<SyntheticCalibrationDataset>::failure(
            "load_dataset_txt: failed to open input file: " + path
        );
    }

    std::string header;
    if (!(in >> header) || header != kHeader) {
        return Result<SyntheticCalibrationDataset>::failure(
            "load_dataset_txt: invalid or missing file header"
        );
    }

    if (!read_expected_token(in, "object_points")) {
        return Result<SyntheticCalibrationDataset>::failure(
            "load_dataset_txt: expected token 'object_points'"
        );
    }

    std::size_t object_point_count = 0;
    if (!(in >> object_point_count)) {
        return Result<SyntheticCalibrationDataset>::failure(
            "load_dataset_txt: failed to read object point count"
        );
    }

    if (object_point_count == 0) {
        return Result<SyntheticCalibrationDataset>::failure(
            "load_dataset_txt: object point count must be positive"
        );
    }

    SyntheticCalibrationDataset dataset;
    dataset.object_points.reserve(object_point_count);

    for (std::size_t i = 0; i < object_point_count; ++i) {
        Point3D p;
        if (!(in >> p.x >> p.y >> p.z)) {
            return Result<SyntheticCalibrationDataset>::failure(
                "load_dataset_txt: failed to read object point " +
                std::to_string(i)
            );
        }
        dataset.object_points.push_back(p);
    }

    if (!read_expected_token(in, "views")) {
        return Result<SyntheticCalibrationDataset>::failure(
            "load_dataset_txt: expected token 'views'"
        );
    }

    std::size_t view_count = 0;
    if (!(in >> view_count)) {
        return Result<SyntheticCalibrationDataset>::failure(
            "load_dataset_txt: failed to read view count"
        );
    }

    if (view_count == 0) {
        return Result<SyntheticCalibrationDataset>::failure(
            "load_dataset_txt: view count must be positive"
        );
    }

    dataset.views.reserve(view_count);

    for (std::size_t view_index = 0; view_index < view_count; ++view_index) {
        if (!read_expected_token(in, "view")) {
            return Result<SyntheticCalibrationDataset>::failure(
                "load_dataset_txt: expected token 'view'"
            );
        }

        std::size_t file_view_index = 0;
        if (!(in >> file_view_index)) {
            return Result<SyntheticCalibrationDataset>::failure(
                "load_dataset_txt: failed to read view index"
            );
        }

        if (file_view_index != view_index) {
            return Result<SyntheticCalibrationDataset>::failure(
                "load_dataset_txt: unexpected view index"
            );
        }

        if (!read_expected_token(in, "pose")) {
            return Result<SyntheticCalibrationDataset>::failure(
                "load_dataset_txt: expected token 'pose'"
            );
        }

        SyntheticView view;

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                if (!(in >> view.pose.R(r, c))) {
                    return Result<SyntheticCalibrationDataset>::failure(
                        "load_dataset_txt: failed to read pose rotation"
                    );
                }
            }
        }

        if (!read_expected_token(in, "t")) {
            return Result<SyntheticCalibrationDataset>::failure(
                "load_dataset_txt: expected token 't'"
            );
        }

        if (!(in >> view.pose.t(0) >> view.pose.t(1) >> view.pose.t(2))) {
            return Result<SyntheticCalibrationDataset>::failure(
                "load_dataset_txt: failed to read pose translation"
            );
        }

        if (!read_expected_token(in, "image_points")) {
            return Result<SyntheticCalibrationDataset>::failure(
                "load_dataset_txt: expected token 'image_points'"
            );
        }

        std::size_t image_point_count = 0;
        if (!(in >> image_point_count)) {
            return Result<SyntheticCalibrationDataset>::failure(
                "load_dataset_txt: failed to read image point count"
            );
        }

        if (image_point_count != object_point_count) {
            return Result<SyntheticCalibrationDataset>::failure(
                "load_dataset_txt: image point count does not match object point count"
            );
        }

        view.image_points.reserve(image_point_count);

        for (std::size_t i = 0; i < image_point_count; ++i) {
            Point2D p;
            if (!(in >> p.x >> p.y)) {
                return Result<SyntheticCalibrationDataset>::failure(
                    "load_dataset_txt: failed to read image point " +
                    std::to_string(i)
                );
            }
            view.image_points.push_back(p);
        }

        if (!read_expected_token(in, "end_view")) {
            return Result<SyntheticCalibrationDataset>::failure(
                "load_dataset_txt: expected token 'end_view'"
            );
        }

        dataset.views.push_back(view);
    }

    return Result<SyntheticCalibrationDataset>::success(dataset);
}

}  // namespace clean_calib::synthetic