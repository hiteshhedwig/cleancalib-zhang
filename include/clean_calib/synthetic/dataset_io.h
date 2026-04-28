#pragma once

#include "clean_calib/synthetic/dataset.h"
#include "clean_calib/util/result.h"

#include <string>

namespace clean_calib::synthetic {

// Saves a synthetic calibration dataset to a simple whitespace-based text file.
// The file preserves the key calibration correspondence:
//     dataset.object_points[i] <-> dataset.views[j].image_points[i]
// Returns Result<bool> because the project's Result<T> does not support void.
Result<bool> save_dataset_txt(const SyntheticCalibrationDataset& dataset,
                              const std::string& path);

// Loads a synthetic calibration dataset saved by save_dataset_txt(...).
Result<SyntheticCalibrationDataset> load_dataset_txt(const std::string& path);

}  // namespace clean_calib::synthetic