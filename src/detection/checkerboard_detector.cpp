#include "clean_calib/detection/checkerboard_detector.h"

#include "clean_calib/calib/homography.h"
#include "clean_calib/image/image_utils.h"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <string>

namespace clean_calib::detection {
namespace {

std::size_t index_of(int width, int x, int y) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
           static_cast<std::size_t>(x);
}

bool finite_point(const Point2D& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

double bilinear_gray(const Image& grayscale, double x, double y) {
    x = std::clamp(x, 0.0, static_cast<double>(grayscale.width - 1));
    y = std::clamp(y, 0.0, static_cast<double>(grayscale.height - 1));
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, grayscale.width - 1);
    const int y1 = std::min(y0 + 1, grayscale.height - 1);
    const double fraction_x = x - static_cast<double>(x0);
    const double fraction_y = y - static_cast<double>(y0);
    const double top =
        (1.0 - fraction_x) *
            grayscale.data[index_of(grayscale.width, x0, y0)] +
        fraction_x * grayscale.data[index_of(grayscale.width, x1, y0)];
    const double bottom =
        (1.0 - fraction_x) *
            grayscale.data[index_of(grayscale.width, x0, y1)] +
        fraction_x * grayscale.data[index_of(grayscale.width, x1, y1)];
    return (1.0 - fraction_y) * top + fraction_y * bottom;
}

double bilinear_field(const std::vector<double>& field,
                      int width,
                      int height,
                      double x,
                      double y) {
    x = std::clamp(x, 0.0, static_cast<double>(width - 1));
    y = std::clamp(y, 0.0, static_cast<double>(height - 1));
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);
    const double fraction_x = x - x0;
    const double fraction_y = y - y0;
    const double top =
        (1.0 - fraction_x) * field[index_of(width, x0, y0)] +
        fraction_x * field[index_of(width, x1, y0)];
    const double bottom =
        (1.0 - fraction_x) * field[index_of(width, x0, y1)] +
        fraction_x * field[index_of(width, x1, y1)];
    return (1.0 - fraction_y) * top + fraction_y * bottom;
}

double checkerboard_saddle_score(const Image& grayscale,
                                 const Point2D& point,
                                 int base_radius) {
    constexpr int sample_count = 16;
    constexpr double pi = 3.14159265358979323846;
    const std::array<double, 3> radius_scales = {0.75, 1.0, 1.4};
    double score_sum = 0.0;
    int valid_radii = 0;
    for (double radius_scale : radius_scales) {
        const double radius =
            radius_scale * static_cast<double>(std::max(2, base_radius));
        if (point.x - radius < 0.0 || point.y - radius < 0.0 ||
            point.x + radius >= grayscale.width ||
            point.y + radius >= grayscale.height) {
            continue;
        }
        std::array<double, sample_count> samples{};
        for (int sample = 0; sample < sample_count; ++sample) {
            const double angle =
                2.0 * pi * static_cast<double>(sample) /
                static_cast<double>(sample_count);
            samples[static_cast<std::size_t>(sample)] = bilinear_gray(
                grayscale, point.x + radius * std::cos(angle),
                point.y + radius * std::sin(angle));
        }
        double quarter_turn_contrast = 0.0;
        double opposite_error = 0.0;
        for (int sample = 0; sample < sample_count; ++sample) {
            quarter_turn_contrast += std::abs(
                samples[static_cast<std::size_t>(sample)] -
                samples[static_cast<std::size_t>(
                    (sample + sample_count / 4) % sample_count)]);
            opposite_error += std::abs(
                samples[static_cast<std::size_t>(sample)] -
                samples[static_cast<std::size_t>(
                    (sample + sample_count / 2) % sample_count)]);
        }
        quarter_turn_contrast /= sample_count;
        opposite_error /= sample_count;
        score_sum += std::max(0.0, quarter_turn_contrast - opposite_error);
        ++valid_radii;
    }
    return valid_radii > 0 ? score_sum / static_cast<double>(valid_radii) : 0.0;
}

Result<std::vector<CornerCandidate>> score_checkerboard_candidates(
    const Image& image,
    const std::vector<CornerCandidate>& candidates,
    int base_radius) {
    const Image grayscale = to_grayscale(image);
    if (grayscale.empty()) {
        return Result<std::vector<CornerCandidate>>::failure(
            "detect_checkerboard: grayscale conversion failed during candidate scoring");
    }
    double maximum_harris = 0.0;
    for (const CornerCandidate& candidate : candidates) {
        maximum_harris = std::max(maximum_harris, candidate.response);
    }
    std::vector<CornerCandidate> scored = candidates;
    for (CornerCandidate& candidate : scored) {
        const double saddle = checkerboard_saddle_score(
            grayscale, candidate.point, base_radius);
        const double harris_weight = maximum_harris > 0.0
                                         ? std::sqrt(std::max(
                                               0.0, candidate.response /
                                                        maximum_harris))
                                         : 0.0;
        candidate.response = saddle * (0.5 + 0.5 * harris_weight);
    }
    std::stable_sort(
        scored.begin(), scored.end(),
        [](const CornerCandidate& first, const CornerCandidate& second) {
            return first.response > second.response;
        });
    return Result<std::vector<CornerCandidate>>::success(std::move(scored));
}

double turn(const Point2D& origin,
            const Point2D& first,
            const Point2D& second) {
    return (first.x - origin.x) * (second.y - origin.y) -
           (first.y - origin.y) * (second.x - origin.x);
}

std::vector<Point2D> convex_hull(
    const std::vector<CornerCandidate>& candidates) {
    std::vector<Point2D> points;
    points.reserve(candidates.size());
    for (const CornerCandidate& candidate : candidates) {
        points.push_back(candidate.point);
    }
    std::sort(points.begin(), points.end(),
              [](const Point2D& first, const Point2D& second) {
                  return first.x < second.x ||
                         (first.x == second.x && first.y < second.y);
              });
    std::vector<Point2D> hull;
    for (const Point2D& point : points) {
        while (hull.size() >= 2 &&
               turn(hull[hull.size() - 2], hull.back(), point) <= 1e-9) {
            hull.pop_back();
        }
        hull.push_back(point);
    }
    const std::size_t lower_size = hull.size();
    for (auto iterator = points.rbegin() + 1; iterator != points.rend();
         ++iterator) {
        while (hull.size() > lower_size &&
               turn(hull[hull.size() - 2], hull.back(), *iterator) <= 1e-9) {
            hull.pop_back();
        }
        hull.push_back(*iterator);
    }
    if (!hull.empty()) {
        hull.pop_back();
    }
    return hull;
}

Result<bool> validate_response(const HarrisResponse& response,
                               const char* function_name) {
    if (response.width <= 0 || response.height <= 0 ||
        response.values.size() !=
            static_cast<std::size_t>(response.width) *
                static_cast<std::size_t>(response.height)) {
        return Result<bool>::failure(
            std::string(function_name) + ": response dimensions are invalid");
    }
    for (double value : response.values) {
        if (!std::isfinite(value)) {
            return Result<bool>::failure(
                std::string(function_name) + ": response is non-finite");
        }
    }
    return Result<bool>::success(true);
}

double squared_distance(const Point2D& first, const Point2D& second) {
    const double dx = first.x - second.x;
    const double dy = first.y - second.y;
    return dx * dx + dy * dy;
}

double point_line_distance(const Point2D& point,
                           const Point2D& line_start,
                           const Point2D& line_end) {
    const double dx = line_end.x - line_start.x;
    const double dy = line_end.y - line_start.y;
    const double length = std::hypot(dx, dy);
    if (length <= std::numeric_limits<double>::epsilon()) {
        return std::numeric_limits<double>::infinity();
    }
    return std::abs(dx * (line_start.y - point.y) -
                    (line_start.x - point.x) * dy) /
           length;
}

void canonicalize_image_order(std::vector<CornerCandidate>& ordered,
                              int rows,
                              int cols) {
    double first_row_y = 0.0;
    double last_row_y = 0.0;
    for (int col = 0; col < cols; ++col) {
        first_row_y += ordered[static_cast<std::size_t>(col)].point.y;
        last_row_y += ordered[static_cast<std::size_t>((rows - 1) * cols + col)]
                          .point.y;
    }
    if (first_row_y > last_row_y) {
        for (int row = 0; row < rows / 2; ++row) {
            for (int col = 0; col < cols; ++col) {
                std::swap(
                    ordered[static_cast<std::size_t>(row * cols + col)],
                    ordered[static_cast<std::size_t>(
                        (rows - 1 - row) * cols + col)]);
            }
        }
    }

    double first_col_x = 0.0;
    double last_col_x = 0.0;
    for (int row = 0; row < rows; ++row) {
        first_col_x +=
            ordered[static_cast<std::size_t>(row * cols)].point.x;
        last_col_x +=
            ordered[static_cast<std::size_t>(row * cols + cols - 1)].point.x;
    }
    if (first_col_x > last_col_x) {
        for (int row = 0; row < rows; ++row) {
            std::reverse(
                ordered.begin() + static_cast<std::ptrdiff_t>(row * cols),
                ordered.begin() + static_cast<std::ptrdiff_t>((row + 1) * cols));
        }
    }
}

Result<double> grid_geometry_error(
    const std::vector<CornerCandidate>& ordered,
    int rows,
    int cols,
    double minimum_spacing) {
    double spacing_sum = 0.0;
    std::size_t spacing_count = 0;
    double minimum_observed_spacing = std::numeric_limits<double>::infinity();
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col + 1 < cols; ++col) {
            const double spacing = std::sqrt(squared_distance(
                ordered[static_cast<std::size_t>(row * cols + col)].point,
                ordered[static_cast<std::size_t>(row * cols + col + 1)].point));
            spacing_sum += spacing;
            ++spacing_count;
            minimum_observed_spacing =
                std::min(minimum_observed_spacing, spacing);
        }
    }
    for (int row = 0; row + 1 < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const double spacing = std::sqrt(squared_distance(
                ordered[static_cast<std::size_t>(row * cols + col)].point,
                ordered[static_cast<std::size_t>((row + 1) * cols + col)].point));
            spacing_sum += spacing;
            ++spacing_count;
            minimum_observed_spacing =
                std::min(minimum_observed_spacing, spacing);
        }
    }
    if (spacing_count == 0 || minimum_observed_spacing < minimum_spacing) {
        return Result<double>::failure(
            "fit_checkerboard_grid: adjacent corners are too close");
    }
    const double mean_spacing = spacing_sum / static_cast<double>(spacing_count);

    double squared_line_error = 0.0;
    std::size_t line_count = 0;
    for (int row = 0; row < rows; ++row) {
        const Point2D& start =
            ordered[static_cast<std::size_t>(row * cols)].point;
        const Point2D& end =
            ordered[static_cast<std::size_t>(row * cols + cols - 1)].point;
        for (int col = 0; col < cols; ++col) {
            const double error = point_line_distance(
                ordered[static_cast<std::size_t>(row * cols + col)].point,
                start, end);
            squared_line_error += error * error;
            ++line_count;
        }
    }
    for (int col = 0; col < cols; ++col) {
        const Point2D& start =
            ordered[static_cast<std::size_t>(col)].point;
        const Point2D& end =
            ordered[static_cast<std::size_t>((rows - 1) * cols + col)].point;
        for (int row = 0; row < rows; ++row) {
            const double error = point_line_distance(
                ordered[static_cast<std::size_t>(row * cols + col)].point,
                start, end);
            squared_line_error += error * error;
            ++line_count;
        }
    }

    double orientation_sign = 0.0;
    for (int row = 0; row + 1 < rows; ++row) {
        for (int col = 0; col + 1 < cols; ++col) {
            const Point2D& origin =
                ordered[static_cast<std::size_t>(row * cols + col)].point;
            const Point2D& right =
                ordered[static_cast<std::size_t>(row * cols + col + 1)].point;
            const Point2D& down =
                ordered[static_cast<std::size_t>((row + 1) * cols + col)].point;
            const double cross =
                (right.x - origin.x) * (down.y - origin.y) -
                (right.y - origin.y) * (down.x - origin.x);
            if (std::abs(cross) <= 0.02 * mean_spacing * mean_spacing) {
                return Result<double>::failure(
                    "fit_checkerboard_grid: grid contains collapsed cells");
            }
            if (orientation_sign == 0.0) {
                orientation_sign = cross;
            } else if (cross * orientation_sign <= 0.0) {
                return Result<double>::failure(
                    "fit_checkerboard_grid: grid cells have inconsistent orientation");
            }
        }
    }

    const double rms_line_error =
        std::sqrt(squared_line_error / static_cast<double>(line_count));
    return Result<double>::success(rms_line_error / mean_spacing);
}

Result<double> grid_projective_error(
    const std::vector<CornerCandidate>& ordered,
    int rows,
    int cols) {
    std::vector<Point2D> model;
    std::vector<Point2D> image;
    model.reserve(ordered.size());
    image.reserve(ordered.size());
    double spacing_sum = 0.0;
    std::size_t spacing_count = 0;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const std::size_t index =
                static_cast<std::size_t>(row * cols + col);
            model.push_back(
                {static_cast<double>(col), static_cast<double>(row)});
            image.push_back(ordered[index].point);
            if (col + 1 < cols) {
                spacing_sum += std::sqrt(squared_distance(
                    ordered[index].point, ordered[index + 1].point));
                ++spacing_count;
            }
            if (row + 1 < rows) {
                spacing_sum += std::sqrt(squared_distance(
                    ordered[index].point,
                    ordered[index + static_cast<std::size_t>(cols)].point));
                ++spacing_count;
            }
        }
    }
    if (spacing_count == 0) {
        return Result<double>::failure(
            "fit_checkerboard_grid: cannot measure grid spacing");
    }
    Result<Eigen::Matrix3d> homography =
        clean_calib::calib::estimate_homography(model, image);
    if (!homography.ok) {
        return Result<double>::failure(homography.error);
    }
    Result<double> rms =
        clean_calib::calib::homography_rms_reprojection_error(
            homography.value, model, image);
    if (!rms.ok) {
        return rms;
    }
    const double mean_spacing =
        spacing_sum / static_cast<double>(spacing_count);
    return Result<double>::success(rms.value / mean_spacing);
}

std::vector<CornerCandidate> select_affine_lattice_subset(
    const std::vector<CornerCandidate>& candidates,
    int rows,
    int cols,
    double minimum_spacing) {
    const std::size_t expected =
        static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
    if (candidates.size() <= expected) {
        return candidates;
    }
    double maximum_response = 0.0;
    for (const CornerCandidate& candidate : candidates) {
        maximum_response = std::max(maximum_response, candidate.response);
    }
    if (maximum_response <= 0.0) {
        return {};
    }

    double best_quality = -std::numeric_limits<double>::infinity();
    std::vector<CornerCandidate> best;
    const std::size_t neighbor_count =
        std::min<std::size_t>(20, candidates.size() - 1);
    for (std::size_t origin_index = 0;
         origin_index < candidates.size(); ++origin_index) {
        std::vector<std::pair<double, std::size_t>> distances;
        distances.reserve(candidates.size() - 1);
        for (std::size_t candidate = 0;
             candidate < candidates.size(); ++candidate) {
            if (candidate != origin_index) {
                distances.push_back(
                    {squared_distance(candidates[origin_index].point,
                                      candidates[candidate].point),
                     candidate});
            }
        }
        std::partial_sort(
            distances.begin(), distances.begin() +
                                   static_cast<std::ptrdiff_t>(neighbor_count),
            distances.end());

        for (std::size_t first_neighbor = 0;
             first_neighbor < neighbor_count; ++first_neighbor) {
            for (std::size_t second_neighbor = first_neighbor + 1;
                 second_neighbor < neighbor_count; ++second_neighbor) {
                const Point2D& origin = candidates[origin_index].point;
                const Point2D& first_point =
                    candidates[distances[first_neighbor].second].point;
                const Point2D& second_point =
                    candidates[distances[second_neighbor].second].point;
                const Eigen::Vector2d first(first_point.x - origin.x,
                                            first_point.y - origin.y);
                const Eigen::Vector2d second(second_point.x - origin.x,
                                             second_point.y - origin.y);
                const double first_length = first.norm();
                const double second_length = second.norm();
                const double shorter = std::min(first_length, second_length);
                const double longer = std::max(first_length, second_length);
                if (shorter < minimum_spacing || longer > 2.5 * shorter ||
                    std::abs(first.dot(second)) >
                        0.82 * first_length * second_length) {
                    continue;
                }

                for (int swapped = 0; swapped < 2; ++swapped) {
                    const Eigen::Vector2d column_step =
                        swapped == 0 ? first : second;
                    const Eigen::Vector2d row_step =
                        swapped == 0 ? second : first;
                    const double match_radius =
                        0.65 * std::min(column_step.norm(), row_step.norm());
                    const double maximum_squared_error =
                        match_radius * match_radius;
                    std::vector<bool> used(candidates.size(), false);
                    std::vector<CornerCandidate> ordered;
                    ordered.reserve(expected);
                    std::vector<Eigen::Vector2d> matched_points;
                    matched_points.reserve(expected);
                    double squared_error_sum = 0.0;
                    double response_sum = 0.0;
                    bool complete = true;
                    for (int row = 0; row < rows && complete; ++row) {
                        for (int col = 0; col < cols; ++col) {
                            const std::size_t grid_index =
                                static_cast<std::size_t>(row * cols + col);
                            Eigen::Vector2d predicted;
                            if (row == 0 && col == 0) {
                                predicted = Eigen::Vector2d(origin.x, origin.y);
                            } else if (row == 0 && col == 1) {
                                predicted = Eigen::Vector2d(origin.x, origin.y) +
                                            column_step;
                            } else if (row == 0) {
                                predicted =
                                    2.0 * matched_points[grid_index - 1] -
                                    matched_points[grid_index - 2];
                            } else if (col == 0 && row == 1) {
                                predicted = Eigen::Vector2d(origin.x, origin.y) +
                                            row_step;
                            } else if (col == 0) {
                                predicted =
                                    2.0 * matched_points[grid_index - cols] -
                                    matched_points[grid_index - 2 * cols];
                            } else {
                                predicted =
                                    matched_points[grid_index - 1] +
                                    matched_points[grid_index - cols] -
                                    matched_points[grid_index - cols - 1];
                            }
                            std::size_t nearest_index = candidates.size();
                            double nearest_distance =
                                std::numeric_limits<double>::infinity();
                            for (std::size_t candidate = 0;
                                 candidate < candidates.size(); ++candidate) {
                                if (used[candidate]) {
                                    continue;
                                }
                                const Point2D predicted_point{
                                    predicted.x(), predicted.y()};
                                const double distance = squared_distance(
                                    predicted_point, candidates[candidate].point);
                                if (distance < nearest_distance) {
                                    nearest_distance = distance;
                                    nearest_index = candidate;
                                }
                            }
                            if (nearest_index == candidates.size() ||
                                nearest_distance > maximum_squared_error) {
                                complete = false;
                                break;
                            }
                            used[nearest_index] = true;
                            ordered.push_back(candidates[nearest_index]);
                            matched_points.push_back(Eigen::Vector2d(
                                candidates[nearest_index].point.x,
                                candidates[nearest_index].point.y));
                            squared_error_sum += nearest_distance;
                            response_sum += candidates[nearest_index].response;
                        }
                    }
                    if (!complete || ordered.size() != expected) {
                        continue;
                    }
                    Result<double> geometry = grid_geometry_error(
                        ordered, rows, cols, minimum_spacing);
                    if (!geometry.ok || geometry.value > 0.10) {
                        continue;
                    }
                    Result<double> projective =
                        grid_projective_error(ordered, rows, cols);
                    if (!projective.ok || projective.value > 0.16) {
                        continue;
                    }
                    const double mean_step =
                        0.5 * (column_step.norm() + row_step.norm());
                    const double normalized_match_error =
                        std::sqrt(squared_error_sum /
                                  static_cast<double>(expected)) /
                        mean_step;
                    const double mean_response =
                        response_sum /
                        (static_cast<double>(expected) * maximum_response);
                    const double quality =
                        0.03 * mean_response - 0.25 * normalized_match_error -
                        geometry.value - 2.0 * projective.value;
                    if (quality > best_quality) {
                        best_quality = quality;
                        best = std::move(ordered);
                    }
                }
            }
        }
    }
    return best;
}

Result<std::vector<CornerCandidate>> order_with_axes(
    const std::vector<CornerCandidate>& selected,
    const Eigen::Vector2d& row_axis,
    const Eigen::Vector2d& column_axis,
    int rows,
    int cols) {
    std::vector<CornerCandidate> ordered = selected;
    std::sort(ordered.begin(), ordered.end(),
              [&](const CornerCandidate& first, const CornerCandidate& second) {
                  const Eigen::Vector2d first_point(first.point.x, first.point.y);
                  const Eigen::Vector2d second_point(second.point.x, second.point.y);
                  const double first_projection = row_axis.dot(first_point);
                  const double second_projection = row_axis.dot(second_point);
                  if (first_projection != second_projection) {
                      return first_projection < second_projection;
                  }
                  return column_axis.dot(first_point) <
                         column_axis.dot(second_point);
              });
    for (int row = 0; row < rows; ++row) {
        auto begin = ordered.begin() + static_cast<std::ptrdiff_t>(row * cols);
        auto end = begin + cols;
        std::sort(begin, end,
                  [&](const CornerCandidate& first,
                      const CornerCandidate& second) {
                      return column_axis.dot(
                                 Eigen::Vector2d(first.point.x, first.point.y)) <
                             column_axis.dot(
                                 Eigen::Vector2d(second.point.x, second.point.y));
                  });
    }
    canonicalize_image_order(ordered, rows, cols);
    return Result<std::vector<CornerCandidate>>::success(std::move(ordered));
}

}  // namespace

Result<std::vector<CornerCandidate>> suppress_harris_nonmaxima(
    const HarrisResponse& response,
    double relative_threshold,
    int radius,
    std::size_t maximum_candidates) {
    Result<bool> validation =
        validate_response(response, "suppress_harris_nonmaxima");
    if (!validation.ok) {
        return Result<std::vector<CornerCandidate>>::failure(validation.error);
    }
    if (!std::isfinite(relative_threshold) || relative_threshold <= 0.0 ||
        relative_threshold > 1.0 || radius < 1) {
        return Result<std::vector<CornerCandidate>>::failure(
            "suppress_harris_nonmaxima: options are invalid");
    }
    const double maximum =
        *std::max_element(response.values.begin(), response.values.end());
    if (maximum <= 0.0) {
        return Result<std::vector<CornerCandidate>>::success({});
    }
    const double threshold = relative_threshold * maximum;
    std::vector<CornerCandidate> candidates;
    for (int y = 0; y < response.height; ++y) {
        for (int x = 0; x < response.width; ++x) {
            const double value = response.values[index_of(response.width, x, y)];
            if (value < threshold) {
                continue;
            }
            bool maximum_in_window = true;
            for (int dy = -radius; dy <= radius && maximum_in_window; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    const int neighbor_x = x + dx;
                    const int neighbor_y = y + dy;
                    if (neighbor_x < 0 || neighbor_y < 0 ||
                        neighbor_x >= response.width ||
                        neighbor_y >= response.height ||
                        (dx == 0 && dy == 0)) {
                        continue;
                    }
                    const double neighbor = response.values[
                        index_of(response.width, neighbor_x, neighbor_y)];
                    if (neighbor > value ||
                        (neighbor == value &&
                         (neighbor_y < y ||
                          (neighbor_y == y && neighbor_x < x)))) {
                        maximum_in_window = false;
                        break;
                    }
                }
            }
            if (maximum_in_window) {
                candidates.push_back({Point2D{static_cast<double>(x),
                                              static_cast<double>(y)},
                                      value});
            }
        }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const CornerCandidate& first, const CornerCandidate& second) {
                  if (first.response != second.response) {
                      return first.response > second.response;
                  }
                  if (first.point.y != second.point.y) {
                      return first.point.y < second.point.y;
                  }
                  return first.point.x < second.point.x;
              });
    if (maximum_candidates > 0 && candidates.size() > maximum_candidates) {
        candidates.resize(maximum_candidates);
    }
    return Result<std::vector<CornerCandidate>>::success(std::move(candidates));
}

Result<std::vector<CornerCandidate>> rank_checkerboard_candidates(
    const Image& image,
    const std::vector<CornerCandidate>& candidates,
    int sampling_radius) {
    if (sampling_radius < 1) {
        return Result<std::vector<CornerCandidate>>::failure(
            "rank_checkerboard_candidates: sampling radius must be positive");
    }
    return score_checkerboard_candidates(image, candidates, sampling_radius);
}

Result<CheckerboardDetection> fit_checkerboard_grid(
    const std::vector<CornerCandidate>& candidates,
    int rows,
    int cols,
    double minimum_spacing_pixels,
    double maximum_grid_line_error) {
    if (rows < 2 || cols < 2 || !std::isfinite(minimum_spacing_pixels) ||
        minimum_spacing_pixels <= 0.0 ||
        !std::isfinite(maximum_grid_line_error) ||
        maximum_grid_line_error <= 0.0) {
        return Result<CheckerboardDetection>::failure(
            "fit_checkerboard_grid: options are invalid");
    }
    const std::size_t expected =
        static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
    if (candidates.size() < expected) {
        return Result<CheckerboardDetection>::failure(
            "fit_checkerboard_grid: not enough corner candidates");
    }
    for (const CornerCandidate& candidate : candidates) {
        if (!finite_point(candidate.point) ||
            !std::isfinite(candidate.response)) {
            return Result<CheckerboardDetection>::failure(
                "fit_checkerboard_grid: candidate is non-finite");
        }
    }

    std::vector<CornerCandidate> ranked = candidates;
    std::stable_sort(ranked.begin(), ranked.end(),
                     [](const CornerCandidate& first,
                        const CornerCandidate& second) {
                         return first.response > second.response;
                     });
    std::vector<CornerCandidate> lattice_pool = ranked;
    if (lattice_pool.size() > 2 * expected) {
        lattice_pool.resize(2 * expected);
    }
    std::vector<CornerCandidate> selected = select_affine_lattice_subset(
        lattice_pool, rows, cols, minimum_spacing_pixels);
    if (selected.size() != expected) {
        selected = ranked;
        selected.resize(expected);
    }

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const CornerCandidate& candidate : selected) {
        centroid += Eigen::Vector2d(candidate.point.x, candidate.point.y);
    }
    centroid /= static_cast<double>(selected.size());
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const CornerCandidate& candidate : selected) {
        const Eigen::Vector2d centered =
            Eigen::Vector2d(candidate.point.x, candidate.point.y) - centroid;
        covariance += centered * centered.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(covariance);
    if (eigen_solver.info() != Eigen::Success ||
        eigen_solver.eigenvalues()(0) <=
            std::numeric_limits<double>::epsilon() *
                eigen_solver.eigenvalues()(1)) {
        return Result<CheckerboardDetection>::failure(
            "fit_checkerboard_grid: candidates do not span a 2D grid");
    }

    const Eigen::Vector2d first_axis = eigen_solver.eigenvectors().col(0);
    const Eigen::Vector2d second_axis = eigen_solver.eigenvectors().col(1);
    double best_error = std::numeric_limits<double>::infinity();
    double best_score = std::numeric_limits<double>::infinity();
    std::vector<CornerCandidate> best_order;

    // A checkerboard grid's convex hull is a quadrilateral (intermediate edge
    // points are collinear and removed). Hypothesize its cyclic orientations,
    // predict the full projective lattice, and assign every prediction to a
    // unique detected candidate. This handles perspective cases in which
    // adjacent rows overlap along a global PCA projection.
    const std::vector<Point2D> hull = convex_hull(selected);
    std::vector<std::array<Point2D, 4>> hull_quadrilaterals;
    if (hull.size() >= 4 && hull.size() <= 16) {
        for (std::size_t first = 0; first + 3 < hull.size(); ++first) {
            for (std::size_t second = first + 1;
                 second + 2 < hull.size(); ++second) {
                for (std::size_t third = second + 1;
                     third + 1 < hull.size(); ++third) {
                    for (std::size_t fourth = third + 1;
                         fourth < hull.size(); ++fourth) {
                        hull_quadrilaterals.push_back(
                            {hull[first], hull[second],
                             hull[third], hull[fourth]});
                    }
                }
            }
        }
    }
    for (const std::array<Point2D, 4>& quadrilateral :
         hull_quadrilaterals) {
        const std::vector<Point2D> model_corners = {
            {0.0, 0.0}, {static_cast<double>(cols - 1), 0.0},
            {static_cast<double>(cols - 1), static_cast<double>(rows - 1)},
            {0.0, static_cast<double>(rows - 1)}};
        for (int direction : {-1, 1}) {
            for (int start = 0; start < 4; ++start) {
                std::vector<Point2D> image_corners;
                for (int corner = 0; corner < 4; ++corner) {
                    const int hull_index =
                        (start + direction * corner + 8) % 4;
                    image_corners.push_back(
                        quadrilateral[static_cast<std::size_t>(hull_index)]);
                }
                Result<Eigen::Matrix3d> homography =
                    clean_calib::calib::estimate_homography(
                        model_corners, image_corners);
                if (!homography.ok) {
                    continue;
                }

                std::vector<CornerCandidate> ordered;
                std::vector<bool> used(selected.size(), false);
                double squared_match_error = 0.0;
                double predicted_spacing_sum = 0.0;
                std::size_t predicted_spacing_count = 0;
                std::vector<Point2D> predictions;
                predictions.reserve(expected);
                bool mapping_valid = true;
                for (int row = 0; row < rows && mapping_valid; ++row) {
                    for (int col = 0; col < cols; ++col) {
                        Result<Point2D> prediction =
                            clean_calib::calib::apply_homography(
                                homography.value,
                                {static_cast<double>(col),
                                 static_cast<double>(row)});
                        if (!prediction.ok) {
                            mapping_valid = false;
                            break;
                        }
                        predictions.push_back(prediction.value);
                        std::size_t best_index = selected.size();
                        double nearest = std::numeric_limits<double>::infinity();
                        for (std::size_t candidate = 0;
                             candidate < selected.size(); ++candidate) {
                            if (used[candidate]) {
                                continue;
                            }
                            const double distance = squared_distance(
                                prediction.value, selected[candidate].point);
                            if (distance < nearest) {
                                nearest = distance;
                                best_index = candidate;
                            }
                        }
                        if (best_index == selected.size()) {
                            mapping_valid = false;
                            break;
                        }
                        used[best_index] = true;
                        ordered.push_back(selected[best_index]);
                        squared_match_error += nearest;
                    }
                }
                if (!mapping_valid || ordered.size() != expected) {
                    continue;
                }
                for (int row = 0; row < rows; ++row) {
                    for (int col = 0; col < cols; ++col) {
                        const std::size_t index =
                            static_cast<std::size_t>(row * cols + col);
                        if (col + 1 < cols) {
                            predicted_spacing_sum += std::sqrt(squared_distance(
                                predictions[index], predictions[index + 1]));
                            ++predicted_spacing_count;
                        }
                        if (row + 1 < rows) {
                            predicted_spacing_sum += std::sqrt(squared_distance(
                                predictions[index],
                                predictions[index + static_cast<std::size_t>(cols)]));
                            ++predicted_spacing_count;
                        }
                    }
                }
                const double predicted_spacing =
                    predicted_spacing_sum /
                    static_cast<double>(predicted_spacing_count);
                const double normalized_match_error =
                    std::sqrt(squared_match_error /
                              static_cast<double>(expected)) /
                    predicted_spacing;
                if (!std::isfinite(normalized_match_error) ||
                    normalized_match_error > 0.35) {
                    continue;
                }
                canonicalize_image_order(ordered, rows, cols);
                Result<double> error = grid_geometry_error(
                    ordered, rows, cols, minimum_spacing_pixels);
                if (error.ok) {
                    const double score = error.value + normalized_match_error;
                    if (score < best_score) {
                        best_score = score;
                        best_error = error.value;
                        best_order = std::move(ordered);
                    }
                }
            }
        }
    }

    const std::array<std::pair<Eigen::Vector2d, Eigen::Vector2d>, 2> hypotheses = {
        std::make_pair(first_axis, second_axis),
        std::make_pair(second_axis, first_axis)};
    for (const auto& hypothesis : hypotheses) {
        Result<std::vector<CornerCandidate>> ordered = order_with_axes(
            selected, hypothesis.first, hypothesis.second, rows, cols);
        if (!ordered.ok) {
            continue;
        }
        Result<double> error = grid_geometry_error(
            ordered.value, rows, cols, minimum_spacing_pixels);
        if (error.ok && error.value < best_score) {
            best_score = error.value;
            best_error = error.value;
            best_order = std::move(ordered.value);
        }
    }
    if (best_order.empty() || best_error > maximum_grid_line_error) {
        return Result<CheckerboardDetection>::failure(
            "fit_checkerboard_grid: candidates do not form a consistent grid");
    }

    CheckerboardDetection detection;
    detection.candidates = candidates;
    detection.grid_line_error = best_error;
    detection.corners.reserve(best_order.size());
    for (const CornerCandidate& candidate : best_order) {
        detection.corners.push_back(candidate.point);
    }
    return Result<CheckerboardDetection>::success(std::move(detection));
}

Result<std::vector<Point2D>> refine_corners_subpixel(
    const HarrisResponse& response,
    const std::vector<Point2D>& corners) {
    Result<bool> validation =
        validate_response(response, "refine_corners_subpixel");
    if (!validation.ok) {
        return Result<std::vector<Point2D>>::failure(validation.error);
    }
    std::vector<Point2D> refined;
    refined.reserve(corners.size());
    for (const Point2D& corner : corners) {
        if (!finite_point(corner)) {
            return Result<std::vector<Point2D>>::failure(
                "refine_corners_subpixel: corner is non-finite");
        }
        const int center_x = static_cast<int>(std::lround(corner.x));
        const int center_y = static_cast<int>(std::lround(corner.y));
        if (center_x < 1 || center_y < 1 ||
            center_x + 1 >= response.width ||
            center_y + 1 >= response.height) {
            refined.push_back(corner);
            continue;
        }

        Eigen::Matrix<double, 9, 6> design;
        Eigen::Matrix<double, 9, 1> values;
        int sample = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const double x = static_cast<double>(dx);
                const double y = static_cast<double>(dy);
                design.row(sample) << x * x, x * y, y * y, x, y, 1.0;
                values(sample) = response.values[
                    index_of(response.width, center_x + dx, center_y + dy)];
                ++sample;
            }
        }
        const Eigen::Matrix<double, 6, 1> coefficients =
            design.colPivHouseholderQr().solve(values);
        const Eigen::Matrix2d hessian =
            (Eigen::Matrix2d() << 2.0 * coefficients(0), coefficients(1),
                                  coefficients(1), 2.0 * coefficients(2))
                .finished();
        const double determinant = hessian.determinant();
        if (!coefficients.array().isFinite().all() ||
            coefficients(0) >= 0.0 || determinant <= 0.0) {
            refined.push_back(corner);
            continue;
        }
        const Eigen::Vector2d offset =
            -hessian.inverse() * coefficients.segment<2>(3);
        if (!offset.array().isFinite().all() ||
            std::abs(offset.x()) > 1.0 || std::abs(offset.y()) > 1.0) {
            refined.push_back(corner);
            continue;
        }
        refined.push_back(
            {static_cast<double>(center_x) + offset.x(),
             static_cast<double>(center_y) + offset.y()});
    }
    return Result<std::vector<Point2D>>::success(std::move(refined));
}

Result<std::vector<Point2D>> refine_checkerboard_corners_subpixel(
    const Image& image,
    const std::vector<Point2D>& corners,
    int window_radius,
    int maximum_iterations,
    double convergence_tolerance) {
    if (window_radius < 2 || maximum_iterations <= 0 ||
        !std::isfinite(convergence_tolerance) ||
        convergence_tolerance <= 0.0) {
        return Result<std::vector<Point2D>>::failure(
            "refine_checkerboard_corners_subpixel: options are invalid");
    }
    Result<ImageGradients> gradient_result = compute_sobel_gradients(image);
    if (!gradient_result.ok) {
        return Result<std::vector<Point2D>>::failure(gradient_result.error);
    }
    const ImageGradients& gradients = gradient_result.value;
    std::vector<Point2D> refined;
    refined.reserve(corners.size());
    const double sigma = 0.5 * static_cast<double>(window_radius);
    const double maximum_total_shift =
        1.5 * static_cast<double>(window_radius);
    for (const Point2D& original : corners) {
        if (!finite_point(original)) {
            return Result<std::vector<Point2D>>::failure(
                "refine_checkerboard_corners_subpixel: corner is non-finite");
        }
        Eigen::Vector2d estimate(original.x, original.y);
        bool stable = true;
        for (int iteration = 0; iteration < maximum_iterations; ++iteration) {
            if (estimate.x() - window_radius < 1.0 ||
                estimate.y() - window_radius < 1.0 ||
                estimate.x() + window_radius >= gradients.width - 1 ||
                estimate.y() + window_radius >= gradients.height - 1) {
                stable = false;
                break;
            }
            Eigen::Matrix2d normal = Eigen::Matrix2d::Zero();
            Eigen::Vector2d right_hand_side = Eigen::Vector2d::Zero();
            for (int dy = -window_radius; dy <= window_radius; ++dy) {
                for (int dx = -window_radius; dx <= window_radius; ++dx) {
                    const double sample_x = estimate.x() + dx;
                    const double sample_y = estimate.y() + dy;
                    const double gradient_x = bilinear_field(
                        gradients.x, gradients.width, gradients.height,
                        sample_x, sample_y);
                    const double gradient_y = bilinear_field(
                        gradients.y, gradients.width, gradients.height,
                        sample_x, sample_y);
                    const double weight = std::exp(
                        -0.5 * static_cast<double>(dx * dx + dy * dy) /
                        (sigma * sigma));
                    const Eigen::Vector2d gradient(gradient_x, gradient_y);
                    const Eigen::Matrix2d contribution =
                        weight * gradient * gradient.transpose();
                    normal += contribution;
                    right_hand_side +=
                        contribution * Eigen::Vector2d(sample_x, sample_y);
                }
            }
            const double determinant = normal.determinant();
            if (!normal.array().isFinite().all() ||
                determinant <= std::numeric_limits<double>::epsilon() *
                                   normal.squaredNorm()) {
                stable = false;
                break;
            }
            const Eigen::Vector2d updated = normal.ldlt().solve(right_hand_side);
            if (!updated.array().isFinite().all() ||
                (updated - Eigen::Vector2d(original.x, original.y)).norm() >
                    maximum_total_shift) {
                stable = false;
                break;
            }
            const double movement = (updated - estimate).norm();
            estimate = updated;
            if (movement <= convergence_tolerance) {
                break;
            }
        }
        refined.push_back(stable ? Point2D{estimate.x(), estimate.y()} : original);
    }
    return Result<std::vector<Point2D>>::success(std::move(refined));
}

Result<CheckerboardDetection> detect_checkerboard(
    const Image& image,
    const CheckerboardDetectionOptions& options) {
    if (options.rows < 2 || options.cols < 2 ||
        options.candidate_pool_multiplier < 1) {
        return Result<CheckerboardDetection>::failure(
            "detect_checkerboard: board dimensions or candidate pool are invalid");
    }
    Result<HarrisResponse> response =
        compute_harris_response(image, options.harris);
    if (!response.ok) {
        return Result<CheckerboardDetection>::failure(response.error);
    }
    const std::size_t expected =
        static_cast<std::size_t>(options.rows) *
        static_cast<std::size_t>(options.cols);
    const std::size_t maximum_candidates =
        expected * static_cast<std::size_t>(options.candidate_pool_multiplier);
    Result<std::vector<CornerCandidate>> candidates =
        suppress_harris_nonmaxima(
            response.value, options.response_threshold_relative,
            options.nonmaximum_radius, maximum_candidates);
    if (!candidates.ok) {
        return Result<CheckerboardDetection>::failure(candidates.error);
    }
    Result<std::vector<CornerCandidate>> scored_candidates =
        rank_checkerboard_candidates(
            image, candidates.value,
            std::max(7, options.nonmaximum_radius));
    if (!scored_candidates.ok) {
        return Result<CheckerboardDetection>::failure(scored_candidates.error);
    }
    Result<CheckerboardDetection> grid = fit_checkerboard_grid(
        scored_candidates.value, options.rows, options.cols,
        options.minimum_spacing_pixels, options.maximum_grid_line_error);
    if (!grid.ok) {
        return grid;
    }
    Result<std::vector<Point2D>> refined =
        refine_checkerboard_corners_subpixel(
            image, grid.value.corners, 5, 20, 1e-3);
    if (!refined.ok) {
        return Result<CheckerboardDetection>::failure(refined.error);
    }
    grid.value.corners = std::move(refined.value);

    std::vector<CornerCandidate> refined_candidates;
    refined_candidates.reserve(grid.value.corners.size());
    for (const Point2D& corner : grid.value.corners) {
        refined_candidates.push_back({corner, 1.0});
    }
    Result<double> refined_error = grid_geometry_error(
        refined_candidates, options.rows, options.cols,
        options.minimum_spacing_pixels);
    if (!refined_error.ok ||
        refined_error.value > options.maximum_grid_line_error) {
        return Result<CheckerboardDetection>::failure(
            "detect_checkerboard: refined grid failed geometry validation");
    }
    grid.value.grid_line_error = refined_error.value;
    return grid;
}

}  // namespace clean_calib::detection
