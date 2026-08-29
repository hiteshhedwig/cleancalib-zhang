#include "clean_calib/detection/harris.h"
#include "clean_calib/image/image_io.h"
#include "test_harness.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

using clean_calib::Image;
using clean_calib::Result;
using clean_calib::detection::HarrisOptions;
using clean_calib::detection::HarrisResponse;
using clean_calib::detection::ImageGradients;
using clean_calib::detection::compute_harris_response;
using clean_calib::detection::compute_sobel_gradients;
using clean_calib::detection::harris_response_to_image;
using clean_calib::detection::save_harris_response_image;

namespace {

std::size_t index_of(const Image& image, int x, int y) {
    return static_cast<std::size_t>(y * image.width + x);
}

std::size_t index_of(const ImageGradients& gradients, int x, int y) {
    return static_cast<std::size_t>(y * gradients.width + x);
}

Image make_gray_image(int width, int height, unsigned char value = 0) {
    Image image;
    image.width = width;
    image.height = height;
    image.channels = 1;
    image.data.assign(static_cast<std::size_t>(width * height), value);
    return image;
}

CC_TEST(harris_sobel_constant_image_has_zero_gradients) {
    const Image image = make_gray_image(7, 6, 91);
    Result<ImageGradients> result = compute_sobel_gradients(image);
    CC_EXPECT_TRUE(result.ok);
    CC_EXPECT_EQ(result.value.width, image.width);
    CC_EXPECT_EQ(result.value.height, image.height);
    CC_EXPECT_EQ(result.value.x.size(), image.pixel_count());
    CC_EXPECT_EQ(result.value.y.size(), image.pixel_count());
    for (std::size_t i = 0; i < result.value.x.size(); ++i) {
        CC_EXPECT_NEAR(result.value.x[i], 0.0, 1e-12);
        CC_EXPECT_NEAR(result.value.y[i], 0.0, 1e-12);
    }
}

CC_TEST(harris_sobel_recovers_horizontal_and_vertical_ramp_slopes) {
    Image horizontal = make_gray_image(7, 7);
    Image vertical = make_gray_image(7, 7);
    for (int y = 0; y < 7; ++y) {
        for (int x = 0; x < 7; ++x) {
            horizontal.data[index_of(horizontal, x, y)] =
                static_cast<unsigned char>(10 * x + 20);
            vertical.data[index_of(vertical, x, y)] =
                static_cast<unsigned char>(12 * y + 15);
        }
    }
    Result<ImageGradients> horizontal_result =
        compute_sobel_gradients(horizontal);
    Result<ImageGradients> vertical_result = compute_sobel_gradients(vertical);
    CC_EXPECT_TRUE(horizontal_result.ok);
    CC_EXPECT_TRUE(vertical_result.ok);
    const std::size_t center = index_of(horizontal_result.value, 3, 3);
    CC_EXPECT_NEAR(horizontal_result.value.x[center], 10.0, 1e-12);
    CC_EXPECT_NEAR(horizontal_result.value.y[center], 0.0, 1e-12);
    CC_EXPECT_NEAR(vertical_result.value.x[center], 0.0, 1e-12);
    CC_EXPECT_NEAR(vertical_result.value.y[center], 12.0, 1e-12);
}

CC_TEST(harris_sobel_reuses_rgb_grayscale_conversion) {
    Image image;
    image.width = 7;
    image.height = 7;
    image.channels = 3;
    image.data.resize(static_cast<std::size_t>(7 * 7 * 3));
    for (int y = 0; y < 7; ++y) {
        for (int x = 0; x < 7; ++x) {
            const unsigned char value = static_cast<unsigned char>(8 * x + 10);
            const std::size_t index =
                static_cast<std::size_t>((y * 7 + x) * 3);
            image.data[index] = value;
            image.data[index + 1] = value;
            image.data[index + 2] = value;
        }
    }
    Result<ImageGradients> gradients = compute_sobel_gradients(image);
    CC_EXPECT_TRUE(gradients.ok);
    const std::size_t center = index_of(gradients.value, 3, 3);
    CC_EXPECT_NEAR(gradients.value.x[center], 8.0, 1e-12);
    CC_EXPECT_NEAR(gradients.value.y[center], 0.0, 1e-12);
}

CC_TEST(harris_flat_image_has_zero_response) {
    const Image image = make_gray_image(11, 10, 128);
    Result<HarrisResponse> result = compute_harris_response(image);
    CC_EXPECT_TRUE(result.ok);
    CC_EXPECT_EQ(result.value.values.size(), image.pixel_count());
    for (double value : result.value.values) {
        CC_EXPECT_NEAR(value, 0.0, 1e-12);
    }
}

CC_TEST(harris_step_corner_has_strong_positive_local_response) {
    Image image = make_gray_image(17, 17, 0);
    for (int y = 8; y < image.height; ++y) {
        for (int x = 8; x < image.width; ++x) {
            image.data[index_of(image, x, y)] = 255;
        }
    }
    HarrisOptions options;
    options.window_radius = 2;
    options.sensitivity = 0.04;
    Result<HarrisResponse> result = compute_harris_response(image, options);
    CC_EXPECT_TRUE(result.ok);

    const auto maximum = std::max_element(
        result.value.values.begin(), result.value.values.end());
    CC_EXPECT_TRUE(maximum != result.value.values.end());
    CC_EXPECT_TRUE(*maximum > 0.0);
    const std::size_t maximum_index =
        static_cast<std::size_t>(maximum - result.value.values.begin());
    const int maximum_x = static_cast<int>(maximum_index % image.width);
    const int maximum_y = static_cast<int>(maximum_index / image.width);
    CC_EXPECT_TRUE(std::abs(maximum_x - 8) <= 2);
    CC_EXPECT_TRUE(std::abs(maximum_y - 8) <= 2);

    // A pure horizontal edge has a negative Harris score, while the junction
    // of horizontal and vertical edges is positive.
    const double edge = result.value.values[
        static_cast<std::size_t>(12 * image.width + 8)];
    CC_EXPECT_TRUE(*maximum > edge);

    for (int x = 0; x < image.width; ++x) {
        CC_EXPECT_NEAR(result.value.values[static_cast<std::size_t>(x)],
                       0.0, 1e-12);
    }
}

CC_TEST(harris_debug_image_clamps_negative_and_normalizes_positive_values) {
    HarrisResponse response;
    response.width = 2;
    response.height = 2;
    response.values = {-3.0, 0.0, 2.0, 4.0};
    Result<Image> image = harris_response_to_image(response);
    CC_EXPECT_TRUE(image.ok);
    CC_EXPECT_EQ(image.value.width, 2);
    CC_EXPECT_EQ(image.value.height, 2);
    CC_EXPECT_EQ(image.value.channels, 1);
    CC_EXPECT_EQ(static_cast<int>(image.value.data[0]), 0);
    CC_EXPECT_EQ(static_cast<int>(image.value.data[1]), 0);
    CC_EXPECT_EQ(static_cast<int>(image.value.data[2]), 128);
    CC_EXPECT_EQ(static_cast<int>(image.value.data[3]), 255);
}

CC_TEST(harris_debug_response_can_be_saved_and_reloaded) {
    const std::string path = "/tmp/clean_calib_harris_response.png";
    std::remove(path.c_str());
    HarrisResponse response;
    response.width = 3;
    response.height = 2;
    response.values = {0.0, 1.0, 2.0, 3.0, -1.0, 4.0};
    Result<bool> saved = save_harris_response_image(path, response);
    CC_EXPECT_TRUE(saved.ok);
    Result<Image> loaded = clean_calib::image::load(path);
    CC_EXPECT_TRUE(loaded.ok);
    CC_EXPECT_EQ(loaded.value.width, response.width);
    CC_EXPECT_EQ(loaded.value.height, response.height);
    CC_EXPECT_EQ(loaded.value.channels, 1);
    CC_EXPECT_EQ(static_cast<int>(loaded.value.data.back()), 255);
    std::remove(path.c_str());
}

CC_TEST(harris_rejects_malformed_images_options_and_responses) {
    Image too_small = make_gray_image(2, 2);
    CC_EXPECT_TRUE(!compute_sobel_gradients(too_small).ok);

    Image malformed = make_gray_image(5, 5);
    malformed.data.pop_back();
    CC_EXPECT_TRUE(!compute_sobel_gradients(malformed).ok);

    Image unsupported = make_gray_image(5, 5);
    unsupported.channels = 2;
    unsupported.data.resize(50);
    CC_EXPECT_TRUE(!compute_sobel_gradients(unsupported).ok);

    HarrisOptions bad_options;
    bad_options.window_radius = 0;
    CC_EXPECT_TRUE(!compute_harris_response(
        make_gray_image(7, 7), bad_options).ok);

    HarrisResponse malformed_response;
    malformed_response.width = 2;
    malformed_response.height = 2;
    malformed_response.values = {1.0};
    CC_EXPECT_TRUE(!harris_response_to_image(malformed_response).ok);

    malformed_response.values.resize(4, 1.0);
    malformed_response.values[2] =
        std::numeric_limits<double>::infinity();
    CC_EXPECT_TRUE(!harris_response_to_image(malformed_response).ok);
}

}  // namespace

void register_harris_tests() {
    using clean_calib::test::registry;
    registry().push_back({"harris.sobel_constant_image",
                          harris_sobel_constant_image_has_zero_gradients});
    registry().push_back({"harris.sobel_linear_ramps",
                          harris_sobel_recovers_horizontal_and_vertical_ramp_slopes});
    registry().push_back({"harris.sobel_rgb_grayscale_conversion",
                          harris_sobel_reuses_rgb_grayscale_conversion});
    registry().push_back({"harris.flat_image_zero_response",
                          harris_flat_image_has_zero_response});
    registry().push_back({"harris.step_corner_positive_response",
                          harris_step_corner_has_strong_positive_local_response});
    registry().push_back({"harris.debug_image_normalization",
                          harris_debug_image_clamps_negative_and_normalizes_positive_values});
    registry().push_back({"harris.debug_image_save_reload",
                          harris_debug_response_can_be_saved_and_reloaded});
    registry().push_back({"harris.rejects_invalid_inputs",
                          harris_rejects_malformed_images_options_and_responses});
}
