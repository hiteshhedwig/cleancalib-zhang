#include "clean_calib/image/image_utils.h"
#include "test_harness.h"

#include <cstddef>

using clean_calib::Image;
using clean_calib::in_bounds;
using clean_calib::clamp_to_image;
using clean_calib::pixel_offset;
using clean_calib::to_grayscale;

namespace {

CC_TEST(image_in_bounds_accepts_valid_coordinates) {
    Image image;
    image.width = 4;
    image.height = 3;
    image.channels = 1;
    image.data.resize(4 * 3);

    CC_EXPECT_TRUE(in_bounds(image, 0, 0));
    CC_EXPECT_TRUE(in_bounds(image, 3, 2));

    CC_EXPECT_TRUE(!in_bounds(image, -1, 0));
    CC_EXPECT_TRUE(!in_bounds(image, 0, -1));
    CC_EXPECT_TRUE(!in_bounds(image, 4, 0));
    CC_EXPECT_TRUE(!in_bounds(image, 0, 3));
}

CC_TEST(image_clamp_to_image_clamps_coordinates) {
    Image image;
    image.width = 4;
    image.height = 3;
    image.channels = 1;
    image.data.resize(4 * 3);

    auto a = clamp_to_image(image, -1, 1);
    CC_EXPECT_EQ(a.first, 0);
    CC_EXPECT_EQ(a.second, 1);

    auto b = clamp_to_image(image, 10, 1);
    CC_EXPECT_EQ(b.first, 3);
    CC_EXPECT_EQ(b.second, 1);

    auto c = clamp_to_image(image, 2, -5);
    CC_EXPECT_EQ(c.first, 2);
    CC_EXPECT_EQ(c.second, 0);

    auto d = clamp_to_image(image, 2, 99);
    CC_EXPECT_EQ(d.first, 2);
    CC_EXPECT_EQ(d.second, 2);
}

CC_TEST(image_pixel_offset_is_row_major_interleaved) {
    Image image;
    image.width = 4;
    image.height = 3;
    image.channels = 3;
    image.data.resize(4 * 3 * 3);

    CC_EXPECT_EQ(pixel_offset(image, 0, 0, 0), static_cast<std::size_t>(0));
    CC_EXPECT_EQ(pixel_offset(image, 0, 0, 1), static_cast<std::size_t>(1));
    CC_EXPECT_EQ(pixel_offset(image, 1, 0, 0), static_cast<std::size_t>(3));
    CC_EXPECT_EQ(pixel_offset(image, 0, 1, 0), static_cast<std::size_t>(12));
    CC_EXPECT_EQ(pixel_offset(image, 2, 1, 1), static_cast<std::size_t>(19));
}

CC_TEST(image_grayscale_preserves_existing_gray_image) {
    Image image;
    image.width = 2;
    image.height = 1;
    image.channels = 1;
    image.data = {10, 200};

    Image gray = to_grayscale(image);

    CC_EXPECT_EQ(gray.width, 2);
    CC_EXPECT_EQ(gray.height, 1);
    CC_EXPECT_EQ(gray.channels, 1);
    CC_EXPECT_EQ(gray.data.size(), static_cast<std::size_t>(2));
    CC_EXPECT_EQ(static_cast<int>(gray.data[0]), 10);
    CC_EXPECT_EQ(static_cast<int>(gray.data[1]), 200);
}

CC_TEST(image_grayscale_converts_rgb_image) {
    Image image;
    image.width = 4;
    image.height = 1;
    image.channels = 3;

    image.data = {
        255, 0, 0,       // red   -> 76
        0, 255, 0,       // green -> 150
        0, 0, 255,       // blue  -> 29
        255, 255, 255    // white -> 255
    };

    Image gray = to_grayscale(image);

    CC_EXPECT_EQ(gray.width, 4);
    CC_EXPECT_EQ(gray.height, 1);
    CC_EXPECT_EQ(gray.channels, 1);
    CC_EXPECT_EQ(gray.data.size(), static_cast<std::size_t>(4));

    CC_EXPECT_EQ(static_cast<int>(gray.data[0]), 76);
    CC_EXPECT_EQ(static_cast<int>(gray.data[1]), 150);
    CC_EXPECT_EQ(static_cast<int>(gray.data[2]), 29);
    CC_EXPECT_EQ(static_cast<int>(gray.data[3]), 255);
}

CC_TEST(image_grayscale_converts_rgba_and_ignores_alpha) {
    Image image;
    image.width = 1;
    image.height = 1;
    image.channels = 4;
    image.data = {
        255, 0, 0, 123
    };

    Image gray = to_grayscale(image);

    CC_EXPECT_EQ(gray.width, 1);
    CC_EXPECT_EQ(gray.height, 1);
    CC_EXPECT_EQ(gray.channels, 1);
    CC_EXPECT_EQ(gray.data.size(), static_cast<std::size_t>(1));
    CC_EXPECT_EQ(static_cast<int>(gray.data[0]), 76);
}

CC_TEST(image_grayscale_rejects_unsupported_channel_count) {
    Image image;
    image.width = 1;
    image.height = 1;
    image.channels = 2;
    image.data = {10, 20};

    Image gray = to_grayscale(image);

    CC_EXPECT_EQ(gray.width, 0);
    CC_EXPECT_EQ(gray.height, 0);
    CC_EXPECT_EQ(gray.channels, 0);
    CC_EXPECT_TRUE(gray.data.empty());
}

}  // namespace

void register_image_utils_tests() {
    using clean_calib::test::registry;

    registry().push_back({"image_utils.in_bounds_accepts_valid_coordinates",
                          image_in_bounds_accepts_valid_coordinates});

    registry().push_back({"image_utils.clamp_to_image_clamps_coordinates",
                          image_clamp_to_image_clamps_coordinates});

    registry().push_back({"image_utils.pixel_offset_is_row_major_interleaved",
                          image_pixel_offset_is_row_major_interleaved});

    registry().push_back({"image_utils.grayscale_preserves_existing_gray_image",
                          image_grayscale_preserves_existing_gray_image});

    registry().push_back({"image_utils.grayscale_converts_rgb_image",
                          image_grayscale_converts_rgb_image});

    registry().push_back({"image_utils.grayscale_converts_rgba_and_ignores_alpha",
                          image_grayscale_converts_rgba_and_ignores_alpha});

    registry().push_back({"image_utils.grayscale_rejects_unsupported_channel_count",
                          image_grayscale_rejects_unsupported_channel_count});
}