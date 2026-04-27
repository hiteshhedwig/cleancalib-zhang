#include <cstdio>
#include <string>

#include "clean_calib/image/image_io.h"
#include "test_harness.h"

using clean_calib::Image;

namespace {

Image make_solid(int w, int h, int c, unsigned char value) {
    Image img;
    img.width = w; img.height = h; img.channels = c;
    img.data.assign(static_cast<std::size_t>(w * h * c), value);
    return img;
}

CC_TEST(image_load_missing_path_fails_gracefully) {
    auto r = clean_calib::image::load("/nonexistent/path/clean_calib_does_not_exist.png");
    CC_EXPECT_TRUE(!r.ok);
    CC_EXPECT_TRUE(!r.error.empty());
}

CC_TEST(image_save_empty_image_fails) {
    Image img;
    auto r = clean_calib::image::save("/tmp/clean_calib_should_not_be_written.png", img);
    CC_EXPECT_TRUE(!r.ok);
}

CC_TEST(image_save_unsupported_extension_fails) {
    Image img = make_solid(2, 2, 3, 255);
    auto r = clean_calib::image::save("/tmp/clean_calib_test.xyz", img);
    CC_EXPECT_TRUE(!r.ok);
}

CC_TEST(image_roundtrip_png) {
    const std::string path = "/tmp/clean_calib_roundtrip.png";
    Image src = make_solid(4, 3, 3, 200);
    src.data[0] = 10;  // tiny variation so the file is not entirely uniform
    auto saved = clean_calib::image::save(path, src);
    if (!saved.ok) CC_FAIL("save failed: " + saved.error);
    auto loaded = clean_calib::image::load(path);
    if (!loaded.ok) CC_FAIL("load failed: " + loaded.error);
    CC_EXPECT_EQ(loaded.value.width, src.width);
    CC_EXPECT_EQ(loaded.value.height, src.height);
    CC_EXPECT_EQ(loaded.value.channels, src.channels);
    std::remove(path.c_str());
}

}  // namespace

void register_image_io_tests() {
    using clean_calib::test::registry;
    registry().push_back({"image_io.load_missing_path_fails",
                          image_load_missing_path_fails_gracefully});
    registry().push_back({"image_io.save_empty_image_fails",
                          image_save_empty_image_fails});
    registry().push_back({"image_io.save_unsupported_extension_fails",
                          image_save_unsupported_extension_fails});
    registry().push_back({"image_io.roundtrip_png", image_roundtrip_png});
}
