// Minimal test harness — no external framework.
//
// Each test file defines a void register_*_tests() function which
// registers tests via REGISTER_TEST(...). main() runs them all and
// reports pass/fail counts.

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "test_harness.h"

namespace clean_calib::test {

std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}

}  // namespace clean_calib::test

void register_checkerboard_tests();
void register_image_io_tests();
void register_camera_types_tests();
void register_image_utils_tests();

int main() {
    register_checkerboard_tests();
    register_image_io_tests();
    register_camera_types_tests();
    register_image_utils_tests();

    auto& tests = clean_calib::test::registry();
    int passed = 0;
    int failed = 0;
    for (const auto& t : tests) {
        std::string err;
        bool ok = false;
        try {
            t.fn(err);
            ok = err.empty();
        } catch (const std::exception& e) {
            err = std::string("exception: ") + e.what();
            ok = false;
        } catch (...) {
            err = "unknown exception";
            ok = false;
        }
        if (ok) {
            std::printf("  PASS  %s\n", t.name);
            ++passed;
        } else {
            std::printf("  FAIL  %s — %s\n", t.name, err.c_str());
            ++failed;
        }
    }
    std::printf("\n%d passed, %d failed (out of %zu)\n",
                passed, failed, tests.size());
    return failed == 0 ? 0 : 1;
}
