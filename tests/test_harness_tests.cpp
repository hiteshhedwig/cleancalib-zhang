#include "test_harness.h"

#include <limits>

using clean_calib::test::values_are_near;

namespace {

CC_TEST(harness_values_are_near_accepts_values_within_tolerance) {
    CC_EXPECT_TRUE(values_are_near(1.0, 1.0001, 0.001));
}

CC_TEST(harness_values_are_near_rejects_values_outside_tolerance) {
    CC_EXPECT_TRUE(!values_are_near(1.0, 1.1, 0.001));
}

CC_TEST(harness_values_are_near_rejects_non_finite_values) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();

    CC_EXPECT_TRUE(!values_are_near(nan, 1.0, 0.001));
    CC_EXPECT_TRUE(!values_are_near(1.0, nan, 0.001));
    CC_EXPECT_TRUE(!values_are_near(infinity, infinity, 0.001));
    CC_EXPECT_TRUE(!values_are_near(1.0, 1.0, infinity));
    CC_EXPECT_TRUE(!values_are_near(1.0, 1.0, -0.001));
}

}  // namespace

void register_harness_tests() {
    using clean_calib::test::registry;

    registry().push_back({"harness.values_are_near_accepts_values_within_tolerance",
                          harness_values_are_near_accepts_values_within_tolerance});
    registry().push_back({"harness.values_are_near_rejects_values_outside_tolerance",
                          harness_values_are_near_rejects_values_outside_tolerance});
    registry().push_back({"harness.values_are_near_rejects_non_finite_values",
                          harness_values_are_near_rejects_non_finite_values});
}
