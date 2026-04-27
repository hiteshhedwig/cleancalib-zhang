#pragma once

#include <cmath>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

namespace clean_calib::test {

struct TestCase {
    const char* name;
    std::function<void(std::string&)> fn;
};

std::vector<TestCase>& registry();

struct AutoRegister {
    AutoRegister(const char* name, std::function<void(std::string&)> fn) {
        registry().push_back({name, std::move(fn)});
    }
};

}  // namespace clean_calib::test

// Helpers used inside test bodies. Tests take a `std::string& _err`
// parameter; setting it non-empty marks the test failed.
#define CC_FAIL(msg) do { _err = (msg); return; } while (0)

#define CC_EXPECT_TRUE(cond) do { \
    if (!(cond)) { \
        std::ostringstream _oss; _oss << "expected true: " #cond; \
        _err = _oss.str(); return; \
    } \
} while (0)

#define CC_EXPECT_EQ(a, b) do { \
    auto _va = (a); auto _vb = (b); \
    if (!(_va == _vb)) { \
        std::ostringstream _oss; \
        _oss << "expected " #a " == " #b " (got " << _va << " vs " << _vb << ")"; \
        _err = _oss.str(); return; \
    } \
} while (0)

#define CC_EXPECT_NEAR(a, b, tol) do { \
    double _va = static_cast<double>(a); \
    double _vb = static_cast<double>(b); \
    if (std::abs(_va - _vb) > (tol)) { \
        std::ostringstream _oss; \
        _oss << "expected |" #a " - " #b "| <= " << (tol) \
             << " (got " << _va << " vs " << _vb << ")"; \
        _err = _oss.str(); return; \
    } \
} while (0)

#define CC_TEST(name) static void name(std::string& _err)
