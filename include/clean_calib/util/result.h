#pragma once

#include <string>
#include <utility>

namespace clean_calib {

// Tiny Result<T> for functions that can fail with a human message.
//
//   Result<Image> r = load_image("foo.png");
//   if (!r.ok) { std::cerr << r.error << "\n"; return 1; }
//   const Image& img = r.value;
//
// Intentionally minimal: no monadic combinators, no exceptions.
template <typename T>
struct Result {
    bool ok = false;
    T value{};
    std::string error;

    static Result success(T v) {
        Result r;
        r.ok = true;
        r.value = std::move(v);
        return r;
    }
    static Result failure(std::string msg) {
        Result r;
        r.ok = false;
        r.error = std::move(msg);
        return r;
    }
};

}  // namespace clean_calib
