# Instructions for future Claude Code sessions

This file is the contract for any AI assistant working on this
project. Read it before making changes.

## Project values (in priority order)

1. **Readability** beats cleverness. If a hand-written 30-line loop is
   easier to follow than a 5-line Eigen one-liner, keep the loop.
2. **Modularity**. Each module must be understandable in isolation.
3. **Testability**. Every math module gets at least one test that runs
   on synthetic data with a known ground-truth answer.
4. **Educational value**. Comments explain *why* and *what the math
   means*, not what the C++ does.

## Hard rules

- **No OpenCV, no Ceres, no Sophus, no Boost.** Allowed dependencies
  are only Eigen 3.4 and the two stb headers.
- Do **not** add a heavyweight test framework. The custom
  `tests/test_main.cpp` harness is enough.
- Keep functions small. If a function exceeds ~40 lines, split it.
- Prefer free functions over classes. Use plain structs for data.
- Keep image detection code separate from calibration math. The
  homography solver, Zhang init, and refinement must be testable
  without ever loading an image.

## Workflow rules

- **Do not implement future milestones unless explicitly asked.** The
  current milestone is whatever is the first unchecked item in
  `PROGRESS.md`.
- After completing a milestone, **update `PROGRESS.md`** in the same
  change.
- When adding a new math module, add at least one synthetic test in
  the same change.
- When changing a public type in `include/clean_calib/core/`, search
  for callers and update them all in the same change.

## Style rules

- C++17. No designated initialisers, no concepts, no `<format>`.
- Header guards: `#pragma once`.
- Namespaces: everything lives in `clean_calib`. Submodules use
  nested namespaces (`clean_calib::image`, `clean_calib::synthetic`).
- Eigen types: prefer `Eigen::Matrix3d`, `Eigen::Vector3d`,
  `Eigen::VectorXd`. Use `double` everywhere unless there is a
  measured reason to use `float`.
- File coordinate conventions go at the **top of every relevant
  header** in a comment block. Image origin is top-left, x → right,
  y → down. World coordinates are right-handed with Z up unless a
  specific function says otherwise.

## What to avoid

- Generic "abstractions" like `ICamera`, `ISolver`, factory classes.
  Just write functions.
- Hiding linear algebra behind opaque method names. If the code
  computes `b = (V^T V)^-1 ...`, say so in a comment.
- Premature optimisation. Clarity first; profile later.
- Silent failures. Return a `Result<T>` or a clearly-named status,
  do not return sentinel values.

## Build & test commands

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
./build/clean_calib_tests
```

If a test fails, fix the test or the code in the *same* change — do
not commit a red bar.
