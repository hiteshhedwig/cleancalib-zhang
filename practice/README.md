# Practice workspace

This directory is a mathematical playground for small, targeted programs
written before a production milestone. Practice code is allowed to be narrow,
temporary, repetitive, and imperfect. Its purpose is to make the underlying
idea familiar before production concerns are introduced.

## Relationship to production code

```text
paper derivation
      ↓
small practice programs
      ↓
explain the result without code
      ↓
design production API and tests
      ↓
implement in include/, src/, and tests/
```

Practice programs should not call the corresponding algorithm from the
`clean_calib` library. That would test the production function rather than
practice implementing the idea. Eigen may be used for basic matrix operations
and decompositions.

It is fine to copy tiny input values into a practice program. Avoid turning
this directory into a second library or maintaining reusable abstractions here.

## Build

Practice is disabled by default, so an unfinished exercise cannot break the
normal library build.

```bash
cmake -S . -B /tmp/clean-calib-practice \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCLEAN_CALIB_BUILD_PRACTICE=ON
cmake --build /tmp/clean-calib-practice -j
```

Run the milestone 8 sandbox:

```bash
/tmp/clean-calib-practice/practice/practice_m08_sandbox
```

## How to add exercises

Use one small executable per concept when preserving earlier experiments is
valuable. Add it to `practice/CMakeLists.txt` with:

```cmake
add_clean_calib_practice(
  practice_m08_example
  milestone08_homography/example.cpp
)
```

If the experiment is truly disposable, reuse that milestone's `sandbox.cpp`.
Do not add practice executables to CTest: production tests belong under
`tests/`, while practice programs are inspected interactively.

## Rules of thumb

- Use tiny inputs whose expected result can be calculated by hand.
- Print intermediate matrices and invariants, not only a final answer.
- Change one idea at a time.
- Deliberately create a failing or degenerate input and predict the result.
- Keep a short note explaining what was learned.
- Do not copy a toy implementation directly into production. Re-design it
  around validation, naming, module boundaries, and tests.
- A practice program is complete when you can explain why it works without
  looking at it.

## Milestones

- [Milestone 0 — build-system refresher](milestone00_repository/README.md)
- [Milestone 1 — image I/O](milestone01_image_io/README.md)
- [Milestone 2 — image utilities](milestone02_image_utils/README.md)
- [Milestone 3 — geometry conventions](milestone03_geometry/README.md)
- [Milestone 4 — checkerboard model](milestone04_checkerboard/README.md)
- [Milestone 5 — pinhole projection](milestone05_pinhole/README.md)
- [Milestone 6 — distortion](milestone06_distortion/README.md)
- [Milestone 7 — synthetic data](milestone07_synthetic_data/README.md)
- [Milestone 8 — homography](milestone08_homography/README.md)

Add a new milestone directory when its study guide identifies concepts worth
isolating. Not every milestone needs practice code.
