# Milestone 0 — Repository skeleton

## Purpose

Create a project in which mathematical code can be built and tested without a
CLI or image dataset. This is architecture work, but it directly affects how
easy later numerical failures are to isolate.

## Understand before coding

- The difference between a library target and an executable target.
- Public headers versus private implementation files.
- Include paths, link dependencies, and transitive dependencies.
- Configure, build, and test as separate phases.
- Why generated build artifacts should live outside source control.

The dependency direction should be:

```text
CLI ───────┐
tests ─────┼──> clean_calib library ──> Eigen + stb
examples ──┘
```

The library must not depend on the CLI or test harness.

## Practice before implementation

1. Make a tiny static library containing `double square(double)` and link it
   into two executables.
2. Break an include path intentionally and explain the compiler error.
3. Remove a link dependency intentionally and explain the linker error.
4. Register one executable with CTest and run it through `ctest`.

## Design questions

- Which headers form the public API?
- Which dependencies should consumers inherit?
- Can calibration math be tested without loading an image?
- Can a future detector depend on image utilities without making calibration
  math depend on the detector?

## Suggested implementation stages

1. Create the `include/`, `src/`, and `tests/` layout.
2. Build an empty `clean_calib` library.
3. Link a minimal CLI to it.
4. Link a minimal test runner to it.
5. Add warnings for first-party code.
6. Register tests through CTest.
7. Verify both Debug and Release configurations.

## Tests and checks

- A clean out-of-tree configure succeeds.
- Library, CLI, and tests build independently.
- `clean_calib --help` succeeds.
- A failing unit test gives a nonzero process exit code.
- No generated build files appear as untracked source files.

## Common mistakes

- Putting all algorithms in `main.cpp`.
- Making tests exercise the CLI instead of the library API.
- Globally adding include directories or compiler flags.
- Committing `CMakeCache.txt`, object files, and executables.
- Fetching a large dependency for a very small need.

## Resources

- CMake, [official tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html):
  complete Step 1's executable and library exercises, then read the testing
  step when adding CTest.
- CMake, [`add_library`](https://cmake.org/cmake/help/latest/command/add_library.html),
  [`target_include_directories`](https://cmake.org/cmake/help/latest/command/target_include_directories.html),
  and [`target_link_libraries`](https://cmake.org/cmake/help/latest/command/target_link_libraries.html):
  focus on `PUBLIC`, `PRIVATE`, and `INTERFACE` propagation.

## Completion questions

- Why is Eigen a `PUBLIC` dependency of this library?
- Why should `src/app/main.cpp` contain orchestration rather than algorithms?
- What is the difference between a compiler error and linker error?
