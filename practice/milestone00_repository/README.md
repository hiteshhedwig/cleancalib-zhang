# Milestone 0 practice — build-system refresher

This milestone does not need a permanent sandbox executable. Practice it once
in a temporary directory so the real repository cannot supply the answers.

## Exercise

Create a tiny C++ project containing:

```text
toy-project/
  CMakeLists.txt
  include/toy/math.h
  src/math.cpp
  app/main.cpp
  tests/test_math.cpp
```

Make `math.cpp` a static library, then link both executables to it. Enable
warnings on the library, register the test executable with CTest, and build
outside the source directory.

After it works, deliberately:

1. Remove the public include path and identify the compiler failure.
2. Remove the link dependency and identify the linker failure.
3. Make the test return a nonzero value and observe CTest.
4. Explain `PRIVATE`, `PUBLIC`, and `INTERFACE` without consulting the file.

Delete the temporary project when you can recreate its CMake file from memory.
Then compare it with the real top-level `CMakeLists.txt`.
