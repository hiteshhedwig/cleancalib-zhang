# Milestone 2 practice — indexing and grayscale

## Exercises

1. Hard-code a `2 × 2` RGB buffer containing red, green, blue, and white.
2. Write the row-major interleaved offset formula yourself.
3. Print every channel using `(x,y,c)` indexing.
4. Convert each pixel with `0.299R + 0.587G + 0.114B`.
5. Predict all four rounded grayscale values before running the program.
6. Add `in_bounds` and try every edge plus negative coordinates.
7. Compare skipping, clamping, and zero-padding one out-of-bounds lookup.

Do not include production image headers. This exercise is about reconstructing
layout and boundary policies from first principles.

## Exit criteria

You can find any channel offset without trial and error and explain why border
handling is a policy rather than part of the offset formula.
