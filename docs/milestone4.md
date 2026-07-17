# Milestone 4 — Synthetic checkerboard object points

## Purpose

Represent the known calibration target as ordered 3D points. Calibration only
works when each detected image point is matched to the correct object point.

## Understand before coding

- The distinction between checkerboard squares and inner corners.
- A planar target frame with every `Z = 0`.
- Row-major ordering and correspondence invariants.
- Square size as the metric scale of recovered translations.
- Why changing the board-frame origin changes extrinsics but not intrinsics.

For `rows × cols` inner corners with spacing `s`:

```text
index(i, j) = i * cols + j
P(i, j) = (j*s, i*s, 0)
```

## Practice before implementation

1. List every point for a `2 × 3` inner-corner grid.
2. Draw the ordering on a checkerboard and trace the vector indices.
3. Move the board origin to its center and predict what calibration quantities
   would change.
4. Compare passing square size in metres and millimetres.

## Suggested implementation stages

1. Choose whether API rows/columns mean squares or inner corners.
2. Document the choice unambiguously.
3. Validate positive dimensions and finite positive spacing.
4. Reserve the exact result size safely.
5. Generate points in one obvious nested loop.
6. Expose a CLI printer only as a way to inspect the library result.

## Tests

- Count is `rows * cols`.
- First point is the chosen origin.
- Adjacent columns differ by `(s, 0, 0)`.
- Adjacent rows differ by `(0, s, 0)`.
- Every `Z` is zero.
- Last point and row-major order are correct.
- Invalid and non-finite spacing is rejected.

## Common mistakes

- Passing square count while the API expects inner-corner count.
- Swapping rows and columns.
- Generating the outside square corners instead of detectable inner corners.
- Losing the same ordering between object and image points.
- Assuming square size affects recovered focal length in pixels; it primarily
  fixes the scale of translation.

## Resources

- Zhang's report, Sections 1 and 2.2: understand why the target is planar and
  why its model frame can set `Z = 0`.
- Eigen is not needed for this milestone. Keeping generation in plain loops
  makes the correspondence order visible.

## Completion questions

- Why is corner ordering more important than the choice of board origin?
- What does square size determine in the recovered calibration?
- Why does planarity reduce the mapping to a homography?
