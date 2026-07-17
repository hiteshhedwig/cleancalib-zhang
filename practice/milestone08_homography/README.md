# Milestone 8 practice — normalized DLT homography

Complete these exercises in order. Each should be a small program with
hard-coded points and printed intermediate results. Derive the expected values
on paper first.

The detailed theory and production checklist remain in
[`docs/milestone8.md`](../../docs/milestone8.md).

## Exercise 1 — Homogeneous coordinates

Start in `sandbox.cpp`.

- Represent `[x,y,1]^T` with `Eigen::Vector3d`.
- Multiply it by a known translation homography.
- Divide by the third component to recover `(u,v)`.
- Multiply the entire result or `H` by a nonzero scale and verify that the
  inhomogeneous point is unchanged.
- Try a homography that makes the third component zero. Decide what a safe
  production function should do.

You are done when you can explain `x' ~ Hx` and the meaning of `~`.

## Exercise 2 — Point normalization

- Hard-code four corners of an off-center rectangle.
- Compute the centroid with an explicit loop.
- Compute the mean Euclidean distance from the centroid.
- Construct Hartley's similarity transform `T`.
- Transform every point and print them.
- Verify numerically that their centroid is `(0,0)` and their mean distance is
  `sqrt(2)`.
- Repeat after multiplying all original coordinates by 1000.

Do not write a generic API yet. The goal is to see what normalization does.

## Exercise 3 — Build one DLT block

- Choose one source point and its known destination.
- Write the two corresponding `A` rows by hand.
- Put the same values into an Eigen matrix.
- Flatten a known `H` into `h` using your chosen coefficient order.
- Verify that both dot products in `A*h` are approximately zero.

This catches sign and reshaping mistakes before SVD is involved.

## Exercise 4 — SVD null space

- Build a small matrix with an obvious one-dimensional null space.
- Compute `JacobiSVD` with full `V`.
- Print singular values and the last column of `V`.
- Verify `A*v` is near zero.
- Modify the matrix to lose rank and observe what changes.

Only after this exercise should SVD be connected to the DLT matrix.

## Exercise 5 — Four-point DLT

- Create four non-collinear source points.
- Transform them using a known homography.
- Build the complete `8 × 9` matrix.
- Recover the last right singular vector and reshape it into `H_est`.
- Compare mappings produced by `H_est` and ground-truth `H`; do not compare
  their raw entries before handling scale.

First do this without point normalization using modest coordinates.

## Exercise 6 — Normalized DLT

- Normalize both point sets independently.
- Estimate `H_n` from normalized coordinates.
- Denormalize with `T_dst.inverse() * H_n * T_src`.
- Repeat the experiment with large offsets and very different coordinate
  scales.
- Compare reprojection error with and without normalization.

Print centroids, mean distances, singular values, estimated matrices, and
per-point errors. These intermediate values are the point of the exercise.

## Exercise 7 — Degeneracy experiments

Try each case and predict what happens before running it:

- Only three correspondences
- Four collinear points
- Duplicate correspondences
- Nearly collinear points
- One incorrect correspondence
- Small Gaussian pixel noise

Record which cases can be rejected structurally and which require examining
rank, conditioning, or reprojection error.

## Exit criteria

Move to production only when you can derive the DLT rows, explain the last
column of `V`, derive the denormalization order, compare homographies up to
scale, and describe why collinear points are degenerate.

The production implementation should then be written afresh with a clean API,
input validation, `Result` errors, and permanent tests.
