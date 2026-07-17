# Milestone 8 — Homography estimation with normalized DLT

## Purpose

Estimate a `3 × 3` projective transformation from planar model coordinates to
image pixels for each calibration view. These homographies are the input to
Zhang's closed-form intrinsic initialization.

This is the first inverse estimation milestone: observations are given and the
transformation that produced them must be recovered.

## Read before touching the implementation

Read in this order:

1. Zhang's report, Section 2.2, to see why a planar target induces
   `H = K [r1 r2 t]`.
2. Zhang's Appendix A for its homography estimation formulation.
3. Hartley's [In Defence of the 8-point Algorithm](https://users.cecs.anu.edu.au/~hartley/Papers/fundamental/ICCV-final/fundamental.pdf),
   especially the normalization argument. The paper estimates a fundamental
   matrix, but centering and scaling points before a homogeneous linear solve
   is exactly the lesson needed here.
4. Eigen's [SVD module](https://eigen.tuxfamily.org/dox/group__SVD__Module.html)
   and [`JacobiSVD`](https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html).

Do not start with an implementation tutorial. First derive the two DLT rows
from the homogeneous equation yourself.

## Prerequisites

You should be able to explain:

- Homogeneous 2D points: `(x,y) ↔ [x,y,1]^T`.
- Equality up to nonzero scale.
- A homography's eight degrees of freedom despite nine entries.
- Cross product constraint `x' × Hx = 0`.
- Null spaces, singular values, and why the last right singular vector solves
  `Ah = 0` under `||h||=1`.
- Why coordinate magnitude affects conditioning.
- Similarity transforms and denormalization.

## Derive the DLT system

Let source point `(x,y)` map to destination `(u,v)`, and let the rows of `H`
be `h1^T`, `h2^T`, `h3^T`. In inhomogeneous form:

```text
u = (h1^T [x y 1]^T) / (h3^T [x y 1]^T)
v = (h2^T [x y 1]^T) / (h3^T [x y 1]^T)
```

Rearranging gives two linear equations in the nine entries of `H`. One valid
sign convention is:

```text
[-x -y -1   0  0  0   u*x u*y u]
[ 0  0  0  -x -y -1   v*x v*y v]
```

Changing the sign of a row does not change the null space. Be consistent with
the coefficient-vector reshaping order.

With `N` correspondences, `A` has dimensions `2N × 9`. At least four
non-collinear correspondences are needed for a general homography.

## Hartley normalization

Normalize source and destination sets independently:

1. Compute their centroid `(cx,cy)`.
2. Compute the mean Euclidean distance `d` from that centroid.
3. Choose scale `s = sqrt(2)/d`.
4. Construct

```text
    [ s  0  -s*cx ]
T = [ 0  s  -s*cy ]
    [ 0  0     1  ]
```

The transformed points have centroid near zero and mean distance `sqrt(2)`.
Estimate `H_n` between normalized point sets. If

```text
x_n  = T_src x
x'_n = T_dst x'
x'_n ~ H_n x_n
```

then denormalize with:

```text
H = inverse(T_dst) * H_n * T_src
```

Derive this substitution yourself rather than memorizing it.

## Eigen knowledge required

- Use `Eigen::MatrixXd A(2*N, 9)` because the row count is dynamic.
- Compute the right singular vectors. The null-space solution is the column of
  `V` associated with the smallest singular value.
- For this small dense problem, `JacobiSVD` prioritizes accuracy and clarity.
- Request the form of `V` that actually contains all nine right singular
  vectors. Understand the difference between thin and full factors before
  choosing options.
- Never form `A.transpose() * A` just to use an eigenvalue solver: it squares
  the condition number and discards the direct SVD lesson.

## API design guidance

Keep the solver generic:

```text
source Point2D correspondences + destination Point2D correspondences
    -> Result<Matrix3d>
```

It should not depend on images, checkerboards, camera types, or
`SyntheticCalibrationDataset`. For this project, board `Point3D` values can be
converted to planar `(X,Y)` by the caller after verifying `Z=0`.

Useful separable responsibilities are:

- normalize a point set and return both normalized points and `T`;
- build/solve normalized DLT;
- apply a homography with safe homogeneous division;
- compute reprojection errors.

Not every helper must be public. Keep helpers private unless another module or
an independently valuable test needs them.

## Practice before implementation

Use the dedicated
[`practice/milestone08_homography`](../practice/milestone08_homography/README.md)
workspace for the exercise sequence below. Keep each experiment small and
print its intermediate matrices.

1. Apply a translation homography to four points by hand.
2. Apply a scale-plus-translation homography and recover inhomogeneous points.
3. Derive both DLT rows from `u(h3^T x)=h1^T x` and
   `v(h3^T x)=h2^T x`.
4. Normalize a four-point square manually and verify its centroid and mean
   distance.
5. Substitute the normalization equations to derive the denormalization order.
6. Explain why `H` and `-3H` represent the same mapping.

## Implementation stages

1. Implement safe homography application and test known matrices.
2. Implement point normalization and test its invariants.
3. Validate equal sizes, at least four pairs, finite values, and non-degenerate
   spread.
4. Assemble `A` and check its dimensions and selected rows in tests.
5. Solve `Ah=0` with SVD and reshape `h` consistently.
6. Denormalize and choose a scale convention. Dividing by `H(2,2)` is readable
   only when that entry is safely away from zero; Frobenius normalization is a
   more general alternative.
7. Measure per-point Euclidean reprojection error and RMS error.
8. Test exact synthetic data, then different coordinate scales, then small
   noise.

## Required tests

- Identity homography.
- Translation, scaling, affine shear, and a genuine projective homography.
- More than four correspondences.
- Coordinates with very different magnitudes to demonstrate normalization.
- Recovery judged by transformed points, not raw matrix equality alone.
- Scale/sign ambiguity handled correctly.
- Source/destination size mismatch.
- Fewer than four pairs.
- Duplicate, collinear, or nearly degenerate points.
- A mapping whose homogeneous denominator is zero or near zero.
- No NaN or infinity can pass a numerical assertion.

For exact synthetic data, reprojection error should be close to floating-point
precision. With noise, require a sensible error range rather than exact matrix
coefficients.

## Common mistakes

- Skipping normalization because a small example happens to work.
- Normalizing only image points instead of both sets.
- Reversing `T_src` and `T_dst` during denormalization.
- Taking a column of `U` instead of the last column of `V`.
- Reshaping Eigen's vector according to storage layout rather than the explicit
  coefficient order used to build `A`.
- Solving by fixing `h33=1`, which fails for valid homographies where that
  coefficient is zero or tiny.
- Comparing raw `H` entries without accounting for scale.
- Reporting algebraic `||Ah||` as geometric reprojection error.

## Ready-to-code checklist

Before adding `homography.cpp`, you should be able to answer:

- Why does one correspondence contribute only two independent equations?
- Why are four collinear points insufficient?
- What does the smallest singular value tell you?
- What invariants should normalized points satisfy?
- Why is the denormalization order `T_dst^-1 H_n T_src`?
- How will your tests compare two homographies up to scale?

If any answer is unclear, resolve it on paper first. That will save far more
time than debugging a plausible-looking `3 × 3` matrix later.
