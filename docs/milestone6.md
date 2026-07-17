# Milestone 6 — Brown–Conrady distortion

## Purpose

Extend ideal pinhole projection with a transparent lens-distortion model.
Distortion is evaluated in normalized camera coordinates before applying `K`.

## Model

For normalized `(x,y)` and `r²=x²+y²`:

```text
radial = 1 + k1*r² + k2*r⁴ + k3*r⁶

x_d = x*radial + 2*p1*x*y + p2*(r² + 2*x²)
y_d = y*radial + p1*(r² + 2*y²) + 2*p2*x*y
```

Then apply intrinsics to `(x_d,y_d)`.

## Understand before coding

- Ideal versus distorted normalized coordinates.
- Radial symmetry around the distortion center, assumed here to coincide with
  the principal point through normalized coordinates.
- Barrel and pincushion behavior and the sign of low-order radial terms.
- Tangential/decentering distortion caused by imperfect lens alignment.
- Forward distortion versus iterative undistortion. This milestone implements
  only the forward model needed for projection and optimization.
- Coefficient conventions differ across libraries; names and equations are
  more reliable than vector position.

## Paper-and-pencil practice

1. Show that `(0,0)` is unchanged by every coefficient.
2. Evaluate radial-only distortion at `(1,0)` and `(0,1)`.
3. Evaluate tangential-only distortion at `(1,1)`.
4. Sketch how negative `k1` changes points as radius grows.
5. Explain why applying the same polynomial directly to pixel coordinates
   would make coefficients depend on resolution and focal length.

## Suggested implementation stages

1. Compute `r²`, `r⁴`, and `r⁶` without `pow`.
2. Implement the radial multiplier alone and test symmetry.
3. Add tangential terms and test them independently.
4. Compose world transform, division, distortion, and intrinsics.
5. Preserve a zero-distortion path mathematically equivalent to pinhole
   projection; a special code branch is unnecessary.

## Tests

- Zero coefficients preserve normalized points exactly within tolerance.
- Origin remains fixed.
- Radial-only cases on axes and off-axis.
- Tangential-only case with hand-computed output.
- Radial symmetry for points with equal radius.
- Complete zero-distortion projection matches `project_pinhole`.
- Large/non-finite coefficients are not allowed to silently pass numerical
  assertions as NaNs.

## Common mistakes

- Distorting after converting to pixels.
- Using `r` where the polynomial requires `r²`.
- Swapping the `p1` and `p2` formulas.
- Assuming every source orders coefficients as `k1,k2,k3,p1,p2`.
- Implementing inverse undistortion when the optimizer needs forward
  projection.

## Resources

- Zhang's report, Section 3.3, for its radial-distortion treatment and where
  distortion enters calibration.
- Duane Brown, [Lens Distortion for Close-Range Photogrammetry](https://www.asprs.org/wp-content/uploads/pers/1986journal/jan/1986_jan_51-58.pdf):
  historical and mathematical background; the notation is broader than this
  project's compact model.
- Keep this project's equations in `camera.h` authoritative so implementation,
  tests, and future optimization use one convention.

## Completion questions

- Why is the distortion function applied before `K`?
- Which terms are radial and which are decentering/tangential?
- Why does calibration need forward distortion even if the eventual user wants
  an undistorted image?
