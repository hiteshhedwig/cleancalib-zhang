# Milestone 6 practice — Brown–Conrady distortion

## Exercises

1. Implement the forward distortion equations for one normalized point.
2. Print `r²`, `r⁴`, `r⁶`, radial multiplier, tangential contributions, and
   final distorted coordinates.
3. Test zero coefficients and predict the result.
4. Test radial-only points `(1,0)`, `(0,1)`, and `(1,1)`.
5. Verify that points with equal radius share the same radial multiplier.
6. Test tangential-only `(1,1)` and calculate it on paper.
7. Change the sign of `k1` and sketch the qualitative motion of points.
8. Apply intrinsics only after distortion and explain why.

Do not call `distort_normalized`. Keep every term visible until you can derive
it without the README.

## Exit criteria

You can distinguish radial from tangential terms, state the coordinate space in
which they operate, and explain forward distortion versus undistortion.
