# Milestone 5 — Pinhole projection without distortion

## Purpose

Implement the forward model that predicts an ideal pixel from a known 3D
point, camera pose, and intrinsic matrix. Every later calibration stage is
judged by how well its parameters reproduce this mapping.

## Pipeline

```text
world point X_w
    │  X_c = R X_w + t
camera point (X_c, Y_c, Z_c)
    │  x = X_c/Z_c, y = Y_c/Z_c
normalized point (x, y)
    │  u = fx*x + skew*y + cx
pixel point (u, v), where v = fy*y + cy
```

## Understand before coding

- World-to-camera versus camera-to-world transformations.
- Perspective division and why points with `Z_c <= 0` are not projectable.
- Normalized coordinates as coordinates on the plane `Z_c = 1`.
- The intrinsic matrix and homogeneous scale.
- Why focal length in the projection is expressed in pixels.
- How the image `+y`-down convention is embodied by the camera frame.

In homogeneous form:

```text
s p = K [R | t] P
```

This compact equation is useful, but the implementation should preserve the
individual conceptual stages so each can be tested.

## Paper-and-pencil practice

1. Project `(0,0,2)` with identity rotation, zero translation, and principal
   point `(320,240)`.
2. Project `(1,2,4)` and perform the perspective divide explicitly.
3. Translate a camera-frame point along `x` and predict the pixel direction.
4. Use nonzero skew and identify which pixel component changes.
5. Derive `P_c = R(P_w-C)` and show that `t=-RC`.

## Suggested implementation stages

1. `world_to_camera`: perform only the rigid transform.
2. `camera_to_normalized`: validate depth and divide by `Z_c`.
3. `normalized_to_pixel`: apply only intrinsics.
4. `project_pinhole`: compose the stages and preserve failures.
5. Keep image dimensions out of projection; a valid projection may still lie
   outside a particular sensor.

## Tests

- Identity pose leaves coordinates unchanged before division.
- Translation and a genuine rotation are tested separately.
- Known perspective divide with `Z != 1`.
- Principal point and off-axis projections.
- Nonzero skew.
- Zero and negative depth fail.
- Non-finite points and camera parameters follow a deliberate policy.

## Common mistakes

- Applying translation before rotation as `R(X+t)`.
- Treating `t` as the camera center.
- Dividing pixel coordinates instead of camera coordinates.
- Applying intrinsics before distortion; distortion belongs in normalized
  space and is introduced in milestone 6.
- Rejecting pixels outside the image in this geometry-only module.

## Resources

- Zhang's technical report, Sections 2.1–2.2: camera notation and the planar
  homography equation.
- Eigen, [matrix arithmetic](https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html):
  matrix-vector multiplication, transpose, and coefficient access.
- Eigen, [Geometry module](https://www.eigen.tuxfamily.org/dox/group__Geometry__Module.html):
  useful background for rotations, while this project keeps the equation
  explicit.

## Completion questions

- What frame is a point in immediately before perspective division?
- Why is normalized image space dimensionless?
- Why does a point behind the camera return failure instead of a pixel?
