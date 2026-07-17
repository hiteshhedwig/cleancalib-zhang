# Milestone 3 — Core geometry types and conventions

## Purpose

Give every later equation an unambiguous representation. Most calibration bugs
are convention mismatches disguised as algebra bugs.

## Understand before coding

- Points versus vectors and why this small project may represent both plainly.
- World, camera, normalized-image, distorted-normalized, and pixel coordinates.
- A rigid world-to-camera transform: `X_c = R X_w + t`.
- What a proper rotation requires: `R^T R = I` and `det(R) = +1`.
- The intrinsic matrix:

```text
    [ fx  skew  cx ]
K = [  0    fy  cy ]
    [  0     0   1 ]
```

- Units: world coordinates and translation share a unit; normalized image
  coordinates are dimensionless; final coordinates are pixels.

This project uses image `+y` downward and camera `+z` forward. A `Pose` maps
world points into camera coordinates; it is not a camera-to-world pose.

## Practice before implementation

1. Draw all axes and verify that camera `x × y = z` under the chosen convention.
2. For a camera center `C` and world-to-camera rotation `R`, derive
   `t = -R C`.
3. Construct a 90-degree rotation and verify orthogonality and determinant.
4. Multiply `K [x, y, 1]^T` by hand, including nonzero skew.

## Suggested implementation stages

1. Document global coordinate conventions before defining types.
2. Add small `Point2D` and `Point3D` value types.
3. Add named intrinsic and distortion fields.
4. Store the pose rotation and translation with identity defaults.
5. Bundle intrinsics and distortion into a camera model, leaving pose separate
   because it changes per view.
6. Test defaults and explicitly verify the pose direction in later projection.

## Tests

- Default points and coefficients have deterministic values.
- Default pose is identity.
- A known rotation satisfies `R^T R ≈ I` and `det(R) ≈ 1`.
- Named fields survive assignment without parameter-order confusion.

## Common mistakes

- Confusing camera position `C` with extrinsic translation `t`.
- Using a camera-to-world pose in a world-to-camera equation.
- Mixing metres and millimetres between points and translation.
- Calling pixel coordinates “normalized coordinates.”
- Storing per-view pose inside a camera model shared across views.

## Resources

- Eigen, [Getting started](https://eigen.tuxfamily.org/dox/GettingStarted.html)
  and [Matrix class tutorial](https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html).
- Eigen, [Geometry module](https://www.eigen.tuxfamily.org/dox/group__Geometry__Module.html):
  read the rotation and transform conventions, but keep this project's `Pose`
  representation explicit.
- Zhang's report, Sections 2.1–2.2, for notation, `K`, extrinsics, and the
  planar camera equation.

## Completion questions

- Does `t` equal the camera's world position? Why not?
- Which quantities carry physical units?
- Why are intrinsics shared while pose is per view?
