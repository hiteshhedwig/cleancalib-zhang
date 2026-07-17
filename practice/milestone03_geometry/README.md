# Milestone 3 practice — coordinate frames and geometry

## Exercises

1. Draw world and camera frames using the project's `+x` right, `+y` down,
   `+z` forward convention.
2. In the sandbox, construct a 90-degree rotation around one axis and verify
   `R^T R = I` and `det(R)=+1`.
3. Choose a camera center `C`, calculate `t=-RC`, and verify that transforming
   `C` gives the camera origin.
4. Transform three basis points and explain every resulting coordinate.
5. Construct the intrinsic matrix from `fx,fy,cx,cy,skew` and multiply a
   normalized homogeneous point.
6. Label the units of every quantity you print.

## Exit criteria

You can distinguish camera center from extrinsic translation and can state the
direction of `Pose` without consulting production code.
