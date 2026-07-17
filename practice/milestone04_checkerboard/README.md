# Milestone 4 practice — planar checkerboard model

## Exercises

1. Generate a `2 × 3` inner-corner grid with spacing `0.025` using plain loops.
2. Predict and print every index and `(x,y,z)` value.
3. Assert that horizontal and vertical neighbor distances equal the spacing.
4. Move the board-frame origin to the grid center and regenerate the points.
5. Explain which later calibration quantities would change under that new
   origin and which would not.
6. Repeat with millimetres and predict how recovered translation would scale.

Do not call `generate_planar_board`; recreate the small loop so ordering becomes
automatic again.

## Exit criteria

You can distinguish square count from inner-corner count and explain why object
and image arrays must retain identical ordering.
