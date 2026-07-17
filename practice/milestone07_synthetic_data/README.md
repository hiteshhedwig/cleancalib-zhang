# Milestone 7 practice — synthetic calibration data

This exercise connects the earlier pieces without using the production board
or projection functions.

## Exercises

1. Generate a tiny `2 × 3` planar point grid locally.
2. Choose known intrinsics and two explicit poses.
3. Project every point using a small local pinhole function.
4. Store one image-point vector per pose and preserve index correspondence.
5. Print rows as `view, index, X, Y, Z, u, v`.
6. Tilt the second view rather than using translation only.
7. Verify that every view contains exactly one image point per object point.
8. Create one invalid pose and report which view/point fails.
9. Add fixed noise only after the exact dataset is correct.

This is the one refresher where repeating small portions of milestones 4 and 5
is intentional: composition is the concept being practiced.

## Experiments

- Use several identical poses and explain why they add no information.
- Use only fronto-parallel poses and identify the missing viewpoint diversity.
- Tilt around different axes and observe the projected grid.
- Change board units and inspect pixels versus translation units.

## Exit criteria

You can explain shared intrinsics, per-view extrinsics, correspondence ordering,
ground truth, and why exact synthetic recovery precedes noisy experiments.
