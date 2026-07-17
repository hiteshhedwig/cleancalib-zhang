# Milestone 7 — Synthetic calibration datasets

## Purpose

Generate exact object-to-image correspondences from known camera parameters
and poses. Synthetic data is the laboratory for milestones 8–10: it separates
algorithm errors from corner-detection errors and provides ground truth.

## Understand before coding

- A calibration dataset contains shared object points and per-view image
  points in identical order.
- Intrinsics are shared across views; extrinsics are per view.
- A useful Zhang dataset needs varied board orientations, not merely many
  translations of a fronto-parallel board.
- Visibility has several levels: positive depth, inside image bounds, and not
  occluded. The current generator guarantees projection but does not model a
  complete renderer.
- Deterministic zero-noise data should pass before Gaussian pixel noise is
  introduced.
- Serialization precision matters when round-tripping `double` ground truth.

## Dataset invariant

For every view `j` and point index `i`:

```text
object_points[i]  <──corresponds to──>  views[j].image_points[i]
```

Never sort or filter one side without applying exactly the same operation to
the other.

## Practice before implementation

1. Choose a camera and identity rotation with `t_z > 0`; project a `2 × 2`
   board by hand.
2. Construct a rotation around the board's `x` or `y` axis and predict which
   rows or columns compress in the image.
3. Explain why five identical poses provide no more independent calibration
   information than one pose.
4. Add a fixed sample of Gaussian noise by hand and distinguish ground truth
   from observations.

## Suggested implementation stages

1. Project an arbitrary vector of object points for one pose.
2. Generate the checkerboard once and reuse its ordering for every view.
3. Accept explicit poses initially so tests remain simple and deterministic.
4. Fail with the view and point context when projection fails.
5. Serialize counts, pose, and correspondences with 17-digit precision.
6. Parse defensively and validate matching counts.
7. Later add a seeded pose sampler and optional Gaussian image noise as
   separate policies, not hidden behavior.

## Tests

- Correct board and view counts.
- All per-view image counts equal object-point count.
- Principal-point projection for the board origin.
- Translation and nontrivial rotation affect projections as predicted.
- A behind-camera view fails with context.
- Save/load compares every rotation, translation, object point, and image
  point—not merely selected entries.
- Malformed headers, counts, truncated views, and unexpected trailing data have
  a documented policy.
- Fixed random seed reproduces noise exactly when noise is added.

## View-design guidance for later calibration

A good synthetic set should include tilts about different axes, varying board
positions and distances, complete visibility, and realistic image coverage.
Avoid only fronto-parallel views, nearly identical homographies, tiny boards in
one image region, or points behind the camera. Zhang's report discusses
degenerate configurations and viewpoint sensitivity.

## Common mistakes

- Regenerating object points independently for each view and losing ordering.
- Mixing the camera model used to generate data with the model being estimated.
- Adding noise before exact recovery is demonstrated.
- Calling a function a pose generator when it only accepts supplied poses.
- Testing calibration only on the same special fronto-parallel configuration.

## Resources

- Zhang's report, Sections 4 and 5.1: degenerate configurations and computer
  simulations.
- C++, [`<random>`](https://en.cppreference.com/w/cpp/header/random): use a
  locally owned engine with an explicit seed when optional noise is added.
- C++, [`std::setprecision`](https://en.cppreference.com/w/cpp/io/manip/setprecision):
  understand why `max_digits10`/17 significant digits preserve `double`
  round trips.

## Completion questions

- Which quantities are shared and which are per view?
- Why should zero-noise recovery be solved first?
- What makes a set of poses informative for Zhang calibration?
