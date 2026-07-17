# Visible demo roadmap

Every substantial milestone should finish with something observable: an image,
overlay, or compact report that makes the mathematics concrete. These demos are
learning checkpoints, not production applications.

`PROGRESS.md` remains the implementation checklist. This file separately tracks
whether each milestone has a satisfying visible result.

## Quality bar

A demo is complete when it:

- runs with one documented command;
- uses deterministic, representative inputs;
- saves one or two clear artifacts under `examples/output/`;
- prints the important parameters and output path;
- has an obvious visual interpretation or color legend;
- exercises the public library API rather than duplicating the algorithm; and
- is small enough to understand in one sitting.

A decent demo should show more than a raw coordinate dump. It does not need a
window, interactive controls, animation, fonts, or a polished CLI.

## Scope limits

Keep each demo to roughly one short source file and at most a few hours of work.
Reuse a tiny drawing helper for pixels, lines, circles, and image saving when
several demos need it. Do not add OpenCV, a plotting framework, GUI toolkit, or
font renderer.

Hard-coded representative parameters are acceptable. Robust argument parsing,
arbitrary input support, and exhaustive validation belong in the library or
final CLI, not in a milestone demo.

Use this output convention:

```text
examples/output/m02_grayscale.png
examples/output/m05_projected_board.png
examples/output/m10_refinement_before_after.png
```

Generated outputs are ignored by Git. A particularly useful small reference
artifact may be committed deliberately, but that should be exceptional.

## Build and generate milestones 1–7

```bash
cmake -S . -B /tmp/clean-calib-demos \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCLEAN_CALIB_BUILD_DEMOS=ON

cmake --build /tmp/clean-calib-demos --target run_clean_calib_demos -j
```

The second command builds and runs every completed demo below. Open the PNGs in
`examples/output/`; read the same-named `.txt` reports for conventions and
parameters.

## Tracking legend

- `[x]` satisfying demo exists
- `[~]` the underlying command exists, but the curated demo is incomplete
- `[ ]` not created
- `[—]` intentionally no visual demo

## Foundation milestones

### Milestone 0 — Repository skeleton `[—]`

No image is needed. The tangible checkpoint is a successful configure/build,
the CLI help text, and a green test summary. Do not invent a visualization for
build-system work.

### Milestone 1 — Image I/O `[x]`

**Artifact:** a copied PNG beside a short metadata report.

Use `examples/images/leena.png`. Load it, save it as PNG, reload it, and print
dimensions, channels, and whether lossless pixels match.

**Done when:** the input and output can be opened visually and the PNG
round-trip comparison reports equality.

### Milestone 2 — Grayscale and image utilities `[x]`

**Artifact:** `m02_grayscale.png`, plus the original image for comparison.

Use `examples/images/leena.png` and save a side-by-side original/grayscale
canvas as well as the standalone grayscale image.

**Visual question:** which colored regions become brighter or darker, and does
that agree with the RGB luma weights?

### Milestone 3 — Geometry conventions `[x]` (optional)

**Artifact:** `m03_coordinate_frames.png` showing world axes, camera axes, a
camera center, and one transformed point.

SVG can be emitted as simple text without a graphics dependency. This demo is
optional because the milestone is primarily about definitions. Create it only
if coordinate-frame confusion persists.

**Visual question:** where does a world point land after the world-to-camera
transform, and why is `t` not the camera center?

### Milestone 4 — Checkerboard object points `[x]`

**Artifact:** `m04_board_points.png`.

Render generated inner corners on a blank image. Connect horizontal neighbors
in one color and vertical neighbors in another. Highlight the origin and final
row-major point. Print the index and coordinates of those highlighted points
to the terminal; drawing text into the image is unnecessary.

**Visual question:** do row/column orientation, spacing, origin, and ordering
match the documented board frame?

### Milestone 5 — Pinhole projection `[x]`

**Artifact:** `m05_projected_board.png` containing three panels or overlaid
colors for fronto-parallel, translated, and tilted poses.

Project the milestone 4 board onto an empty `640 × 480` image. Draw corners and
grid edges, clipping only while rasterizing—not inside projection math. Print
each pose and the pixel bounds of its projection.

**Visual question:** do translation, distance, and tilt change the projected
grid in the direction predicted on paper?

### Milestone 6 — Brown–Conrady distortion `[x]`

**Artifact:** `m06_distortion_grid.png` with ideal and distorted grids side by
side, or overlaid in different colors.

Create a regular grid of normalized points, distort each point, convert both
sets to pixels, and connect corresponding grid lines. Use coefficients large
enough to be visible but label them as exaggerated.

**Visual question:** which displacement is radial, which is tangential, and why
does displacement grow away from the center?

Applying distortion to a photograph is a later bonus. A correct image warp
needs inverse mapping and interpolation; it should not inflate this milestone.

### Milestone 7 — Synthetic calibration dataset `[x]`

**Artifact:** `m07_synthetic_views.png`, a contact sheet of four to six views.

Render each generated view as a small panel using the same intrinsics and
different poses. Include tilts about different axes and print the pose beside
each panel in a companion text report if image text is inconvenient.

**Visual question:** is there genuine viewpoint diversity, is the full board
visible, and are all point correspondences ordered consistently?

## Calibration milestones

### Milestone 8 — Homography estimation `[ ]`

**Artifact:** `m08_homography_overlay.png`.

Estimate a homography from synthetic planar correspondences. Draw observed
points in green and homography-reprojected points in magenta; exact agreement
should appear nearly as one mark. Add a second noisy example with short error
segments and print RMS error.

**Visual question:** does the estimated projective map reproduce the entire
plane, including points not used in the four-point minimal solve?

### Milestone 9 — Zhang closed-form initialization `[ ]`

**Artifacts:** `m09_intrinsics_report.txt` and
`m09_recovered_projection.png`.

Print ground-truth and recovered `K`, their element-wise differences, and pose
rotation/translation errors. Project the board using both parameter sets and
overlay their points.

**Visual question:** do recovered intrinsics and extrinsics explain the
synthetic observations before nonlinear refinement?

### Milestone 10 — Nonlinear refinement `[ ]`

**Artifact:** `m10_refinement_before_after.png` and a short iteration table.

Draw initial reprojections and refined reprojections against observations in
separate panels. Print iteration number, objective/RMS, damping value, and
accepted/rejected step status.

**Visual question:** do residual segments shrink, and does the reported
objective decrease for accepted steps?

## Detection and integration milestones

### Milestone 11 — Detection preparation `[ ]`

**Artifacts:** grayscale, gradient magnitude, and colorized Harris-response
images for one real checkerboard photograph.

Normalize diagnostic values only for display; keep the computation itself in
its natural numeric range.

**Visual question:** are strong responses concentrated near real checkerboard
corners rather than flat regions or only along edges?

### Milestone 12 — Checkerboard detection `[ ]`

**Artifact:** `m12_detected_corners.png`.

Overlay candidate points faintly, accepted grid corners strongly, and connect
them in canonical row-major order. Highlight the first corner so orientation is
obvious. Optionally draw integer versus subpixel positions at high zoom.

**Visual question:** are all inner corners present exactly once and ordered in
the same convention as object points?

### Milestone 13 — Real-image calibration `[ ]`

**Artifacts:** final parameter report, per-view reprojection overlays, and one
original/undistorted image pair.

Run on a public multi-view chessboard dataset from one fixed camera. Print `K`,
distortion, total RMS, per-view RMS, accepted/rejected images, and board
configuration.

Validate in three layers:

1. Our solver and OpenCV use the same corner correspondences.
2. Our detector is compared with OpenCV's detected corners.
3. The complete pipelines are compared end to end.

Use matching distortion models and constraints before comparing parameters.
Treat OpenCV as a reference, not physical ground truth.

**Visual question:** are reprojected corners aligned, and do straight lines look
straighter in the undistorted result without introducing severe artifacts?

### Milestone 14 — Final walkthrough `[ ]`

**Artifact:** a short Markdown gallery linking the best outputs from milestones
4–13 and explaining what each stage contributed.

This is assembly and explanation, not a new visualization system. Reuse the
existing outputs rather than rebuilding them.

## Demo completion template

Copy this block under a milestone when implementing its demo:

```markdown
- [ ] One-command build/run documented
- [ ] Deterministic input and parameters
- [ ] Output saved under examples/output/
- [ ] Visual legend or interpretation documented
- [ ] Key numeric values printed
- [ ] Public library API exercised
- [ ] Demo remains small and dependency-free
```

## Recommended next visible work

Milestones 1–7 are complete. The next visible checkpoint should be added only
after milestone 8's homography implementation and tests:

```text
M08 observed points → estimated homography → reprojected overlay → RMS report
```
