# PROGRESS

A milestone-driven checklist. Each milestone is small enough to finish
in one short session and to verify with a test or a CLI command.
Do not skip ahead — each step builds on the previous one.

Legend: `[x]` done · `[ ]` pending · `[~]` partially done

---

## Milestone 0 — Repository skeleton
- [x] CMake configuration that builds library + CLI + tests
- [x] `clean_calib` executable runs and prints help
- [x] `clean_calib_tests` executable runs
- [x] `README.md`, `PROGRESS.md`, `CLAUDE.md` exist
- [x] Folder layout: `include/`, `src/`, `tests/`, `examples/`, `external/`

## Milestone 1 — Image I/O basics
- [x] Load image with `stb_image`
- [x] `image-info` CLI prints width/height/channels and load status
- [x] Save image with `stb_image_write`
- [x] `image-copy` CLI proves round-trip works
- [ ] Friendly error messages for missing/unreadable paths
- [ ] Support both PNG and JPEG round-trip

## Milestone 2 — Basic image utilities
- [x] `to_grayscale(const Image&)` for RGB / RGBA / already-gray inputs
- [x] CLI command to save a grayscale copy
- [x] Bounds helpers: `in_bounds(x,y)`, `clamp_to_image`
- [ ] Optional debug PPM writer for quick visual inspection

## Milestone 3 — Core geometry types
- [x] `Point2D`, `Point3D`
- [x] `CameraIntrinsics` (`fx, fy, cx, cy, skew`)
- [x] `Distortion` (`k1, k2, k3, p1, p2`)
- [x] `Pose` (`R`, `t`)
- [x] `CameraModel` bundling intrinsics + distortion
- [x] Coordinate conventions documented in headers

## Milestone 4 — Synthetic checkerboard object points
- [x] `generate_planar_board(rows, cols, square_size)` returns inner corners on Z=0
- [x] CLI `generate-board`
- [x] Tests for count and ordering
- [ ] Tests for square spacing

## Milestone 5 — Pinhole projection without distortion
- [x] World → camera transform via `Pose`
- [x] Perspective divide
- [x] Apply `CameraIntrinsics` (with skew)
- [x] Tests with known cameras (identity pose, axis-aligned offsets)

## Milestone 6 — Brown-Conrady distortion model
- [x] Radial term `k1, k2, k3`
- [x] Tangential term `p1, p2`
- [x] `project_point(world, pose, model)` end-to-end
- [x] Test: zero distortion ≡ pinhole projection

## Milestone 7 — Synthetic calibration dataset generator
- [x] Generate N camera poses around a target board
- [x] Project all corners per view
- [ ] Optional Gaussian pixel noise
- [x] Save correspondences as plain text (`.txt` / `.csv`)
- [x] Reload and verify

## Milestone 8 — Homography estimation
- [x] Hartley point normalisation
- [x] DLT 2D→2D homography
- [x] SVD solve via Eigen
- [x] Reprojection-error tests on synthetic data

## Milestone 9 — Zhang closed-form initialisation
- [x] Build the `V` matrix from per-view homographies
- [x] Solve for `b` (vec(B)) via SVD
- [x] Recover `K` from `B`
- [x] Per-view `(R, t)` from `K^{-1} H`
- [x] Project rotation to nearest valid `SO(3)` via SVD
- [x] Test on synthetic zero-distortion data: recovered `K` ≈ ground truth

## Milestone 10 — Nonlinear refinement
- [x] Reprojection residual function (intrinsics + distortion + poses)
- [x] Numeric Jacobian (central differences)
- [x] Gauss-Newton iteration
- [x] Levenberg-Marquardt with trust-region update
- [x] Refine all parameters jointly; record RMS reprojection error

## Milestone 11 — Checkerboard detection prep
- [x] Grayscale conversion (re-uses Milestone 2)
- [x] Sobel / central-difference gradients
- [x] Harris-style corner response
- [x] Save debug images of the response map

## Milestone 12 — Checkerboard detection
- [x] Non-maximum suppression of corner candidates
- [x] Grid fitting to recover row/column structure
- [x] Canonical corner ordering
- [x] Subpixel refinement (local quadratic fit)
- [x] Reject detections with bad geometry

## Milestone 13 — Full real-image calibration CLI
- [ ] `calibrate` command consuming a folder of images
- [ ] Detect → init → refine → report
- [ ] Print final RMS reprojection error
- [ ] Export camera model to a small JSON / text file

## Milestone 14 — Documentation and examples
- [ ] Walkthrough: synthetic calibration end-to-end
- [ ] Walkthrough: real-image calibration end-to-end
- [ ] Short note explaining Zhang's method intuition
- [ ] Short note explaining Brown-Conrady distortion
