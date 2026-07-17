# clean-calib study guide

This directory is the learning path for implementing Zhang calibration from
scratch. `PROGRESS.md` records completion; these documents explain what to
understand before writing each milestone.

## How to use the guides

For a milestone:

1. Read its prerequisites and write down the coordinate conventions yourself.
2. Work through the paper-and-pencil exercises before opening the implementation.
3. Read only the specified parts of the resources.
4. Design the smallest API and tests that expose the mathematics.
5. Implement one stage at a time and run its tests before continuing.
6. Return to the guide's completion questions and answer them without code.

Do not treat the equations as recipes. Track the dimensions, coordinate frame,
units, and scale ambiguity of every quantity.

After completing the tests for a milestone, use the
[visible demo roadmap](../examples/demos/README.md) to produce a small tangible
artifact without turning the demo into a second project.

## Curriculum

| Milestone | Subject | Main learning outcome |
|---|---|---|
| [0](milestone0.md) | Repository skeleton | Separate library, CLI, and tests |
| [1](milestone1.md) | Image I/O | Own pixels safely across a C API boundary |
| [2](milestone2.md) | Image utilities | Understand layout, indexing, and grayscale |
| [3](milestone3.md) | Geometry types | Fix conventions before writing equations |
| [4](milestone4.md) | Checkerboard model | Create ordered planar object points |
| [5](milestone5.md) | Pinhole projection | Map world points to ideal pixels |
| [6](milestone6.md) | Lens distortion | Apply Brown–Conrady in normalized space |
| [7](milestone7.md) | Synthetic datasets | Produce ground-truth calibration problems |
| [8](milestone8.md) | Homography estimation | Recover a plane-to-image projective map |

Milestone 8 is the current major milestone. Milestones 0–7 describe and audit
the foundation already present in the repository.

## Core references

- Zhengyou Zhang, [A Flexible New Technique for Camera Calibration](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/).
  The longer technical report linked on that page is the central reference.
- [Eigen documentation](https://eigen.tuxfamily.org/dox/), especially its
  matrix, geometry, and decomposition tutorials.
- Richard Hartley, [In Defence of the 8-point Algorithm](https://users.cecs.anu.edu.au/~hartley/Papers/fundamental/ICCV-final/fundamental.pdf).
  Its point-normalization argument applies directly to normalized DLT.

The guides deliberately avoid OpenCV calibration functions: they are useful
later for validation, but they would hide the algorithms this project exists
to learn.
