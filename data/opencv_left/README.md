# OpenCV checkerboard calibration images

The 13 `left*.jpg` files are the public sample calibration images from the
OpenCV repository's `samples/data` directory (branch `4.x`). The sequence runs
from `left01.jpg` through `left14.jpg`; upstream does not contain `left10.jpg`.

- Source: https://github.com/opencv/opencv/tree/4.x/samples/data
- Tutorial: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- Pattern: 6 columns by 7 rows of inner corners
- Square size: one arbitrary unit (the physical size is not published)

They are retained here as a reproducible real-image detector and calibration
audit set. OpenCV is distributed under the Apache License 2.0.
