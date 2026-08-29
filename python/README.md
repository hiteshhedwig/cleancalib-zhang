# Python binding and calibration GUI

The optional Python layer exposes the real C++ detector and calibration
pipeline—there is no Python reimplementation and no NumPy dependency in the
binding itself. The local Flask GUI adds an optional comparison with OpenCV's
classic checkerboard pipeline.

## Build the binding

Use the same Python interpreter to configure CMake and run the application:

```bash
cmake -S . -B build-python \
  -DCMAKE_BUILD_TYPE=Release \
  -DCLEAN_CALIB_BUILD_PYTHON=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build build-python -j
```

`/usr/bin/python3` is the verified interpreter on this development machine.
The default Conda interpreter currently has an incompatible OpenCV/NumPy pair.
On another machine, substitute any Python interpreter whose OpenCV and NumPy
imports succeed.

If the chosen interpreter does not already provide Flask, NumPy, and OpenCV:

```bash
python3 -m pip install -r python/requirements-gui.txt
```

The C++ extension needs Python development headers. On Debian/Ubuntu these are
provided by `python3-dev`.

## Python API

Add the generated package directory to `PYTHONPATH`:

```bash
PYTHONPATH=build-python/python /usr/bin/python3 - <<'PY'
from pathlib import Path
import clean_calib

images = sorted(Path("data/opencv_left").glob("*.jpg"))
report = clean_calib.calibrate(images, rows=7, cols=6, square_size=1.0)
print(report["detected"], report["images_total"], report["rms_px"])
print(report["intrinsics"])
PY
```

The returned dictionary includes:

- dataset-level detection coverage and initial/final RMS;
- convergence and iteration count;
- `fx`, `fy`, `cx`, `cy`, and skew;
- five Brown-Conrady distortion coefficients;
- per-image detection status, homography RMS, grid score, and final RMS.

## Run the browser GUI

```bash
CLEAN_CALIB_PYTHON_PATH=build-python/python \
  /usr/bin/python3 python/gui/app.py
```

Open <http://127.0.0.1:5000>. You can select individual images, select a whole
folder in browsers that support directory upload, or drag images into the drop
zone. The app:

1. runs the native `clean-calib` binding;
2. runs OpenCV `findChessboardCorners` + `cornerSubPix` + `calibrateCamera`;
3. recalibrates on the intersection when the detectors find different views;
4. displays coverage, timings, shared-subset RMS, camera parameters, distortion,
   and per-image metrics;
5. lets you download the complete report as JSON.

Uploads are stored in a private temporary directory for one request and removed
immediately afterward. The development server binds only to `127.0.0.1`.

## Tests

With `CLEAN_CALIB_BUILD_PYTHON=ON`, `ctest` always runs the native binding test.
The full upload/API comparison test is also registered when Flask, NumPy, and
OpenCV import successfully in the selected interpreter.
