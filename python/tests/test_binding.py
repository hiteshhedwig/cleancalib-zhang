import os
import unittest
from pathlib import Path

import clean_calib


SOURCE_ROOT = Path(os.environ["CLEAN_CALIB_SOURCE_DIR"])


class BindingTests(unittest.TestCase):
    def test_rejects_invalid_board(self):
        with self.assertRaises(ValueError):
            clean_calib.calibrate(["unused.jpg"], 1, 6)

    def test_real_dataset_calibrates(self):
        paths = sorted((SOURCE_ROOT / "data" / "opencv_left").glob("*.jpg"))
        result = clean_calib.calibrate(paths, rows=7, cols=6, square_size=1.0)
        self.assertEqual(result["images_total"], 13)
        self.assertEqual(result["detected"], 13)
        self.assertTrue(result["converged"])
        self.assertLess(result["rms_px"], 0.25)
        self.assertEqual(len(result["images"]), 13)
        self.assertTrue(all(image["found"] for image in result["images"]))


if __name__ == "__main__":
    unittest.main()
