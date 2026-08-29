import os
import sys
import unittest
from pathlib import Path


SOURCE_ROOT = Path(os.environ["CLEAN_CALIB_SOURCE_DIR"])
sys.path.insert(0, str(SOURCE_ROOT / "python" / "gui"))
import app as gui_app  # noqa: E402


class GuiIntegrationTests(unittest.TestCase):
    def test_page_and_real_comparison(self):
        client = gui_app.app.test_client()
        self.assertEqual(client.get("/").status_code, 200)
        health = client.get("/api/health").get_json()
        self.assertTrue(health["clean_calib"])
        self.assertTrue(health["opencv"]["available"])

        handles = []
        try:
            uploads = []
            for path in sorted((SOURCE_ROOT / "data" / "opencv_left").glob("*.jpg")):
                handle = path.open("rb")
                handles.append(handle)
                uploads.append((handle, path.name))
            response = client.post(
                "/api/calibrate",
                data={
                    "rows": "7",
                    "cols": "6",
                    "square_size": "1.0",
                    "images": uploads,
                },
                content_type="multipart/form-data",
            )
        finally:
            for handle in handles:
                handle.close()

        self.assertEqual(response.status_code, 200, response.get_data(as_text=True))
        report = response.get_json()
        self.assertEqual(report["clean_calib"]["detected"], 13)
        self.assertEqual(report["opencv"]["detected"], 11)
        self.assertEqual(report["comparison"]["shared_images"], 11)
        self.assertLess(report["comparison"]["clean_calib_rms_px"], 0.25)
        self.assertLess(report["comparison"]["opencv_rms_px"], 0.25)


if __name__ == "__main__":
    unittest.main()
