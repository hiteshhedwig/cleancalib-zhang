#!/usr/bin/env python3
"""Generate dependency-free SVG plots from the checked-in benchmark CSVs."""

import csv
import pathlib


ROOT = pathlib.Path(__file__).resolve().parent
RESULTS = ROOT / "results"
PLOTS = ROOT / "plots"
BLUE = "#2563eb"
ORANGE = "#ea580c"
INK = "#172033"
MUTED = "#64748b"
GRID = "#dbe3ef"
BACKGROUND = "#ffffff"


def esc(text):
    return (str(text).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))


def text(x, y, value, size=14, anchor="start", weight=400, fill=INK):
    return (f'<text x="{x}" y="{y}" font-family="system-ui, sans-serif" '
            f'font-size="{size}" font-weight="{weight}" text-anchor="{anchor}" '
            f'fill="{fill}">{esc(value)}</text>')


def write_svg(name, width, height, elements):
    PLOTS.mkdir(parents=True, exist_ok=True)
    document = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}" role="img">',
        f'<rect width="{width}" height="{height}" fill="{BACKGROUND}" rx="12"/>',
        *elements,
        "</svg>",
    ]
    (PLOTS / name).write_text("\n".join(document) + "\n", encoding="utf-8")


def summary_plot(summary):
    own = {row["implementation"]: row for row in summary
           if row["subset"] == "own_detected"}
    common = {row["implementation"]: row for row in summary
              if row["subset"] == "common_11"}
    clean_name = "clean-calib"
    opencv_name = "OpenCV 4.8.0 classic"
    panels = [
        ("Detection success", "% of 13 images",
         [float(own[clean_name]["detection_rate_percent"]),
          float(own[opencv_name]["detection_rate_percent"])], 100.0, "%"),
        ("Calibration accuracy", "RMS on the same 11 images (lower is better)",
         [float(common[clean_name]["calibration_rms_px"]),
          float(common[opencv_name]["calibration_rms_px"])], 0.22, " px"),
    ]
    elements = [text(40, 42, "Real-image benchmark summary", 23, weight=700)]
    colors = [BLUE, ORANGE]
    labels = ["clean-calib", "OpenCV classic"]
    for panel_index, (title, subtitle, values, maximum, suffix) in enumerate(panels):
        x0 = 40 + panel_index * 450
        y0 = 82
        elements += [text(x0, y0, title, 18, weight=650),
                     text(x0, y0 + 22, subtitle, 12, fill=MUTED)]
        chart_y = y0 + 52
        for i, (label, value, color) in enumerate(zip(labels, values, colors)):
            y = chart_y + i * 82
            elements.append(text(x0, y + 15, label, 13))
            elements.append(
                f'<rect x="{x0}" y="{y + 27}" width="360" height="25" '
                f'fill="#eef2f7" rx="5"/>')
            width = 360 * value / maximum
            elements.append(
                f'<rect x="{x0}" y="{y + 27}" width="{width:.2f}" height="25" '
                f'fill="{color}" rx="5"/>')
            shown = f"{value:.1f}{suffix}" if suffix == "%" else f"{value:.3f}{suffix}"
            elements.append(text(x0 + 370, y + 46, shown, 14, weight=650))
    elements += [
        f'<circle cx="40" cy="318" r="6" fill="{BLUE}"/>',
        text(54, 323, "clean-calib", 12, fill=MUTED),
        f'<circle cx="150" cy="318" r="6" fill="{ORANGE}"/>',
        text(164, 323, "OpenCV 4.8.0 findChessboardCorners", 12, fill=MUTED),
    ]
    write_svg("benchmark_summary.svg", 940, 345, elements)


def per_view_plot(rows):
    width, height = 1020, 480
    left, right, top, bottom = 70, 28, 70, 92
    chart_width = width - left - right
    chart_height = height - top - bottom
    maximum = 0.30
    elements = [
        text(36, 38, "Per-view calibration RMS on the shared 11-image subset",
             22, weight=700),
        text(36, 59, "Lower is better; both methods calibrated independently on identical views",
             12, fill=MUTED),
    ]
    for tick in range(0, 7):
        value = tick * 0.05
        y = top + chart_height * (1.0 - value / maximum)
        elements.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{width-right}" y2="{y:.2f}" '
            f'stroke="{GRID}" stroke-width="1"/>')
        elements.append(text(left - 10, y + 5, f"{value:.2f}", 11,
                             anchor="end", fill=MUTED))
    group_width = chart_width / len(rows)
    bar_width = group_width * 0.28
    for index, row in enumerate(rows):
        center = left + group_width * (index + 0.5)
        for offset, key, color in [(-bar_width, "clean_calib_rms_px", BLUE),
                                   (0, "opencv_rms_px", ORANGE)]:
            value = float(row[key])
            bar_height = chart_height * value / maximum
            x = center + offset
            y = top + chart_height - bar_height
            elements.append(
                f'<rect x="{x:.2f}" y="{y:.2f}" width="{bar_width:.2f}" '
                f'height="{bar_height:.2f}" fill="{color}" rx="2"/>')
        elements.append(text(center, top + chart_height + 22,
                             row["image"].replace(".jpg", ""), 10,
                             anchor="middle", fill=MUTED))
    elements += [
        f'<rect x="370" y="442" width="12" height="12" fill="{BLUE}" rx="2"/>',
        text(390, 453, "clean-calib", 12, fill=MUTED),
        f'<rect x="500" y="442" width="12" height="12" fill="{ORANGE}" rx="2"/>',
        text(520, 453, "OpenCV classic", 12, fill=MUTED),
    ]
    write_svg("common_per_view_rms.svg", width, height, elements)


def main():
    with (RESULTS / "summary.csv").open(newline="", encoding="utf-8") as handle:
        summary = list(csv.DictReader(handle))
    with (RESULTS / "common_per_view_rms.csv").open(
            newline="", encoding="utf-8") as handle:
        per_view = list(csv.DictReader(handle))
    summary_plot(summary)
    per_view_plot(per_view)


if __name__ == "__main__":
    main()
