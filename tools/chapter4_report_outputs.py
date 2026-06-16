#!/usr/bin/env python3
"""Generate Chapter 4 report tables and figures from existing SIFT logs.

This helper is independent from the live navigation pipeline. It only reads
existing CSV/log artifacts and writes report summaries under chapter4_outputs/.
"""

from __future__ import annotations

import csv
import glob
import math
import os
from pathlib import Path
from datetime import datetime

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-chapter4")

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "chapter4_outputs"
FIG = OUT / "figures"
EXAMPLE_LOGS = ROOT / "examples" / "logs"

MAIN_LOGS = [
    ("Hover 100 m", EXAMPLE_LOGS / "sift_observe_100m_hover_60s.csv"),
    ("Hareket 100 m / ~1 m/s", EXAMPLE_LOGS / "sift_observe_100m_move_1ms_90s.csv"),
]

PERFORMANCE_LOGS = [
    ("100 m FOV65 768 px", ROOT / "live_sift_master_100m_fov65_768_long.csv"),
    ("100 m FOV65 1024 px", ROOT / "live_sift_master_100m_fov65_1024_long.csv"),
    ("512 px FOV65 no-zoom uzun gözlem", ROOT / "live_sift_master_512_fov65_nozoom_observe.csv"),
]

EXPERIMENTAL_LOGS = [
    ("Kalman validation", ROOT / "sift_validation_100m_hover_v2.csv"),
    ("Raw/Kalman baseline metrics", ROOT / "raw_baseline_validation_100m_hover_v2.csv"),
    ("Kalman tuning grid", ROOT / "kalman_tuning_validation_100m_hover_v2.csv"),
]


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(line for line in handle if not line.lstrip().startswith("#"))
        return reader.fieldnames or [], list(reader)


def as_float(value: str | None) -> float:
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def col(rows: list[dict[str, str]], name: str) -> np.ndarray:
    return np.array([as_float(row.get(name)) for row in rows], dtype=float)


def time_col(rows: list[dict[str, str]], name: str) -> np.ndarray:
    numeric_values = col(rows, name)
    if np.isfinite(numeric_values).any():
        return numeric_values

    parsed = []
    for row in rows:
        value = row.get(name, "")
        try:
            parsed.append(datetime.fromisoformat(value).timestamp())
        except ValueError:
            parsed.append(math.nan)
    return np.array(parsed, dtype=float)


def choose_error_col(fields: list[str], rows: list[dict[str, str]]) -> str:
    for name in ("gazebo_error_m", "calc_error_m", "error_m"):
        if name in fields and np.isfinite(col(rows, name)).any():
            return name
    return ""


def finite(values: np.ndarray) -> np.ndarray:
    return values[np.isfinite(values)]


def nan_stat(values: np.ndarray, fn, default: float = math.nan) -> float:
    values = finite(values)
    if len(values) == 0:
        return default
    return float(fn(values))


def duration_s(rows: list[dict[str, str]]) -> float:
    for name in ("timestamp", "time"):
        values = finite(time_col(rows, name))
        if len(values) >= 2:
            return float(np.max(values) - np.min(values))
    return math.nan


def truth_source(rows: list[dict[str, str]]) -> str:
    values = sorted({row.get("truth_source", "") for row in rows if row.get("truth_source", "")})
    return ",".join(values)


def summarize_match_log(name: str, path: Path, independent_truth: bool) -> dict[str, object]:
    fields, rows = read_rows(path)
    err_name = choose_error_col(fields, rows)
    err = col(rows, err_name) if err_name else np.array([], dtype=float)
    good = col(rows, "good_count")
    inl = col(rows, "inliers")
    dur = col(rows, "duration_sec")
    ratio = np.divide(inl, good, out=np.full_like(inl, np.nan), where=good != 0)

    nav = [row.get("nav_status", "") for row in rows]
    accepted = sum(1 for item in nav if item == "OK")
    rejected = sum(1 for item in nav if item == "REJECTED")
    total_nav = accepted + rejected
    avg_dur = nan_stat(dur, np.mean)

    return {
        "scenario": name,
        "file": path.name,
        "independent_gazebo_truth": independent_truth,
        "truth_source": truth_source(rows) or ("gazebo" if independent_truth else ""),
        "error_column_used": err_name,
        "duration_s": duration_s(rows),
        "samples": len(rows),
        "mean_error_m": nan_stat(err, np.mean),
        "median_error_m": nan_stat(err, np.median),
        "rmse_m": nan_stat(err, lambda x: np.sqrt(np.mean(np.square(x)))),
        "p95_error_m": nan_stat(err, lambda x: np.quantile(x, 0.95)),
        "max_error_m": nan_stat(err, np.max),
        "std_error_m": nan_stat(err, np.std),
        "mean_good_matches": nan_stat(good, np.mean),
        "mean_inliers": nan_stat(inl, np.mean),
        "mean_inlier_ratio": nan_stat(ratio, np.mean),
        "mean_processing_time_s": avg_dur,
        "approx_processing_fps": 1.0 / avg_dur if avg_dur and avg_dur > 0 else math.nan,
        "accepted_count": accepted,
        "rejected_count": rejected,
        "accepted_ratio": accepted / total_nav if total_nav else math.nan,
        "rejected_ratio": rejected / total_nav if total_nav else math.nan,
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def fmt(value: object) -> str:
    if isinstance(value, float):
        if math.isnan(value):
            return ""
        return f"{value:.4f}"
    return str(value)


def write_markdown(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = list(rows[0].keys())
    lines = [
        "| " + " | ".join(fields) + " |",
        "| " + " | ".join(["---"] * len(fields)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(fmt(row.get(field, "")) for field in fields) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def load_plot_data(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    return read_rows(path)


def rel_time(rows: list[dict[str, str]]) -> np.ndarray:
    t = time_col(rows, "timestamp")
    if not np.isfinite(t).any():
        t = time_col(rows, "time")
    valid = finite(t)
    if len(valid) == 0:
        return t
    return t - np.nanmin(valid)


def save_main_plots(logs: list[tuple[str, Path]]) -> None:
    loaded = [(name, *load_plot_data(path), path) for name, path in logs]

    plt.figure(figsize=(10, 5))
    for name, fields, rows, _ in loaded:
        err_name = choose_error_col(fields, rows)
        plt.plot(rel_time(rows), col(rows, err_name), label=name, linewidth=1.8)
    plt.xlabel("Zaman (s)")
    plt.ylabel("Konum hatası (m)")
    plt.title("Zaman İçinde Yatay Konum Hatası")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(FIG / "main_error_vs_time.png", dpi=220)
    plt.close()

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, (name, _, rows, _) in zip(axes, loaded):
        ax.plot(col(rows, "true_east_m"), col(rows, "true_north_m"), label="Gazebo truth", linewidth=2.0)
        ax.plot(col(rows, "pred_east_m"), col(rows, "pred_north_m"), label="SIFT tahmin", linewidth=1.5)
        ax.set_title(name)
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.grid(True, alpha=0.3)
        ax.axis("equal")
        ax.legend()
    fig.suptitle("Tahmini Konum ve Gazebo Ground Truth Yörüngesi")
    fig.tight_layout()
    fig.savefig(FIG / "main_trajectory_pred_vs_truth.png", dpi=220)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, (name, _, rows, _) in zip(axes, loaded):
        ax.plot(rel_time(rows), col(rows, "good_count"), label="Good match", linewidth=1.3)
        ax.plot(rel_time(rows), col(rows, "inliers"), label="Inlier", linewidth=1.3)
        ax.set_title(name)
        ax.set_xlabel("Zaman (s)")
        ax.set_ylabel("Eşleşme sayısı")
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.suptitle("Good Match ve Inlier Sayısı")
    fig.tight_layout()
    fig.savefig(FIG / "main_good_matches_inliers.png", dpi=220)
    plt.close(fig)

    plt.figure(figsize=(10, 5))
    for name, _, rows, _ in loaded:
        plt.plot(rel_time(rows), col(rows, "duration_sec"), label=name, linewidth=1.5)
    plt.xlabel("Zaman (s)")
    plt.ylabel("İşlem süresi (s)")
    plt.title("Frame Başına İşlem Süresi")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(FIG / "main_processing_time.png", dpi=220)
    plt.close()

    plt.figure(figsize=(9, 5))
    for name, fields, rows, _ in loaded:
        err_name = choose_error_col(fields, rows)
        plt.hist(finite(col(rows, err_name)), bins=28, alpha=0.58, label=name)
    plt.xlabel("Konum hatası (m)")
    plt.ylabel("Frekans")
    plt.title("Konum Hatası Histogramı")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(FIG / "main_error_histogram.png", dpi=220)
    plt.close()

    data = []
    labels = []
    for name, fields, rows, _ in loaded:
        err_name = choose_error_col(fields, rows)
        data.append(finite(col(rows, err_name)))
        labels.append(name)
    plt.figure(figsize=(8, 5))
    plt.boxplot(data, labels=labels, showfliers=True)
    plt.ylabel("Konum hatası (m)")
    plt.title("Hover ve Hareketli Senaryo Hata Karşılaştırması")
    plt.grid(True, axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(FIG / "main_error_boxplot.png", dpi=220)
    plt.close()


def experimental_summary() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for purpose, path in EXPERIMENTAL_LOGS:
        rows.append(
            {
                "log": path.name,
                "purpose": purpose,
                "exists": path.exists(),
                "report_use": "Deneysel entegrasyon/filtreleme; ana SIFT doğruluk başarısı olarak sunulmaz.",
            }
        )
    for path_str in sorted(glob.glob(str(ROOT / "gazebo_truth_vpe4hz*.log"))):
        rows.append(
            {
                "log": Path(path_str).name,
                "purpose": "Gazebo truth VPE kontrollü MAVLink baseline",
                "exists": True,
                "report_use": "External navigation hattının kontrollü denemesi; SIFT başarısı değildir.",
            }
        )
    for path_str in sorted(glob.glob(str(ROOT / "sift_src2_hover_test_*.csv"))):
        rows.append(
            {
                "log": Path(path_str).name,
                "purpose": "SIFT source set 2 / EKF entegrasyon denemesi",
                "exists": True,
                "report_use": "Sınırlılık ve entegrasyon denemesi; ana başarı sonucu değildir.",
            }
        )
    return rows


def main() -> None:
    OUT.mkdir(exist_ok=True)
    FIG.mkdir(exist_ok=True)

    main_rows = [summarize_match_log(name, path, True) for name, path in MAIN_LOGS]
    write_csv(OUT / "main_accuracy_summary.csv", main_rows)
    write_markdown(OUT / "main_accuracy_summary.md", main_rows)

    perf_rows = [summarize_match_log(name, path, False) for name, path in PERFORMANCE_LOGS]
    for row in perf_rows:
        row["truth_note"] = "Bağımsız Gazebo truth sonucu olarak kullanılmamalı; performans/eşleşme gözlemidir."
    write_csv(OUT / "performance_summary.csv", perf_rows)
    write_markdown(OUT / "performance_summary.md", perf_rows)

    exp_rows = experimental_summary()
    write_csv(OUT / "experimental_integration_summary.csv", exp_rows)
    write_markdown(OUT / "experimental_integration_summary.md", exp_rows)

    save_main_plots(MAIN_LOGS)

    print(f"Wrote {OUT.relative_to(ROOT)}")
    print(f"Main rows: {len(main_rows)}")
    print(f"Performance rows: {len(perf_rows)}")
    print(f"Experimental rows: {len(exp_rows)}")


if __name__ == "__main__":
    main()
