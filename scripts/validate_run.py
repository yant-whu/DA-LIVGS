#!/usr/bin/env python3
"""Validate and summarize a DA-LIVGS mapping output directory."""

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List

METRIC_COLUMNS = ("psnr", "ssim", "lpips")
SPLITS = ("train", "test")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate a DA-LIVGS mapping output directory"
    )
    parser.add_argument("run", type=Path, help="mapping output directory")
    parser.add_argument("--output", type=Path, help="optional JSON report path")
    return parser.parse_args()


def mean_metrics(rows: Iterable[Dict[str, str]]) -> Dict[str, float]:
    values: Dict[str, List[float]] = {name: [] for name in METRIC_COLUMNS}
    for row in rows:
        for name in METRIC_COLUMNS:
            value = float(row[name])
            if not math.isfinite(value):
                raise ValueError(f"non-finite {name} value")
            values[name].append(value)
    if any(not entries for entries in values.values()):
        raise ValueError("a split has no complete metric rows")
    return {
        name: sum(entries) / len(entries) for name, entries in values.items()
    }


def main() -> int:
    arguments = parse_arguments()
    run_path = arguments.run.resolve()
    metrics_path = run_path / "per_view_metrics.csv"
    if not metrics_path.is_file():
        raise SystemExit(f"missing {metrics_path}")
    point_clouds = sorted(run_path.glob("**/point_cloud.ply"))
    if not point_clouds:
        raise SystemExit(f"missing point_cloud.ply under {run_path}")

    with metrics_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    required = {"split", *METRIC_COLUMNS}
    if not rows or not required.issubset(rows[0]):
        raise SystemExit(
            "per_view_metrics.csv must contain split, psnr, ssim, and lpips columns"
        )
    metrics = {}
    frame_counts = {}
    try:
        for split in SPLITS:
            split_rows = [row for row in rows if row["split"] == split]
            metrics[split] = mean_metrics(split_rows)
            frame_counts[split] = len(split_rows)
    except (KeyError, ValueError) as error:
        raise SystemExit(str(error)) from error

    report = {
        "run": str(run_path),
        "valid": True,
        "point_cloud": str(point_clouds[0]),
        "per_view_metrics": str(metrics_path),
        "frame_counts": frame_counts,
        "metrics": metrics,
    }
    serialized = json.dumps(report, indent=2) + "\n"
    if arguments.output:
        arguments.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
