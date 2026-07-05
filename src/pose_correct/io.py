from __future__ import annotations

import csv
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from .models import CalibrationResult, Observation, PoseEstimate


CSV_COLUMNS = ("m", "n", "s_x", "s_y", "s_yaw")


def load_observations_csv(path: str | Path) -> list[Observation]:
    with Path(path).open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        missing = set(CSV_COLUMNS) - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"missing CSV columns: {', '.join(sorted(missing))}")
        return [
            Observation(
                m=float(row["m"]),
                n=float(row["n"]),
                s_x=float(row["s_x"]),
                s_y=float(row["s_y"]),
                s_yaw=float(row["s_yaw"]),
            )
            for row in reader
        ]


def save_poses_csv(path: str | Path, poses: list[PoseEstimate]) -> None:
    with Path(path).open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=("x", "y", "yaw"))
        writer.writeheader()
        for pose in poses:
            writer.writerow(asdict(pose))


def save_calibration(path: str | Path, calibration: CalibrationResult) -> None:
    with Path(path).open("w", encoding="utf-8") as handle:
        json.dump(asdict(calibration), handle, indent=2, sort_keys=True)
        handle.write("\n")


def load_calibration(path: str | Path) -> CalibrationResult:
    data: dict[str, Any]
    with Path(path).open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    return CalibrationResult(**data)
