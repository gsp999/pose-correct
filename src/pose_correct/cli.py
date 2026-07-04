from __future__ import annotations

import argparse
from dataclasses import asdict

from .corrector import PoseCorrector
from .io import (
    load_calibration,
    load_observations_csv,
    save_calibration,
    save_poses_csv,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="pose-correct")
    subparsers = parser.add_subparsers(dest="command", required=True)

    fit = subparsers.add_parser("fit", help="fit a correction model from a CSV file")
    fit.add_argument("input_csv")
    fit.add_argument("--mn-distance", type=float, required=True)
    fit.add_argument("--model", default="pose_correct_model.json")
    fit.add_argument("--yaw-sign", type=int, choices=(-1, 1), default=1)
    fit.add_argument(
        "--method",
        choices=("explicit_geometry", "robust_geometric", "linear"),
        default="explicit_geometry",
    )
    fit.add_argument("--position-scale", type=float, default=0.02)
    fit.add_argument("--yaw-scale", type=float, default=0.02)

    correct = subparsers.add_parser("correct", help="apply a fitted model to a CSV file")
    correct.add_argument("input_csv")
    correct.add_argument("--model", default="pose_correct_model.json")
    correct.add_argument("--output", default="corrected_poses.csv")

    return parser


def main() -> None:
    args = build_parser().parse_args()

    if args.command == "fit":
        observations = load_observations_csv(args.input_csv)
        corrector = PoseCorrector.fit(
            observations,
            mn_distance=args.mn_distance,
            yaw_sign=args.yaw_sign,
            method=args.method,
            position_scale=args.position_scale,
            yaw_scale=args.yaw_scale,
        )
        save_calibration(args.model, corrector.calibration)
        print(asdict(corrector.calibration))
        return

    if args.command == "correct":
        observations = load_observations_csv(args.input_csv)
        corrector = PoseCorrector(load_calibration(args.model))
        save_poses_csv(args.output, corrector.correct_many(observations))
        print(f"wrote {args.output}")
        return

    raise AssertionError(f"unhandled command: {args.command}")


if __name__ == "__main__":
    main()
