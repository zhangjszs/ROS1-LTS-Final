#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import rosbag


EARTH_RADIUS_M = 6378137.0


@dataclass
class PoseSample:
    t_sec: float
    x: float
    y: float
    yaw_rad: float


def _normalize_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def _heading_deg_to_enu_yaw_rad(heading_deg: float) -> float:
    heading_rad = math.radians(heading_deg)
    # heading: north=0, clockwise positive
    # ENU yaw: x-east=0, counter-clockwise positive
    return _normalize_angle((math.pi * 0.5) - heading_rad)


def _latlon_to_local_xy(
    lat_deg: float, lon_deg: float, lat0_deg: float, lon0_deg: float
) -> Tuple[float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)

    x = (lon - lon0) * math.cos(lat0) * EARTH_RADIUS_M
    y = (lat - lat0) * EARTH_RADIUS_M
    return x, y


def _msg_stamp(msg, bag_t_sec: float) -> float:
    if hasattr(msg, "header"):
        stamp = getattr(msg.header, "stamp", None)
        if stamp is not None:
            sec = stamp.to_sec()
            if sec > 0.0:
                return sec
    return bag_t_sec


def _extract_estimate(msg, bag_t_sec: float) -> Optional[PoseSample]:
    if not hasattr(msg, "car_state"):
        return None
    pose = msg.car_state
    if not all(hasattr(pose, key) for key in ("x", "y", "theta")):
        return None
    return PoseSample(
        t_sec=_msg_stamp(msg, bag_t_sec),
        x=float(pose.x),
        y=float(pose.y),
        yaw_rad=_normalize_angle(float(pose.theta)),
    )


def _extract_reference_raw(msg, bag_t_sec: float):
    t_sec = _msg_stamp(msg, bag_t_sec)

    # HUAT_InsP2-like
    if all(hasattr(msg, key) for key in ("Lat", "Lon", "Heading")):
        lat = float(msg.Lat)
        lon = float(msg.Lon)
        yaw = _heading_deg_to_enu_yaw_rad(float(msg.Heading))
        return t_sec, lat, lon, yaw

    # ASENSING-like
    if all(hasattr(msg, key) for key in ("latitude", "longitude", "azimuth")):
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        yaw = _heading_deg_to_enu_yaw_rad(float(msg.azimuth))
        return t_sec, lat, lon, yaw

    return None


def _load_samples(
    bag_path: Path, est_topic: str, ref_topic: str
) -> Tuple[List[PoseSample], List[PoseSample]]:
    est_samples: List[PoseSample] = []
    ref_geo: List[Tuple[float, float, float, float]] = []

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, bag_t in bag.read_messages(topics=[est_topic, ref_topic]):
            bag_t_sec = bag_t.to_sec()
            if topic == est_topic:
                sample = _extract_estimate(msg, bag_t_sec)
                if sample is not None:
                    est_samples.append(sample)
            elif topic == ref_topic:
                ref = _extract_reference_raw(msg, bag_t_sec)
                if ref is not None:
                    ref_geo.append(ref)

    if not est_samples:
        raise RuntimeError(f"No estimate samples found on topic: {est_topic}")
    if not ref_geo:
        raise RuntimeError(f"No reference samples found on topic: {ref_topic}")

    lat0 = ref_geo[0][1]
    lon0 = ref_geo[0][2]
    ref_samples: List[PoseSample] = []
    for t_sec, lat, lon, yaw in ref_geo:
        x, y = _latlon_to_local_xy(lat, lon, lat0, lon0)
        ref_samples.append(PoseSample(t_sec=t_sec, x=x, y=y, yaw_rad=yaw))

    est_samples.sort(key=lambda s: s.t_sec)
    ref_samples.sort(key=lambda s: s.t_sec)
    return est_samples, ref_samples


def _nearest_index(times: List[float], query: float) -> int:
    i = bisect.bisect_left(times, query)
    if i == 0:
        return 0
    if i >= len(times):
        return len(times) - 1
    left = i - 1
    right = i
    if abs(times[right] - query) < abs(query - times[left]):
        return right
    return left


def _compute_alignment(
    est_samples: List[PoseSample], ref_samples: List[PoseSample], sync_tol_s: float
) -> Tuple[float, float, float]:
    ref_times = [s.t_sec for s in ref_samples]
    est0 = est_samples[0]
    ref0 = ref_samples[_nearest_index(ref_times, est0.t_sec)]

    if abs(est0.t_sec - ref0.t_sec) > sync_tol_s:
        raise RuntimeError(
            f"Cannot align trajectories: first matched dt={abs(est0.t_sec-ref0.t_sec):.3f}s "
            f"exceeds sync tolerance {sync_tol_s:.3f}s"
        )

    d_yaw = _normalize_angle(est0.yaw_rad - ref0.yaw_rad)
    cos_y = math.cos(d_yaw)
    sin_y = math.sin(d_yaw)
    x0_rot = cos_y * ref0.x - sin_y * ref0.y
    y0_rot = sin_y * ref0.x + cos_y * ref0.y
    tx = est0.x - x0_rot
    ty = est0.y - y0_rot
    return tx, ty, d_yaw


def _apply_alignment(sample: PoseSample, tx: float, ty: float, d_yaw: float) -> PoseSample:
    cos_y = math.cos(d_yaw)
    sin_y = math.sin(d_yaw)
    x = cos_y * sample.x - sin_y * sample.y + tx
    y = sin_y * sample.x + cos_y * sample.y + ty
    yaw = _normalize_angle(sample.yaw_rad + d_yaw)
    return PoseSample(t_sec=sample.t_sec, x=x, y=y, yaw_rad=yaw)


def _build_rows(
    est_samples: List[PoseSample],
    ref_samples: List[PoseSample],
    sync_tol_s: float,
    align_se2: bool,
) -> List[List[float]]:
    ref_times = [s.t_sec for s in ref_samples]

    tx, ty, d_yaw = (0.0, 0.0, 0.0)
    if align_se2:
        tx, ty, d_yaw = _compute_alignment(est_samples, ref_samples, sync_tol_s)

    rows = []
    for est in est_samples:
        idx = _nearest_index(ref_times, est.t_sec)
        ref = ref_samples[idx]
        dt = abs(est.t_sec - ref.t_sec)
        if dt > sync_tol_s:
            continue
        if align_se2:
            ref = _apply_alignment(ref, tx, ty, d_yaw)

        rows.append(
            [
                est.t_sec,
                est.x,
                est.y,
                est.yaw_rad,
                ref.x,
                ref.y,
                ref.yaw_rad,
                1,
                dt,
            ]
        )
    if not rows:
        raise RuntimeError(
            f"No aligned samples within sync tolerance ({sync_tol_s}s). "
            "Try increasing --sync-tol-s."
        )
    return rows


def _write_csv(output_csv: Path, rows: List[List[float]]) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "t_sec",
                "est_x",
                "est_y",
                "est_yaw_rad",
                "ref_x",
                "ref_y",
                "ref_yaw_rad",
                "est_valid",
                "sync_dt_s",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    f"{row[0]:.6f}",
                    f"{row[1]:.6f}",
                    f"{row[2]:.6f}",
                    f"{row[3]:.9f}",
                    f"{row[4]:.6f}",
                    f"{row[5]:.6f}",
                    f"{row[6]:.9f}",
                    int(row[7]),
                    f"{row[8]:.6f}",
                ]
            )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Build aligned localization evaluation CSV from rosbag. "
            "Reference defaults to INS topic and can be used as proxy baseline."
        )
    )
    parser.add_argument("--bag", type=Path, required=True, help="Input rosbag path.")
    parser.add_argument(
        "--est-topic",
        type=str,
        default="/localization/car_state",
        help="Estimate topic (HUAT_CarState).",
    )
    parser.add_argument(
        "--ref-topic",
        type=str,
        default="/pbox_pub/Ins",
        help="Reference topic (HUAT_InsP2 or ASENSING).",
    )
    parser.add_argument(
        "--sync-tol-s",
        type=float,
        default=0.05,
        help="Timestamp nearest-neighbor tolerance in seconds.",
    )
    parser.add_argument(
        "--align-se2",
        action="store_true",
        help="Apply first-pair SE2 alignment from reference frame to estimate frame.",
    )
    parser.add_argument(
        "--no-align-se2",
        action="store_false",
        dest="align_se2",
        help="Disable SE2 alignment.",
    )
    parser.set_defaults(align_se2=True)
    parser.add_argument(
        "--output-csv",
        type=Path,
        default=Path("docs/baseline/localization_eval.csv"),
        help="Output aligned evaluation CSV path.",
    )
    args = parser.parse_args()

    est_samples, ref_samples = _load_samples(args.bag, args.est_topic, args.ref_topic)
    rows = _build_rows(
        est_samples=est_samples,
        ref_samples=ref_samples,
        sync_tol_s=args.sync_tol_s,
        align_se2=args.align_se2,
    )
    _write_csv(args.output_csv, rows)

    print(f"[OK] estimate samples: {len(est_samples)}")
    print(f"[OK] reference samples: {len(ref_samples)}")
    print(f"[OK] aligned rows: {len(rows)}")
    print(f"[OK] output csv: {args.output_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
