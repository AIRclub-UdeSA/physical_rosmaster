#!/usr/bin/env python3
# Copyright 2026 AIRclub UdeSA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Measure Astra depth scale and noise against a flat wall.

`tools/sensor_capability_probe.py` reports how much depth arrives and how
fast. This answers whether a reported metre is a metre, which needs a surface
at a known distance.

Reading the centre pixel and comparing it against a tape does not work. A
robot is never square to the wall by hand, and depth read off a slanted
surface is the distance to that slant, not to the wall: on `x3-c` a 16 degree
yaw turned a true -1.2% error into an apparent +5.7% one, with the sign
flipped. Fitting the plane instead recovers the perpendicular distance, the
yaw, the camera's pitch, and the scatter about the surface -- which is the
depth noise with the geometry removed.

The plane is found by RANSAC because real rooms contain furniture. A table
between the robot and the wall is simply a second plane with fewer points;
least squares would average the two and report a surface that exists nowhere.
The inlier threshold then adapts to the residual spread, since depth noise
grows steeply with range and a threshold suited to 3.5 m is far too loose at
0.7 m.

Run it on the robot, once per position, and vary the distance: one distance
cannot separate a scale error from a fixed offset in the measuring setup.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


# cam_1_link sits at x=57.105 mm and the base mesh reaches x=116.5 mm, so a
# tape held against the front of the chassis starts this far ahead of the
# depth origin. Measuring from the chassis is easier than guessing where
# inside the camera body the sensor sits.
CHASSIS_FRONT_TO_DEPTH_ORIGIN = 0.0594


def collect(frames_wanted, timeout):
    """Gather depth frames and the intrinsics needed to project them."""
    rclpy.init()
    node = rclpy.create_node("depth_wall_calibration")
    frames, info = [], {}
    node.create_subscription(
        Image,
        "/cam_1/depth/image_raw",
        lambda m: frames.append(
            np.frombuffer(bytes(m.data), dtype="<f4")
            .reshape(m.height, m.step // 4)[:, : m.width].copy()
        ),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        CameraInfo,
        "/cam_1/depth/camera_info",
        lambda m: info.setdefault("k", list(m.k)),
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + timeout
    while (
        rclpy.ok()
        and (len(frames) < frames_wanted or not info)
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    return frames, info


def project(depth, intrinsics):
    """Turn a depth image into optical-frame points, dropping invalid pixels."""
    height, width = depth.shape
    fx, cx, fy, cy = intrinsics[0], intrinsics[2], intrinsics[4], intrinsics[5]
    us, vs = np.meshgrid(np.arange(width), np.arange(height))
    valid = np.isfinite(depth)
    return np.column_stack(
        [
            ((us - cx) * depth / fx)[valid],
            ((vs - cy) * depth / fy)[valid],
            depth[valid],
        ]
    )


def plane_from_triplet(sample):
    """Return a unit normal and offset for three points, or None if collinear."""
    normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
    length = np.linalg.norm(normal)
    if length < 1e-9:
        return None
    normal = normal / length
    return normal, float(normal @ sample[0])


def fit_plane(points, iterations, threshold, seed):
    """Find the dominant plane by RANSAC, then refine it on its own inliers."""
    rng = np.random.default_rng(seed)
    best_hits, normal, offset = 0, None, None
    for _ in range(iterations):
        candidate = plane_from_triplet(
            points[rng.choice(len(points), 3, replace=False)]
        )
        if candidate is None:
            continue
        hits = int(
            np.count_nonzero(
                np.abs(points @ candidate[0] - candidate[1]) < threshold
            )
        )
        if hits > best_hits:
            best_hits, normal, offset = hits, candidate[0], candidate[1]
    if normal is None:
        raise SystemExit("RANSAC found no plane")

    # Least squares on the inliers, re-selecting each round with a threshold
    # taken from the residual spread so the range sets the scale rather than
    # a constant tuned for one distance.
    for _ in range(3):
        inliers = np.abs(points @ normal - offset) < threshold
        selected = points[inliers]
        centroid = selected.mean(axis=0)
        _, _, vt = np.linalg.svd(selected - centroid, full_matrices=False)
        normal = vt[2] / np.linalg.norm(vt[2])
        offset = float(normal @ centroid)
        mad = float(np.median(np.abs(points[inliers] @ normal - offset)))
        threshold = max(0.01, 3.0 * 1.4826 * mad)
    if normal[2] > 0:
        normal, offset = -normal, -offset
    return normal, offset, threshold


def main():
    """Fit the wall, then report distance, angles, and depth noise."""
    parser = argparse.ArgumentParser(
        description="Measure Astra depth scale and noise against a flat wall."
    )
    parser.add_argument(
        "--tape",
        type=float,
        default=None,
        help="tape distance from the CHASSIS FRONT to the wall, in metres",
    )
    parser.add_argument(
        "--tape-case",
        type=float,
        default=0.0,
        help="length of the tape measure's own case in metres, added to --tape; "
        "a retracted case butted against a surface is a constant offset that "
        "otherwise looks exactly like a depth bias",
    )
    parser.add_argument("--frames", type=int, default=50)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--iterations", type=int, default=300)
    parser.add_argument("--threshold", type=float, default=0.08)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output")
    args = parser.parse_args()

    frames, info = collect(args.frames, args.timeout)
    if len(frames) < 2 or not info:
        raise SystemExit(
            "insufficient data: %d frames, camera_info=%s"
            % (len(frames), bool(info))
        )

    stack = np.stack(frames)
    depth = np.nanmedian(np.where(np.isfinite(stack), stack, np.nan), axis=0)
    points = project(depth, info["k"])
    if len(points) < 100:
        raise SystemExit("only %d valid depth points" % len(points))

    normal, offset, threshold = fit_plane(
        points, args.iterations, args.threshold, args.seed
    )
    residual = points @ normal - offset
    inliers = np.abs(residual) < threshold
    distance = abs(offset)
    yaw = math.degrees(math.atan2(normal[0], -normal[2]))
    pitch = math.degrees(math.atan2(normal[1], -normal[2]))
    rms = float(np.sqrt(np.mean(residual[inliers] ** 2)))
    valid_fraction = len(points) / depth.size

    result = {
        "frames": len(frames),
        "valid_fraction": valid_fraction,
        "inlier_fraction": float(inliers.mean()),
        "inlier_threshold_m": threshold,
        "perpendicular_distance_m": distance,
        "yaw_deg": yaw,
        "pitch_deg": pitch,
        "residual_rms_m": rms,
        "intrinsics": {
            "fx": info["k"][0], "fy": info["k"][4],
            "cx": info["k"][2], "cy": info["k"][5],
        },
    }

    print("=" * 70)
    print("Astra depth calibration against a flat wall")
    print("=" * 70)
    print("frames / valid pixels      : %d / %.1f%%"
          % (len(frames), 100 * valid_fraction))
    print("plane inliers              : %.1f%% of valid points (threshold %.3f m)"
          % (100 * inliers.mean(), threshold))
    print()
    print("perpendicular distance     : %.4f m   (depth origin to wall)" % distance)
    if args.tape is not None:
        # The chassis offset lies along the robot's x axis, so only its
        # component along the wall normal counts when the robot is turned.
        truth = (
            args.tape
            + args.tape_case
            + CHASSIS_FRONT_TO_DEPTH_ORIGIN * math.cos(math.radians(yaw))
        )
        error = distance - truth
        result.update({
            "tape_m": args.tape, "tape_case_m": args.tape_case,
            "truth_m": truth, "error_m": error,
            "error_percent": 100 * error / truth,
        })
        print("tape + case + chassis      : %.4f m  <-- ground truth" % truth)
        print("ERROR                      : %+.4f m  (%+.2f%%)"
              % (error, 100 * error / truth))
    print()
    print("camera yaw vs wall         : %+.2f deg" % yaw)
    print("camera pitch vs wall       : %+.2f deg" % pitch)
    print()
    print("residual RMS about plane   : %.4f m   <-- depth noise, geometry removed"
          % rms)
    off_plane = ~inliers
    if off_plane.sum():
        print()
        print("off-plane points (furniture, clutter): %.1f%%, median depth %.2f m"
              % (100 * off_plane.mean(), float(np.median(points[off_plane][:, 2]))))
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(result, handle, indent=2, sort_keys=True)
        print("\nJSON written to %s" % args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
