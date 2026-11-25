"""Smoke test: can pretrained DR-SPAAM load and run on this computer?

This does NOT measure accuracy. It only checks that the old research code
works with current PyTorch, and gives a first rough time per scan on CPU.

Usage:
    python smoke_test_drspaam.py checkpoints/<checkpoint file>.pth
"""
import argparse
import time

import numpy as np
from dr_spaam.detector import Detector

# RPLIDAR C1: about 5000 samples/s at 10 Hz = about 500 points per 360 deg scan (0.72 deg apart)
NUM_POINTS = 500


def make_fake_scan(num_points):
    """A round room with walls 4 m away and two 'legs' 2 m straight ahead."""
    ranges = np.full(num_points, 4.0)
    center = num_points // 2
    for leg_offset in (-5, 5):          # leg centers 10 points apart = 7.2 deg = about 25 cm at 2 m
        start = center + leg_offset - 2
        ranges[start:start + 5] = 2.0   # each leg 5 points wide = 3.6 deg = about 12 cm at 2 m
    return ranges


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint", help="path to a pretrained .pth file")
    parser.add_argument("--model", default="DR-SPAAM", choices=["DR-SPAAM", "DROW3"])
    args = parser.parse_args()

    detector = Detector(
        args.checkpoint,
        model=args.model,
        gpu=False,            # laptop has no NVIDIA GPU, and the Pi 5 won't either
        stride=1,
        panoramic_scan=True,  # the C1 sees a full 360 deg
    )
    detector.set_laser_fov(360)

    scan = make_fake_scan(NUM_POINTS)

    # DR-SPAAM remembers previous scans, so feed it the same scan several times
    times_ms = []
    for _ in range(20):
        t0 = time.perf_counter()
        dets_xy, dets_cls, _ = detector(scan)
        times_ms.append((time.perf_counter() - t0) * 1000.0)

    keep = dets_cls > 0.5
    print(f"Detections above 0.5 confidence: {keep.sum()}")
    for (x, y), conf in zip(dets_xy[keep], dets_cls[keep]):
        print(f"  x={x:6.2f}  y={y:6.2f}  conf={conf:.2f}")

    # The first few calls are slow (PyTorch warms up), so skip them
    print(f"Time per scan: median {np.median(times_ms[5:]):.1f} ms "
          f"(C1 sends a scan every 100 ms)")


if __name__ == "__main__":
    main()
