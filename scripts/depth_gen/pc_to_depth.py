#!/usr/bin/env python3
"""Accumulate PointCloud2 messages in a ROS bag and project them to one depth PNG.

Usage: pc_to_depth.py <config.yaml> ****
"""

import os
import sys

import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
import yaml

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False
    import imageio.v2 as imageio


def load_config(path):
    if not os.path.isfile(path):
        sys.exit(f"Config file not found: {path}")
    with open(path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    for key in ("bag", "topic"):
        if not cfg.get(key):
            sys.exit(f"Config missing required field: '{key}'")

    cfg.setdefault("output", "depth_out")
    cfg.setdefault("depth_filename", "depth.png")
    cfg.setdefault("color_filename", "depth_color.png")
    cfg.setdefault("depth_scale", 1000.0)
    cfg.setdefault("max_depth", None)
    cfg.setdefault("intrinsics", None)
    cfg.setdefault("extrinsics", None)
    return cfg


def _quat_to_matrix(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        raise ValueError("Zero-norm quaternion in extrinsics")
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w),     s * (x * z + y * w)],
        [s * (x * y + z * w),     1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),     s * (y * z + x * w),     1 - s * (x * x + y * y)],
    ], dtype=np.float64)


def resolve_extrinsics(cfg):
    """Return 4x4 T_cam_lidar (point in lidar frame -> point in camera frame)."""
    ext = cfg.get("extrinsics")
    if not ext:
        return np.eye(4, dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    if "matrix" in ext:
        M = np.array(ext["matrix"], dtype=np.float64)
        if M.shape == (4, 4):
            T = M
        elif M.shape == (3, 4):
            T[:3, :] = M
        else:
            sys.exit(f"extrinsics.matrix must be 4x4 or 3x4, got {M.shape}")
    else:
        if "rotation_matrix" in ext:
            R = np.array(ext["rotation_matrix"], dtype=np.float64)
            if R.shape != (3, 3):
                sys.exit(f"extrinsics.rotation_matrix must be 3x3, got {R.shape}")
        elif "quaternion" in ext:
            R = _quat_to_matrix(ext["quaternion"])
        else:
            sys.exit("extrinsics must contain 'matrix', 'rotation_matrix', "
                     "or 'quaternion'")
        t = np.array(ext.get("translation", [0.0, 0.0, 0.0]), dtype=np.float64)
        if t.shape != (3,):
            sys.exit(f"extrinsics.translation must be length 3, got {t.shape}")
        T[:3, :3] = R
        T[:3, 3] = t
    return T


REQUIRED_INTR = ("fx", "fy", "cx", "cy", "width", "height")


def _is_valid_intrinsics(intr):
    if not intr or any(intr.get(k) is None for k in REQUIRED_INTR):
        return False
    return intr["fx"] > 0 and intr["fy"] > 0 and intr["width"] > 0 and intr["height"] > 0


def _intrinsics_from_cfg(cfg):
    intr = cfg.get("intrinsics") or {}
    missing = [k for k in REQUIRED_INTR if intr.get(k) is None]
    if missing:
        return None
    return {k: intr[k] for k in REQUIRED_INTR}


def resolve_intrinsics(cfg):
    intr = _intrinsics_from_cfg(cfg)
    if not _is_valid_intrinsics(intr):
        sys.exit("Config must provide a valid 'intrinsics' block with positive "
                 f"{REQUIRED_INTR}")
    return intr


def read_accumulated_points(bag, topic):
    clouds = []
    frame_count = 0
    total_points = 0
    for _, msg, _ in bag.read_messages(topics=[topic]):
        pts = np.array(list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)
        if pts.size == 0:
            frame_count += 1
            continue
        pts = pts.reshape(-1, 3)
        clouds.append(pts)
        frame_count += 1
        total_points += pts.shape[0]
        if frame_count % 20 == 0:
            print(f"  accumulated {frame_count} frames, {total_points} points")

    if not clouds:
        return np.empty((0, 3), dtype=np.float32), frame_count
    return np.vstack(clouds), frame_count


def points_to_depth(pts, intr, T_cam_lidar, depth_scale, max_depth):
    if pts.size == 0:
        depth = np.zeros((intr["height"], intr["width"]), dtype=np.uint16)
        return depth, {
            "total_points": 0,
            "positive_z_points": 0,
            "projected_points": 0,
            "nonzero_pixels": 0,
            "min_depth_m": 0.0,
            "max_depth_m": 0.0,
        }

    R = T_cam_lidar[:3, :3].astype(np.float32)
    t = T_cam_lidar[:3, 3].astype(np.float32)
    pts_cam = pts @ R.T + t

    x, y, z = pts_cam[:, 0], pts_cam[:, 1], pts_cam[:, 2]
    valid = z > 0
    if max_depth is not None:
        valid &= z <= max_depth
    x, y, z = x[valid], y[valid], z[valid]
    positive_z_points = z.size

    if z.size == 0:
        depth = np.zeros((intr["height"], intr["width"]), dtype=np.uint16)
        return depth, {
            "total_points": pts.shape[0],
            "positive_z_points": 0,
            "projected_points": 0,
            "nonzero_pixels": 0,
            "min_depth_m": 0.0,
            "max_depth_m": 0.0,
        }

    u = np.round(intr["fx"] * x / z + intr["cx"]).astype(np.int32)
    v = np.round(intr["fy"] * y / z + intr["cy"]).astype(np.int32)

    in_img = (u >= 0) & (u < intr["width"]) & (v >= 0) & (v < intr["height"])
    u, v, z = u[in_img], v[in_img], z[in_img]
    projected_points = z.size

    depth = np.full((intr["height"], intr["width"]), np.inf, dtype=np.float32)
    if projected_points:
        flat_idx = v * intr["width"] + u
        np.minimum.at(depth.reshape(-1), flat_idx, z)

    depth[~np.isfinite(depth)] = 0.0
    nonzero = depth > 0.0
    min_depth = float(depth[nonzero].min()) if np.any(nonzero) else 0.0
    max_depth_value = float(depth[nonzero].max()) if np.any(nonzero) else 0.0
    scaled = np.clip(depth * depth_scale, 0, np.iinfo(np.uint16).max)
    stats = {
        "total_points": pts.shape[0],
        "positive_z_points": positive_z_points,
        "projected_points": projected_points,
        "nonzero_pixels": int(np.count_nonzero(scaled)),
        "min_depth_m": min_depth,
        "max_depth_m": max_depth_value,
    }
    return scaled.astype(np.uint16), stats


def save_png(path, img):
    if _HAS_CV2:
        cv2.imwrite(path, img)
    else:
        imageio.imwrite(path, img)


def _normalize_depth_for_display(depth):
    valid = depth > 0
    normalized = np.zeros(depth.shape, dtype=np.uint8)
    if not np.any(valid):
        return normalized, valid

    lo, hi = np.percentile(depth[valid], [2, 98])
    if hi <= lo:
        hi = float(depth[valid].max())
        lo = float(depth[valid].min())
    if hi > lo:
        normalized[valid] = np.clip((depth[valid] - lo) * 255.0 / (hi - lo), 0, 255)
    return normalized, valid


def _fallback_colormap(gray):
    stops = np.array([
        [0, 0, 128],
        [0, 128, 255],
        [0, 255, 255],
        [255, 255, 0],
        [255, 0, 0],
    ], dtype=np.float32)
    x = gray.astype(np.float32) * (len(stops) - 1) / 255.0
    idx = np.floor(x).astype(np.int32)
    idx = np.clip(idx, 0, len(stops) - 2)
    frac = (x - idx)[..., None]
    rgb = stops[idx] * (1.0 - frac) + stops[idx + 1] * frac
    return rgb.astype(np.uint8)


def save_color_depth(path, depth):
    if not path:
        return
    gray, valid = _normalize_depth_for_display(depth)
    if _HAS_CV2:
        color = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
        color[~valid] = 0
    else:
        color = _fallback_colormap(gray)
        color[~valid] = 0
    save_png(path, color)


def main():
    if len(sys.argv) != 2:
        sys.exit(f"Usage: {sys.argv[0]} <config.yaml>")
    cfg = load_config(sys.argv[1])

    if not os.path.isfile(cfg["bag"]):
        sys.exit(f"Bag file not found: {cfg['bag']}")
    os.makedirs(cfg["output"], exist_ok=True)

    bag = rosbag.Bag(cfg["bag"], "r")
    try:
        intr = resolve_intrinsics(cfg)
        T_cam_lidar = resolve_extrinsics(cfg)
        print(f"Intrinsics: {intr}")
        print(f"T_cam_lidar:\n{T_cam_lidar}")

        pts, frame_count = read_accumulated_points(bag, cfg["topic"])
        print(f"Accumulated {pts.shape[0]} points from {frame_count} pointcloud frames")

        depth, stats = points_to_depth(
            pts, intr, T_cam_lidar, cfg["depth_scale"], cfg["max_depth"]
        )
        depth_path = os.path.join(cfg["output"], cfg["depth_filename"])
        save_png(depth_path, depth)

        color_path = None
        if cfg.get("color_filename"):
            color_path = os.path.join(cfg["output"], cfg["color_filename"])
            save_color_depth(color_path, depth)

        print("Projection stats:")
        for key, value in stats.items():
            print(f"  {key}: {value}")
        print(f"Done. Wrote depth image: {depth_path}")
        if color_path:
            print(f"Done. Wrote color depth image: {color_path}")
    finally:
        bag.close()


if __name__ == "__main__":
    main()
