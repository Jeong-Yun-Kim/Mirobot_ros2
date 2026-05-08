from __future__ import annotations

import math
from typing import Callable, Iterable, List, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Bounds = Tuple[float, float, float, float, float, float]


def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def clamp_xyz(x: float, y: float, z: float, bounds: Bounds) -> Vec3:
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    return (clamp(float(x), x_min, x_max), clamp(float(y), y_min, y_max), clamp(float(z), z_min, z_max))


def make_bezier_arc(
    p_start: Vec3,
    p_end: Vec3,
    *,
    bounds: Bounds,
    h_min: float = 40.0,
    k: float = 0.3,
    z_mid_min: float | None = None,
    z_max_margin: float = 10.0,
    n_points: int = 30,
) -> List[Vec3]:
    """Quadratic Bezier arc used for safe pick-and-place motion.

    This keeps the simple path style used in the uploaded vision_robot_node.py, but makes
    the workspace bounds configurable.
    """
    x1, y1, z1 = p_start
    x2, y2, z2 = p_end
    x_min, x_max, y_min, y_max, z_min, z_max = bounds

    n_points = max(2, int(n_points))
    d = math.hypot(x2 - x1, y2 - y1)
    h = max(float(h_min), float(k) * d)

    z_mid = max(z1, z2) + h
    z_mid = min(z_mid, z_max - z_max_margin)
    if z_mid_min is not None:
        z_mid = max(z_mid, float(z_mid_min))
    z_mid = clamp(z_mid, z_min, z_max)

    xm = (x1 + x2) / 2.0
    ym = (y1 + y2) / 2.0

    p0 = (x1, y1, z1)
    p1 = (xm, ym, z_mid)
    p2 = (x2, y2, z2)

    path: List[Vec3] = []
    for i in range(n_points):
        t = i / (n_points - 1)
        s = 1.0 - t
        x = s * s * p0[0] + 2.0 * s * t * p1[0] + t * t * p2[0]
        y = s * s * p0[1] + 2.0 * s * t * p1[1] + t * t * p2[1]
        z = s * s * p0[2] + 2.0 * s * t * p1[2] + t * t * p2[2]
        path.append(clamp_xyz(x, y, z, bounds))
    return path
