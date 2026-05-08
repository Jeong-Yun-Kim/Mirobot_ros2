from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


def rotate_xy(dx: float, dy: float, deg_ccw: float) -> Tuple[float, float]:
    """Rotate an XY offset in the robot/base frame."""
    if abs(float(deg_ccw)) < 1e-9:
        return float(dx), float(dy)
    a = math.radians(float(deg_ccw))
    c = math.cos(a)
    s = math.sin(a)
    return c * float(dx) - s * float(dy), s * float(dx) + c * float(dy)


@dataclass
class PixelToRobotMapper:
    """Convert image pixel center (u, v) to Mirobot base XY.

    The default mode is dynamic/fallback mapping from your calibration direction code:
      camera pixel: +u right, +v down
      robot XY:     +X up,    +Y left
      dX = -dv * scale, dY = -du * scale

    Optional static mode keeps the older vision_robot_node.py style:
      pixel -> board plane by H_inv -> base by T_base_board.
    """

    static_board_path: str = ""
    use_static: bool = False
    fallback_origin_u: float = 640.0
    fallback_origin_v: float = 360.0
    fallback_origin_x: float = 140.0
    fallback_origin_y: float = 0.0
    fallback_mm_per_px: float = 0.5
    dynamic_rotation_deg: float = 0.0

    H_inv: Optional[np.ndarray] = None
    T_base_board: Optional[np.ndarray] = None
    mode: str = "dynamic"

    def load(self) -> None:
        self.H_inv = None
        self.T_base_board = None
        self.mode = "dynamic"

        path = os.path.expanduser(os.path.expandvars(self.static_board_path or ""))
        if not self.use_static:
            return
        if not path or not os.path.exists(path):
            raise FileNotFoundError(f"static calibration file not found: {path}")

        pose = np.load(path)
        if "H_inv" not in pose.files or "T_base_board" not in pose.files:
            raise ValueError(f"static calibration file keys must include H_inv and T_base_board. keys={pose.files}")

        self.H_inv = pose["H_inv"].astype(np.float64)
        self.T_base_board = pose["T_base_board"].astype(np.float64)
        if self.H_inv.shape != (3, 3):
            raise ValueError(f"H_inv must be 3x3, got {self.H_inv.shape}")
        if self.T_base_board.shape != (4, 4):
            raise ValueError(f"T_base_board must be 4x4, got {self.T_base_board.shape}")
        self.mode = "static"

    def pixel_to_base_xy(self, u: float, v: float) -> Tuple[float, float]:
        if self.mode == "static" and self.H_inv is not None and self.T_base_board is not None:
            pt_img = np.array([float(u), float(v), 1.0], dtype=np.float64).reshape(3, 1)
            pt_board_h = self.H_inv @ pt_img
            denom = float(pt_board_h[2, 0])
            if abs(denom) < 1e-9:
                raise ZeroDivisionError("static homography produced near-zero homogeneous scale")
            pt_board_h /= denom
            bx = float(pt_board_h[0, 0])
            by = float(pt_board_h[1, 0])
            p_board = np.array([bx, by, 0.0, 1.0], dtype=np.float64)
            p_base = self.T_base_board @ p_board
            return float(p_base[0]), float(p_base[1])

        du = float(u) - float(self.fallback_origin_u)
        dv = float(v) - float(self.fallback_origin_v)
        dx = -dv * float(self.fallback_mm_per_px)
        dy = -du * float(self.fallback_mm_per_px)
        dx, dy = rotate_xy(dx, dy, self.dynamic_rotation_deg)
        return float(self.fallback_origin_x) + dx, float(self.fallback_origin_y) + dy
