from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class Detection:
    cls: str
    color: str
    conf: float
    bbox: Tuple[float, float, float, float]

    @property
    def center(self) -> Tuple[float, float]:
        x1, y1, x2, y2 = self.bbox
        return (0.5 * (x1 + x2), 0.5 * (y1 + y2))

    @property
    def area(self) -> float:
        x1, y1, x2, y2 = self.bbox
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)


DEFAULT_COLOR_ALIASES: Dict[str, Sequence[str]] = {
    "red": ("red", "red-cube", "red_cube", "red cube", "redbox", "red-box"),
    "green": ("green", "green-cube", "green_cube", "green cube", "greenbox", "green-box"),
    "blue": ("blue", "blue-cube", "blue_cube", "blue cube", "bluebox", "blue-box"),
}


def normalize_label(label: str) -> str:
    return str(label).strip().lower().replace("_", "-")


def class_to_color(label: str, aliases: Optional[Dict[str, Sequence[str]]] = None) -> Optional[str]:
    aliases = aliases or DEFAULT_COLOR_ALIASES
    norm = normalize_label(label)
    for color, names in aliases.items():
        for name in names:
            if norm == normalize_label(name):
                return color
    for color in aliases.keys():
        if norm.startswith(color):
            return color
    return None


def backend_flag(name: str) -> int:
    name = (name or "").strip().lower()
    if name == "v4l2":
        return cv2.CAP_V4L2
    if name == "dshow":
        return cv2.CAP_DSHOW
    if name == "msmf":
        return cv2.CAP_MSMF
    if name in {"any", "auto", ""}:
        return 0
    return 0


def fourcc_to_str(value: float) -> str:
    try:
        n = int(value)
        chars = [chr((n >> (8 * i)) & 0xFF) for i in range(4)]
        s = "".join(chars)
        return s if s.strip("\x00") else str(n)
    except Exception:
        return str(value)


class CvCamera:
    def __init__(self, cam_id: int, width: int = 1280, height: int = 720, backend: str = "v4l2", logger=None):
        self.cam_id = int(cam_id)
        self.width = int(width)
        self.height = int(height)
        self.backend = backend
        self.logger = logger
        self.cap: Optional[cv2.VideoCapture] = None
        self.actual_width = 0
        self.actual_height = 0
        self.last_mean: Optional[float] = None
        self.active_fourcc = ""

    def log(self, level: str, msg: str) -> None:
        try:
            if self.logger is not None:
                norm = str(level).lower()
                if norm in {"warn", "warning"}:
                    self.logger.warning(msg)
                elif norm == "error":
                    self.logger.error(msg)
                elif norm == "debug":
                    self.logger.debug(msg)
                else:
                    self.logger.info(msg)
                return
        except Exception:
            pass
        print(f"[{str(level).upper()}] {msg}")

    def _open_with_fourcc(self, fourcc: str) -> tuple[bool, Optional[float]]:
        self.close()
        flag = backend_flag(self.backend)
        self.cap = cv2.VideoCapture(self.cam_id, flag) if flag else cv2.VideoCapture(self.cam_id)
        if not self.cap.isOpened():
            return False, None

        try:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        except Exception:
            pass
        if self.width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        try:
            self.cap.set(cv2.CAP_PROP_FPS, 30)
        except Exception:
            pass
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        # Warm up: the first several frames can be black or stale on some UVC cameras.
        best_mean: Optional[float] = None
        for _ in range(20):
            ret, frame = self.cap.read()
            if ret and frame is not None:
                mean = float(frame.mean())
                best_mean = mean if best_mean is None else max(best_mean, mean)
                if mean > 5.0:
                    # keep reading a few frames so exposure settles, but don't block long
                    pass
            time.sleep(0.03)

        self.actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.active_fourcc = fourcc_to_str(self.cap.get(cv2.CAP_PROP_FOURCC))
        self.last_mean = best_mean
        return True, best_mean

    def open(self) -> None:
        if self.cap is not None and self.cap.isOpened():
            return

        # Prefer MJPG for high-resolution UVC cameras.  If it opens but remains almost
        # black, try YUYV as a fallback.  This fixes many /dev/video4 black-window cases.
        candidates = ["MJPG", "YUYV"]
        errors: list[str] = []
        for fourcc in candidates:
            ok, mean = self._open_with_fourcc(fourcc)
            if not ok:
                errors.append(f"{fourcc}: open failed")
                continue
            if mean is None:
                errors.append(f"{fourcc}: no frame during warm-up")
                continue
            # Accept any real frame.  The debug overlay will warn if it is nearly black.
            self.log(
                "info",
                f"camera opened: id={self.cam_id}, size={self.actual_width}x{self.actual_height}, "
                f"fps={self.cap.get(cv2.CAP_PROP_FPS):.1f}, request_fourcc={fourcc}, "
                f"actual_fourcc={self.active_fourcc}, last_mean={mean:.2f}",
            )
            if mean <= 2.0 and fourcc != candidates[-1]:
                self.log("warn", f"camera frame is almost black with {fourcc}; retrying another format")
                continue
            return

        self.close()
        raise RuntimeError(f"cannot open/read camera id={self.cam_id} backend={self.backend}; attempts={errors}")

    def close(self) -> None:
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

    def get_size(self) -> tuple[int, int]:
        self.open()
        if self.actual_width <= 0 or self.actual_height <= 0:
            assert self.cap is not None
            self.actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return self.actual_width, self.actual_height

    def read(self, flush: int = 0) -> np.ndarray:
        self.open()
        assert self.cap is not None
        frame = None
        ok = False
        best_frame = None
        best_mean = -1.0
        # Read several frames and keep the brightest usable one.  This avoids returning
        # one stale black frame after camera format changes.
        n_reads = max(3, int(flush)) + 1
        for _ in range(n_reads):
            ok, frame = self.cap.read()
            if ok and frame is not None:
                mean = float(frame.mean())
                if mean > best_mean:
                    best_mean = mean
                    best_frame = frame
                if mean > 5.0 and _ >= int(flush):
                    break
            time.sleep(0.01)
        if best_frame is None:
            raise RuntimeError("camera frame read failed")
        self.last_mean = best_mean
        if best_mean <= 2.0:
            self.log("warn", f"camera frame is almost black: mean={best_mean:.2f}")
        return best_frame


class YoloCubeDetector:
    def __init__(
        self,
        repo: str,
        weight: str,
        *,
        conf: float = 0.5,
        iou: float = 0.45,
        yolo_size: int = 640,
        aliases: Optional[Dict[str, Sequence[str]]] = None,
        logger=None,
    ):
        self.repo = repo
        self.weight = weight
        self.conf = float(conf)
        self.iou = float(iou)
        self.yolo_size = int(yolo_size)
        self.aliases = aliases or DEFAULT_COLOR_ALIASES
        self.logger = logger
        self.model = None

    def log(self, level: str, msg: str) -> None:
        try:
            if self.logger is not None:
                norm = str(level).lower()
                if norm in {"warn", "warning"}:
                    self.logger.warning(msg)
                elif norm == "error":
                    self.logger.error(msg)
                elif norm == "debug":
                    self.logger.debug(msg)
                else:
                    self.logger.info(msg)
                return
        except Exception:
            pass
        print(f"[{str(level).upper()}] {msg}")

    def load(self) -> None:
        if self.model is not None:
            return
        import torch

        self.log("info", f"loading YOLO model: repo={self.repo}, weight={self.weight}")
        model = torch.hub.load(self.repo, "custom", path=self.weight, source="local")
        try:
            model.conf = self.conf
            model.iou = self.iou
        except Exception:
            pass
        self.model = model
        self.log("info", "YOLO model loaded")

    def detect(self, frame_bgr: np.ndarray) -> List[Detection]:
        self.load()
        if frame_bgr is None:
            return []
        h0, w0 = frame_bgr.shape[:2]
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        frame_rgb_resized = cv2.resize(frame_rgb, (self.yolo_size, self.yolo_size), interpolation=cv2.INTER_LINEAR)
        results = self.model(frame_rgb_resized)  # type: ignore[misc]

        if hasattr(results, "pandas"):
            df = results.pandas().xyxy[0]
            detections: List[Detection] = []
            sx = w0 / float(self.yolo_size)
            sy = h0 / float(self.yolo_size)
            for _, row in df.iterrows():
                conf = float(row.get("confidence", row.get("conf", 0.0)))
                if conf < self.conf:
                    continue
                cls = str(row.get("name", row.get("class", "")))
                color = class_to_color(cls, self.aliases)
                if color is None:
                    continue
                xmin = float(row["xmin"]) * sx
                ymin = float(row["ymin"]) * sy
                xmax = float(row["xmax"]) * sx
                ymax = float(row["ymax"]) * sy
                detections.append(Detection(cls=cls, color=color, conf=conf, bbox=(xmin, ymin, xmax, ymax)))
            return detections

        raise RuntimeError("unsupported YOLO result object. Expected YOLOv5 results.pandas().xyxy[0]")


def choose_best_detection(detections: Sequence[Detection], color: str, image_center: Optional[Tuple[float, float]] = None) -> Optional[Detection]:
    candidates = [d for d in detections if d.color == color]
    if not candidates:
        return None
    if image_center is None:
        return max(candidates, key=lambda d: (d.conf, d.area))
    cx, cy = image_center

    def score(d: Detection):
        u, v = d.center
        dist = ((u - cx) ** 2 + (v - cy) ** 2) ** 0.5
        return (d.conf, -dist, d.area)

    return max(candidates, key=score)


def draw_detections(frame_bgr: np.ndarray, detections: Sequence[Detection]) -> np.ndarray:
    out = frame_bgr.copy()
    for det in detections:
        x1, y1, x2, y2 = map(int, det.bbox)
        color = (255, 255, 255)
        if det.color == "red":
            color = (0, 0, 255)
        elif det.color == "green":
            color = (0, 255, 0)
        elif det.color == "blue":
            color = (255, 0, 0)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        cv2.putText(out, f"{det.cls} {det.conf:.2f}", (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        u, v = det.center
        cv2.circle(out, (int(u), int(v)), 4, color, -1, cv2.LINE_AA)
    return out
