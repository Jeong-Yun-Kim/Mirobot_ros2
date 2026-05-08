from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

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
    # tolerate labels like "red-cube-v2" or "blue_cube_1"
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


class CvCamera:
    def __init__(self, cam_id: int, width: int = 1280, height: int = 720, backend: str = "v4l2", logger=None):
        self.cam_id = int(cam_id)
        self.width = int(width)
        self.height = int(height)
        self.backend = backend
        self.logger = logger
        self.cap: Optional[cv2.VideoCapture] = None

    def log(self, level: str, msg: str) -> None:
        if self.logger is not None and hasattr(self.logger, level):
            getattr(self.logger, level)(msg)
        else:
            print(f"[{level.upper()}] {msg}")

    def open(self) -> None:
        if self.cap is not None and self.cap.isOpened():
            return
        flag = backend_flag(self.backend)
        self.cap = cv2.VideoCapture(self.cam_id, flag) if flag else cv2.VideoCapture(self.cam_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera id={self.cam_id} backend={self.backend}")
        if self.width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self.log("info", f"camera opened: id={self.cam_id}, size={int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def read(self, flush: int = 0) -> np.ndarray:
        self.open()
        assert self.cap is not None
        frame = None
        ok = False
        for _ in range(max(0, int(flush)) + 1):
            ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("camera frame read failed")
        return frame


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
        if self.logger is not None and hasattr(self.logger, level):
            getattr(self.logger, level)(msg)
        else:
            print(f"[{level.upper()}] {msg}")

    def load(self) -> None:
        if self.model is not None:
            return
        import torch  # import lazily because ROS startup is faster and error messages are cleaner

        self.log("info", f"loading YOLO model: repo={self.repo}, weight={self.weight}")
        model = torch.hub.load(self.repo, "custom", path=self.weight, source="local")
        # YOLOv5 AutoShape attributes; harmless if the backend ignores them.
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

        # The uploaded code used results.pandas().xyxy[0]. Keep that path first.
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
        # prioritize confidence, then a central object if confidence ties
        return (d.conf, -dist, d.area)

    return max(candidates, key=score)


def draw_detections(frame_bgr: np.ndarray, detections: Sequence[Detection]) -> np.ndarray:
    out = frame_bgr.copy()
    for det in detections:
        x1, y1, x2, y2 = map(int, det.bbox)
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.putText(out, f"{det.cls} {det.conf:.2f}", (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    return out
