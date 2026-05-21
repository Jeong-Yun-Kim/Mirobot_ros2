from __future__ import annotations

import json
import os
import queue
import threading
import time
import traceback
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import cv2
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from .calibration import PixelToRobotMapper
from .geometry import Vec3, clamp_xyz_report
from .robot_gcode import MirobotGCode, RobotConfig
from .vision import CvCamera, Detection, YoloCubeDetector, choose_best_detection, draw_detections

COLOR_KEYS = ("red", "green", "blue")


def package_share_or_source() -> str:
    try:
        return get_package_share_directory("mirobot_order_delivery")
    except PackageNotFoundError:
        return os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))


def as_float_list(value, length: int, name: str) -> List[float]:
    if isinstance(value, str):
        value = [x.strip() for x in value.split(",") if x.strip()]
    seq = list(value)
    if len(seq) != length:
        raise ValueError(f"{name} must have length {length}, got {value}")
    return [float(x) for x in seq]


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


@dataclass
class RuntimeStats:
    current_order_id: Optional[int] = None
    current_task: str = ""
    delivered_red: int = 0
    delivered_green: int = 0
    delivered_blue: int = 0


class MirobotOrderDeliveryNode(Node):
    """Subscribe to /robot/cmd and execute YOLO-guided cube filling with Mirobot."""

    def __init__(self):
        super().__init__("mirobot_order_delivery_node")
        self.share_dir = package_share_or_source()

        self.cmd_topic = self.param("topics.robot_cmd", "/robot/cmd")
        self.stop_topic = self.param("topics.robot_stop", "/robot/stop")
        self.done_topic = self.param("topics.robot_done", "/robot/done")
        self.status_topic = self.param("topics.robot_status", "/robot/status")

        self.motion_arc_points = int(self.param("motion.arc_points", 30))

        self.robot_cfg = RobotConfig(
            port=str(self.param("serial.port", "/dev/ttyUSB0")),
            baud=int(self.param("serial.baud", 115200)),
            eol=str(self.param("serial.eol", "\r\n")).encode("utf-8").decode("unicode_escape"),
            x_min=float(self.param("workspace.x_min", 140.0)),
            x_max=float(self.param("workspace.x_max", 290.0)),
            y_min=float(self.param("workspace.y_min", -270.0)),
            y_max=float(self.param("workspace.y_max", 270.0)),
            z_min=float(self.param("workspace.z_min", 40.0)),
            z_max=float(self.param("workspace.z_max", 300.0)),
            default_feed=float(self.param("motion.feed", 2000.0)),
            pump_pwm=int(self.param("pump.pwm", 1000)),
            pump_dwell_s=float(self.param("pump.pick_dwell_s", 1.5)),
            drop_dwell_s=float(self.param("pump.drop_dwell_s", 0.4)),
            command_timeout_s=float(self.param("serial.command_timeout_s", 12.0)),
            toggle_dtr_rts=as_bool(self.param("serial.toggle_dtr_rts", True)),
            verbose_tx=as_bool(self.param("serial.verbose_tx", True)),
            use_m400=as_bool(self.param("motion.use_m400", False)),
            use_status_wait=as_bool(self.param("motion.use_status_wait", False)),
            motion_settle_s=float(self.param("motion.settle_after_move_s", 0.20)),
            status_wait_timeout_s=float(self.param("motion.status_wait_timeout_s", 1.0)),
        )

        self.camera_id = int(self.param("camera.id", 4))
        self.camera_backend = str(self.param("camera.backend", "v4l2"))
        self.camera_width = int(self.param("camera.width", 1280))
        self.camera_height = int(self.param("camera.height", 720))
        self.camera_settle_s = float(self.param("camera.settle_s", 0.4))
        self.camera_flush_frames = int(self.param("camera.flush_frames", 3))

        self.yolo_repo = str(self.param("yolo.repo", "/home/kjy/yolov5"))
        self.yolo_weight = str(self.param("yolo.weight", "/home/kjy/yolov5/runs/train/exp/weights/best.onnx"))
        self.yolo_conf = float(self.param("yolo.conf", 0.5))
        self.yolo_iou = float(self.param("yolo.iou", 0.45))
        self.yolo_size = int(self.param("yolo.size", 640))

        default_static_path = os.path.join(self.share_dir, "data", "static_board_pose.npz")
        calibration_mode = str(self.param("calibration.mode", "dynamic")).strip().lower()
        use_static = calibration_mode == "static" or as_bool(self.param("calibration.use_static", False))
        static_path = str(self.param("calibration.static_board_path", ""))
        if not static_path:
            static_path = default_static_path
        self.mapper = PixelToRobotMapper(
            static_board_path=static_path,
            use_static=use_static,
            fallback_origin_u=float(self.param("calibration.fallback_origin_u", 640.0)),
            fallback_origin_v=float(self.param("calibration.fallback_origin_v", 360.0)),
            fallback_origin_x=float(self.param("calibration.fallback_origin_x", 200.0)),
            fallback_origin_y=float(self.param("calibration.fallback_origin_y", 0.0)),
            fallback_mm_per_px=float(self.param("calibration.fallback_mm_per_px", 0.25)),
            dynamic_rotation_deg=float(self.param("calibration.dynamic_rotation_deg", 0.0)),
            auto_origin_if_invalid=as_bool(self.param("calibration.auto_origin_if_invalid", True)),
        )
        try:
            self.mapper.load()
            self.log_mapper_config()
        except Exception as exc:
            self.get_logger().warning(f"static calibration load failed; dynamic mapping will be used: {exc}")
            self.mapper.use_static = False
            self.mapper.load()
            self.log_mapper_config()

        self.observe_xyz = tuple(as_float_list(self.param("poses.observe_xyz", [200.0, 0.0, 220.0]), 3, "poses.observe_xyz"))  # type: ignore[assignment]
        self.box_xyz = tuple(as_float_list(self.param("poses.box_xyz", [140.0, 250.0, 170.0]), 3, "poses.box_xyz"))  # type: ignore[assignment]
        self.pick_z = float(self.param("poses.pick_z", 55.0))
        self.travel_z = float(self.param("poses.travel_z", 220.0))
        self.box_approach_z = float(self.param("poses.box_approach_z", 220.0))
        self.pick_y_offset_mm = float(self.param("poses.pick_y_offset_mm", 15.0))
        self.pick_x_offset_mm = float(self.param("poses.pick_x_offset_mm", 0.0))

        self.order_sequence = [str(x).lower() for x in list(self.param("task.order_sequence", ["red", "green", "blue"]))]
        self.detect_timeout_s = float(self.param("task.detect_timeout_s", 5.0))
        self.detect_min_hits = int(self.param("task.detect_min_hits", 1))
        self.return_to_observe_after_each_cube = as_bool(self.param("task.return_to_observe_after_each_cube", True))
        self.pack_passthrough = as_bool(self.param("task.pack_passthrough", True))
        self.publish_failed_done = as_bool(self.param("task.publish_failed_done", False))
        self.allow_fixed_pick_fallback = as_bool(self.param("task.allow_fixed_pick_fallback", False))
        self.fixed_pick_positions = {
            "red": tuple(as_float_list(self.param("fixed_pick.red_xyz", [140.0, -90.0, self.pick_z]), 3, "fixed_pick.red_xyz")),
            "green": tuple(as_float_list(self.param("fixed_pick.green_xyz", [140.0, 0.0, self.pick_z]), 3, "fixed_pick.green_xyz")),
            "blue": tuple(as_float_list(self.param("fixed_pick.blue_xyz", [140.0, 90.0, self.pick_z]), 3, "fixed_pick.blue_xyz")),
        }

        self.box_grid_enabled = as_bool(self.param("box_grid.enabled", False))
        self.box_grid_cols = int(self.param("box_grid.cols", 3))
        self.box_grid_step_mm = float(self.param("box_grid.step_mm", 24.0))
        self.box_grid_axis = str(self.param("box_grid.axis", "y")).lower()

        self.debug_view = as_bool(self.param("debug.view", False))
        self.debug_window = str(self.param("debug.window_name", "mirobot_order_delivery"))
        self.debug_max_width = int(self.param("debug.max_width", 960))
        self.debug_save_latest_frame = as_bool(self.param("debug.save_latest_frame", True))
        self.debug_frame_path = str(self.param("debug.frame_path", "/tmp/mirobot_order_delivery_debug_latest.jpg"))
        self.debug_save_interval_s = float(self.param("debug.save_interval_s", 0.5))
        self._debug_frame_lock = threading.Lock()
        self._latest_debug_frame = None
        self._debug_window_created = False
        self._last_debug_save_t = 0.0

        self.robot = MirobotGCode(self.robot_cfg, logger=self.get_logger())
        self.camera = CvCamera(self.camera_id, self.camera_width, self.camera_height, self.camera_backend, logger=self.get_logger())
        self.detector = YoloCubeDetector(
            self.yolo_repo,
            self.yolo_weight,
            conf=self.yolo_conf,
            iou=self.yolo_iou,
            yolo_size=self.yolo_size,
            logger=self.get_logger(),
        )

        self.cmd_sub = self.create_subscription(String, self.cmd_topic, self.on_robot_cmd, 10)
        self.stop_sub = self.create_subscription(String, self.stop_topic, self.on_robot_stop, 10)
        self.done_pub = self.create_publisher(String, self.done_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.work_q: "queue.Queue[dict]" = queue.Queue()
        self.stop_event = threading.Event()
        self.shutdown_event = threading.Event()
        self.busy_lock = threading.Lock()
        self.busy = False
        self.stats = RuntimeStats()
        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        # OpenCV GUI is unstable when imshow() is called from the worker thread.
        # The worker only prepares frames; this timer displays them in the ROS spin thread.
        self.debug_timer = self.create_timer(0.10, self.debug_timer_cb)

        self.publish_status("ready", note="node started")
        self.get_logger().info(f"subscribed: {self.cmd_topic}, stop: {self.stop_topic}, done: {self.done_topic}")

    def log_mapper_config(self) -> None:
        self.get_logger().info(
            f"calibration mapper mode: {self.mapper.mode} "
            f"origin_uv=({self.mapper.fallback_origin_u:.1f},{self.mapper.fallback_origin_v:.1f}) "
            f"origin_xy=({self.mapper.fallback_origin_x:.1f},{self.mapper.fallback_origin_y:.1f}) "
            f"scale={self.mapper.fallback_mm_per_px:.4f} rot={self.mapper.dynamic_rotation_deg:.2f} "
            f"static_path={self.mapper.static_board_path}"
        )

    def param(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def on_robot_cmd(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        try:
            data = json.loads(raw)
            if not isinstance(data, dict):
                raise ValueError("JSON command must be an object")
        except Exception as exc:
            self.get_logger().error(f"invalid /robot/cmd JSON: {raw} / {exc}")
            self.publish_status("error", note="invalid robot cmd", raw=raw)
            return

        task = str(data.get("task", "fill")).lower()
        if task not in {"fill", "pack"}:
            self.get_logger().warning(f"unsupported task '{task}', command ignored")
            self.publish_status("ignored", task=task, note="unsupported task")
            return

        self.work_q.put(data)
        self.publish_status("queued", task=task, order_id=data.get("id", data.get("order_id")), queue_size=self.work_q.qsize())

    def on_robot_stop(self, msg: String) -> None:
        cmd = (msg.data or "").strip().upper()
        if cmd in {"STOP", "ESTOP", "E-STOP", "PAUSE"}:
            self.stop_event.set()
            self.publish_status("stopping", note=cmd)
            self.get_logger().warning(f"stop requested: {cmd}")
            try:
                self.robot.emergency_soft_stop()
            except Exception as exc:
                self.get_logger().warning(f"soft stop command failed: {exc}")
        elif cmd in {"START", "RESUME", "RESET"}:
            self.stop_event.clear()
            self.publish_status("resumed", note=cmd)
            self.get_logger().info(f"stop flag cleared: {cmd}")
        else:
            self.publish_status("stop_cmd_unknown", note=cmd)

    def worker_loop(self) -> None:
        while not self.shutdown_event.is_set():
            try:
                order = self.work_q.get(timeout=0.1)
            except queue.Empty:
                continue
            with self.busy_lock:
                self.busy = True
            try:
                self.execute_command(order)
            except Exception as exc:
                self.get_logger().error(f"order execution crashed: {exc}\n{traceback.format_exc()}")
                task = str(order.get("task", "fill")).lower()
                order_id = order.get("id", order.get("order_id"))
                self.publish_status("error", task=task, order_id=order_id, note=str(exc))
                if self.publish_failed_done and not self.stop_event.is_set():
                    self.publish_done(task, order_id, ok=False, note=str(exc))
            finally:
                with self.busy_lock:
                    self.busy = False
                self.work_q.task_done()

    def ensure_hardware_ready(self) -> None:
        self.robot.init_robot()
        self.camera.open()
        actual_w, actual_h = self.camera.get_size()
        self.camera_width = actual_w or self.camera_width
        self.camera_height = actual_h or self.camera_height
        msg = self.mapper.adjust_origin_for_image_size(self.camera_width, self.camera_height)
        if msg:
            self.get_logger().warning(msg)
            self.log_mapper_config()
        self.detector.load()

    def execute_command(self, order: dict) -> None:
        task = str(order.get("task", "fill")).lower()
        order_id = order.get("id", order.get("order_id"))
        self.stats.current_order_id = int(order_id) if order_id is not None else None
        self.stats.current_task = task

        if task == "pack":
            if self.pack_passthrough:
                self.publish_status("pack_passthrough", task="pack", order_id=order_id)
                self.publish_done("pack", order_id, ok=True, note="pack passthrough")
            else:
                self.publish_status("pack_ignored", task="pack", order_id=order_id)
            return

        counts = self.extract_counts(order)
        total = sum(counts.values())
        self.publish_status("start_fill", task="fill", order_id=order_id, counts=counts, total=total)
        if total <= 0:
            self.publish_done("fill", order_id, ok=True, counts=counts, delivered={"red": 0, "green": 0, "blue": 0})
            return

        if self.stop_event.is_set():
            self.publish_status("stopped_before_start", task="fill", order_id=order_id)
            return

        self.ensure_hardware_ready()
        if not self.robot.move_xyz(*self.observe_xyz, feed=self.robot_cfg.default_feed, timeout=12.0):
            raise RuntimeError(f"failed to move to observe pose {self.observe_xyz}")
        self.robot.wait_motion_done()

        delivered = {"red": 0, "green": 0, "blue": 0}
        cube_index = 0
        sequence = self.build_color_sequence(counts)
        self.get_logger().info(f"fill sequence for order {order_id}: {sequence}")

        for color in sequence:
            if self.stop_event.is_set():
                self.publish_status("stopped", task="fill", order_id=order_id, delivered=delivered)
                return

            if self.return_to_observe_after_each_cube:
                if not self.robot.move_xyz(*self.observe_xyz, feed=self.robot_cfg.default_feed, timeout=12.0):
                    self.publish_status("move_failed", task="fill", order_id=order_id, note="observe pose failed")
                    return
                self.robot.wait_motion_done()
                time.sleep(max(0.0, self.camera_settle_s))

            pick_xyz = self.find_pick_pose(color)
            place_xyz = self.compute_box_pose(cube_index)
            self.publish_status(
                "moving_cube",
                task="fill",
                order_id=order_id,
                color=color,
                pick_xyz=list(pick_xyz),
                place_xyz=list(place_xyz),
                cube_index=cube_index,
            )

            ok = self.robot.pick_and_place(
                pick_xyz,
                place_xyz,
                travel_z=self.travel_z,
                place_approach_z=self.box_approach_z,
                feed=self.robot_cfg.default_feed,
                n_points=self.motion_arc_points,
                should_stop=self.stop_event.is_set,
            )
            if not ok:
                self.publish_status("move_failed", task="fill", order_id=order_id, color=color, delivered=delivered)
                if self.publish_failed_done and not self.stop_event.is_set():
                    self.publish_done("fill", order_id, ok=False, counts=counts, delivered=delivered, note="move failed")
                return

            delivered[color] += 1
            cube_index += 1
            self.stats.delivered_red = delivered["red"]
            self.stats.delivered_green = delivered["green"]
            self.stats.delivered_blue = delivered["blue"]
            self.publish_status("cube_done", task="fill", order_id=order_id, color=color, delivered=delivered)

        if not self.robot.move_xyz(*self.observe_xyz, feed=self.robot_cfg.default_feed, timeout=12.0):
            self.get_logger().warning(f"final observe move failed: {self.observe_xyz}")
        self.robot.wait_motion_done()
        self.publish_done("fill", order_id, ok=True, counts=counts, delivered=delivered)
        self.publish_status("fill_done", task="fill", order_id=order_id, delivered=delivered)

    def extract_counts(self, order: dict) -> Dict[str, int]:
        result: Dict[str, int] = {}
        for color in COLOR_KEYS:
            value = order.get(color, order.get(color[0], 0))
            try:
                result[color] = max(0, int(value))
            except Exception:
                result[color] = 0
        return result

    def build_color_sequence(self, counts: Dict[str, int]) -> List[str]:
        seq: List[str] = []
        ordered_colors = [c for c in self.order_sequence if c in COLOR_KEYS]
        for color in ordered_colors:
            seq.extend([color] * int(counts.get(color, 0)))
        for color in COLOR_KEYS:
            if color not in ordered_colors:
                seq.extend([color] * int(counts.get(color, 0)))
        return seq

    def find_pick_pose(self, color: str) -> Vec3:
        det = self.detect_target(color)
        if det is not None:
            u, v = det.center
            x_raw, y_raw = self.mapper.pixel_to_base_xy(u, v)
            x_raw += self.pick_x_offset_mm
            y_raw += self.pick_y_offset_mm
            (x, y, z), changed = clamp_xyz_report(x_raw, y_raw, self.pick_z, self.robot_cfg.bounds)
            if changed:
                self.get_logger().warning(
                    f"computed pick pose {(x_raw, y_raw, self.pick_z)} clamped to {(x, y, z)}. "
                    f"Check fallback_mm_per_px/origin if this happens often. {self.mapper.debug_mapping_text(u, v)}"
                )
            self.get_logger().info(
                f"detected {color}: cls={det.cls}, conf={det.conf:.2f}, uv=({u:.1f},{v:.1f}) "
                f"-> pick={(x, y, z)}, mapper={self.mapper.mode}; {self.mapper.debug_mapping_text(u, v)}"
            )
            return (x, y, z)

        if self.allow_fixed_pick_fallback:
            fallback = self.fixed_pick_positions[color]
            self.get_logger().warning(f"YOLO did not find {color}; using fixed fallback pose {fallback}")
            return clamp_xyz_report(*fallback, self.robot_cfg.bounds)[0]

        raise RuntimeError(f"YOLO did not find requested cube color: {color}")

    def detect_target(self, color: str) -> Optional[Detection]:
        deadline = time.time() + max(0.1, self.detect_timeout_s)
        hits = 0
        best: Optional[Detection] = None
        last_detections: Sequence[Detection] = []

        while time.time() < deadline and not self.stop_event.is_set():
            frame = self.camera.read(flush=self.camera_flush_frames)
            h, w = frame.shape[:2]
            image_center = (w / 2.0, h / 2.0)
            detections = self.detector.detect(frame)
            last_detections = detections
            candidate = choose_best_detection(detections, color, image_center=image_center)
            if candidate is not None:
                hits += 1
                if best is None or candidate.conf > best.conf:
                    best = candidate
                self.prepare_debug_frame(frame, detections, requested_color=color, candidate=candidate)
                if hits >= max(1, self.detect_min_hits):
                    return candidate
            else:
                hits = 0
                self.prepare_debug_frame(frame, detections, requested_color=color, candidate=None)
            time.sleep(0.03)

        if best is not None:
            self.get_logger().warning(f"using best non-stable detection for {color}: {best.cls} {best.conf:.2f}")
            return best
        self.get_logger().warning(f"no detection for {color}. last detections={[d.cls for d in last_detections]}")
        return None

    def prepare_debug_frame(self, frame, detections: Sequence[Detection], *, requested_color: str = "", candidate: Optional[Detection] = None) -> None:
        if not self.debug_view and not self.debug_save_latest_frame:
            return
        try:
            view = draw_detections(frame, detections)
            h, w = view.shape[:2]
            mean = float(frame.mean()) if frame is not None else -1.0
            lines = [
                f"frame={w}x{h} mean={mean:.1f} req={requested_color} det={len(detections)}",
                f"origin_uv=({self.mapper.fallback_origin_u:.1f},{self.mapper.fallback_origin_v:.1f}) "
                f"origin_xy=({self.mapper.fallback_origin_x:.1f},{self.mapper.fallback_origin_y:.1f}) scale={self.mapper.fallback_mm_per_px:.3f}",
                f"workspace X[{self.robot_cfg.x_min:.0f},{self.robot_cfg.x_max:.0f}] "
                f"Y[{self.robot_cfg.y_min:.0f},{self.robot_cfg.y_max:.0f}] Z[{self.robot_cfg.z_min:.0f},{self.robot_cfg.z_max:.0f}]",
            ]
            if candidate is not None:
                u, v = candidate.center
                x, y = self.mapper.pixel_to_base_xy(u, v)
                x += self.pick_x_offset_mm
                y += self.pick_y_offset_mm
                lines.append(f"candidate {candidate.cls} conf={candidate.conf:.2f} {self.mapper.debug_mapping_text(u, v)} pick=({x:.1f},{y:.1f},{self.pick_z:.1f})")
            y0 = 26
            for line in lines:
                cv2.putText(view, line, (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
                y0 += 28
            # Draw dynamic origin crosshair if visible.
            ou, ov = int(self.mapper.fallback_origin_u), int(self.mapper.fallback_origin_v)
            if 0 <= ou < w and 0 <= ov < h:
                cv2.drawMarker(view, (ou, ov), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=24, thickness=2)

            if self.debug_save_latest_frame:
                now = time.time()
                if now - self._last_debug_save_t >= max(0.1, self.debug_save_interval_s):
                    cv2.imwrite(self.debug_frame_path, view)
                    self._last_debug_save_t = now

            if self.debug_view:
                with self._debug_frame_lock:
                    self._latest_debug_frame = view
        except Exception as exc:
            self.get_logger().warning(f"debug frame prepare failed: {exc}")

    def debug_timer_cb(self) -> None:
        if not self.debug_view:
            return
        try:
            with self._debug_frame_lock:
                frame = None if self._latest_debug_frame is None else self._latest_debug_frame.copy()
            if frame is None:
                return
            if not self._debug_window_created:
                cv2.namedWindow(self.debug_window, cv2.WINDOW_NORMAL)
                self._debug_window_created = True
            display = frame
            h, w = display.shape[:2]
            if self.debug_max_width > 0 and w > self.debug_max_width:
                scale = self.debug_max_width / float(w)
                display = cv2.resize(display, (self.debug_max_width, max(1, int(h * scale))), interpolation=cv2.INTER_AREA)
            cv2.imshow(self.debug_window, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                self.get_logger().info("debug window disabled by key")
                self.debug_view = False
                cv2.destroyWindow(self.debug_window)
        except Exception as exc:
            self.get_logger().warning(f"debug view failed: {exc}")
            self.debug_view = False

    def compute_box_pose(self, cube_index: int) -> Vec3:
        x, y, z = self.box_xyz
        if self.box_grid_enabled:
            cols = max(1, int(self.box_grid_cols))
            step = float(self.box_grid_step_mm)
            col = cube_index % cols
            row = cube_index // cols
            centered_col = col - (cols - 1) / 2.0
            if self.box_grid_axis == "x":
                x += centered_col * step
                y += row * step
            else:
                y += centered_col * step
                x += row * step
        pose, changed = clamp_xyz_report(x, y, z, self.robot_cfg.bounds)
        if changed:
            self.get_logger().warning(f"box pose {(x, y, z)} clamped to {pose}; verify box_xyz and workspace")
        return pose

    def publish_status(self, state: str, **kwargs) -> None:
        payload = {"state": state, **kwargs, "stamp": time.time()}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def publish_done(self, task: str, order_id, ok: bool = True, **kwargs) -> None:
        payload = {"task": task, "id": order_id, "order_id": order_id, "ok": bool(ok), **kwargs}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.done_pub.publish(msg)
        self.get_logger().info(f"published done: {msg.data}")

    def destroy_node(self):
        self.shutdown_event.set()
        try:
            self.robot.pump_off()
        except Exception:
            pass
        try:
            self.robot.close()
        except Exception:
            pass
        try:
            self.camera.close()
        except Exception:
            pass
        try:
            if self._debug_window_created:
                cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MirobotOrderDeliveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
