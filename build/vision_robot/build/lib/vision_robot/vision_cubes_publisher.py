#!/usr/bin/env python3
import os, time, json
import numpy as np
import cv2
import torch

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ====== 너 환경에 맞춰 조절 ======
TOP_CAM_ID = 4
YOLO_SIZE = 640
CONF_THRES_TOP = 0.5
IOU_THRES = 0.45

# 경로는 너 폴더 구조에 맞게 확인
STATIC_BOARD_PATH = os.path.expanduser("~/Mirobot_ros2/src/vision_robot/data/static_board_pose.npz")
YOLO_REPO = os.path.expanduser("~/yolov5")
YOLO_WEIGHT = os.path.expanduser("~/yolov5/runs/train/exp/weights/best.onnx")  # 너가 쓰는 weight 경로

USE_CLASSES = {"red-cube", "green-cube", "blue-cube"}
Z_PICK = 55.0


def load_yolo():
    model = torch.hub.load(YOLO_REPO, "custom", path=YOLO_WEIGHT, source="local")
    model.iou = IOU_THRES
    return model


def class_to_color_name(cls: str) -> str:
    if cls == "red-cube":
        return "red"
    if cls == "green-cube":
        return "green"
    if cls == "blue-cube":
        return "blue"
    return cls


def box_color_bgr(cls: str):
    # 화면 표시용 BGR 색
    if cls == "red-cube":
        return (0, 0, 255)
    if cls == "green-cube":
        return (0, 255, 0)
    if cls == "blue-cube":
        return (255, 0, 0)
    return (255, 255, 255)


class VisionCubesPublisher(Node):
    def __init__(self):
        super().__init__("vision_cubes_publisher")
        self.pub = self.create_publisher(String, "/vision/cubes", 10)

        if not os.path.exists(STATIC_BOARD_PATH):
            raise FileNotFoundError(f"static_board_pose.npz not found: {STATIC_BOARD_PATH}")

        pose = np.load(STATIC_BOARD_PATH)
        self.H_inv = pose["H_inv"]
        self.T_base_board = pose["T_base_board"]

        self.model = load_yolo()
        self.model.conf = CONF_THRES_TOP

        # 카메라
        self.cap = cv2.VideoCapture(TOP_CAM_ID, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError(f"camera open fail: /dev/video{TOP_CAM_ID}")

        self.timer = self.create_timer(0.10, self.tick)  # 10Hz

    def tick(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        h0, w0 = frame.shape[:2]

        # YOLO 입력(리사이즈)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_rgb_640 = cv2.resize(frame_rgb, (YOLO_SIZE, YOLO_SIZE), interpolation=cv2.INTER_LINEAR)

        results = self.model(frame_rgb_640)
        df = results.pandas().xyxy[0]

        sx = w0 / float(YOLO_SIZE)
        sy = h0 / float(YOLO_SIZE)

        cubes = []
        det_id = 0

        # ---- YOLO 결과를 원본 프레임에 그리기 ----
        for _, row in df.iterrows():
            conf = float(row["confidence"])
            if conf < CONF_THRES_TOP:
                continue

            cls_name = str(row["name"])
            if cls_name not in USE_CLASSES:
                continue

            xmin = float(row["xmin"]) * sx
            ymin = float(row["ymin"]) * sy
            xmax = float(row["xmax"]) * sx
            ymax = float(row["ymax"]) * sy

            x1, y1, x2, y2 = int(xmin), int(ymin), int(xmax), int(ymax)
            u = (xmin + xmax) * 0.5
            v = (ymin + ymax) * 0.5

            # === 이미지좌표 -> 보드좌표 -> 베이스좌표 ===
            pt_img = np.array([u, v, 1.0], dtype=np.float64).reshape(3, 1)
            pt_board_h = self.H_inv @ pt_img
            pt_board_h /= pt_board_h[2, 0]

            bx = pt_board_h[0, 0]
            by = pt_board_h[1, 0]
            bz = 0.0

            P_board = np.array([bx, by, bz, 1.0], dtype=np.float64)
            P_base = self.T_base_board @ P_board
            Xb, Yb, _Zb = P_base[:3]

            Xr = round(float(Xb))
            Yr = round(float(Yb))
            Zr = float(Z_PICK)

            cubes.append({
                "id": det_id,
                "color": class_to_color_name(cls_name),
                "x": float(Xr),
                "y": float(Yr),
                "z": float(Zr),
                "conf": conf
            })
            det_id += 1

            # ---- 화면 표시(박스/라벨) ----
            c = box_color_bgr(cls_name)
            cv2.rectangle(frame, (x1, y1), (x2, y2), c, 2)
            label = f"{cls_name} {conf:.2f}"
            cv2.putText(frame, label, (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)

        # ---- 카메라 창 표시 ----
        cv2.imshow("YOLO Camera", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):  # ESC or q
            rclpy.shutdown()
            return

        # ---- ROS 토픽 publish ----
        msg = String()
        msg.data = json.dumps({
            "frame": "master",
            "t": time.time(),
            "cubes": cubes
        }, ensure_ascii=False)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = None
    try:
        node = VisionCubesPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            try:
                node.cap.release()
            except Exception:
                pass
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

