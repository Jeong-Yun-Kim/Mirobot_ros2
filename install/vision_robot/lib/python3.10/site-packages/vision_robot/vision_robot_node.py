#!/usr/bin/env python3
import os
import cv2
import numpy as np
import torch
import math
import time
import serial
from typing import Tuple, List, Dict
from collections import deque
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

# =========================
# 타입/상수
# =========================
Vec3 = Tuple[float, float, float]
EOL = "\r\n"

# =========================
# 하드웨어/환경 설정 (고정)
# =========================
MIROBOT_PORT = "/dev/ttyUSB0"
MIROBOT_BAUD = 115200

TOP_CAM_ID = 4
SIDE_CAM_ID = 6

# 로봇 작업 범위
X_MIN, X_MAX = 140.0, 290.0
Y_MIN, Y_MAX = -270.0, 270.0
Z_MIN, Z_MAX = 40.0, 300.0

# =========================
# 파일 경로 (ROS2 share 기준)
# =========================
pkg_path = get_package_share_directory("vision_robot")
STATIC_BOARD_PATH = os.path.join(pkg_path, "data", "static_board_pose.npz")

# YOLO
YOLO_REPO = "/home/kjy/yolov5"
YOLO_WEIGHT = "/home/kjy/yolov5/runs/train/exp/weights/best.onnx"  # ONNX 사용

# YOLO 임계값
CONF_THRES_TOP = 0.5
CONF_THRES_SIDE = 0.25
IOU_THRES = 0.45
YOLO_SIZE = 640  # ✅ ONNX 입력 고정

# =========================
# Z 관련
# =========================
Z_PICK = 55.0
Z_OFFSET = 50.0
Z_TRAVEL = Z_PICK + Z_OFFSET

# =========================
# 구조/적층 관련 (phase2)
# =========================
CUBE_SIZE = 20.0
STACK_START = np.array([210.0, 0.0, 60.0])

USE_CLASSES = {"red-cube", "green-cube", "blue-cube"}

COLOR_PICK_POS = {
    "red-cube": (140.0, -90.0, Z_PICK),
    "green-cube": (140.0, 0.0, Z_PICK),
    "blue-cube": (140.0, 90.0, Z_PICK),
}

LAST_POS: Vec3 = None


# =========================
# 유틸
# =========================
def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def clamp_xyz(x: float, y: float, z: float) -> Vec3:
    x = clamp(x, X_MIN, X_MAX)
    y = clamp(y, Y_MIN, Y_MAX)
    z = clamp(z, Z_MIN, Z_MAX)
    return (x, y, z)


# =========================
# YOLO 로드
# =========================
def load_yolo():
    model = torch.hub.load(YOLO_REPO, "custom", path=YOLO_WEIGHT, source="local")
    model.iou = IOU_THRES
    return model


# =========================
# 시리얼/로봇 제어
# =========================
def open_try(port: str, baud: int):
    try:
        ser = serial.Serial(
            port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            write_timeout=1.5,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        try:
            ser.setDTR(False)
            ser.setRTS(False)
            time.sleep(0.05)
            ser.setDTR(True)
            ser.setRTS(True)
        except Exception:
            pass

        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.2)
        return ser
    except Exception as e:
        print(f"[open_fail] {port}@{baud}: {e}")
        return None


def rx_all(ser, delay=0.05):
    time.sleep(delay)
    n = ser.in_waiting
    return ser.read(n).decode(errors="ignore") if n else ""


def tx(ser, cmd: str):
    print("TX:", cmd)
    ser.write((cmd + EOL).encode())
    ser.flush()


def wait_ok(ser, timeout=3.0):
    t0 = time.time()
    buf = ""
    while time.time() - t0 < timeout:
        buf += rx_all(ser, delay=0.02)
        low = buf.lower()
        if "\nok" in ("\n" + low) or low.endswith("ok"):
            return True, buf
        if "error" in low or "alarm" in low or "lock" in low:
            print("RX(ERR):", buf.strip())
            return False, buf
    print("RX(TIMEOUT):", buf.strip())
    return False, buf


def robot_init_gcode(ser):
    rx_all(ser, 0.2)
    for c in ("M21", "M20", "G90"):
        tx(ser, c)
        _ = wait_ok(ser, 1.2)
    tx(ser, "M50")
    _ = wait_ok(ser, 2.0)


def pump_on(ser, pwm: int = 1000):
    cmd = f"M3S{int(pwm)}"
    tx(ser, cmd)
    ok, resp = wait_ok(ser, 2.0)
    if not ok:
        print("[WARN] pump_on 응답:", resp.strip())
    else:
        print("✅ Pump ON (", cmd, ")")


def pump_off(ser):
    cmd = "M3S0"
    tx(ser, cmd)
    ok, resp = wait_ok(ser, 2.0)
    if not ok:
        print("[WARN] pump_off 응답:", resp.strip())
    else:
        print("✅ Pump OFF")


def g1_move(ser, x: float, y: float, z: float, feed: float = 2000.0, timeout: float = 10.0) -> bool:
    x, y, z = clamp_xyz(x, y, z)
    cmd = f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F{feed:.1f}"
    tx(ser, cmd)
    ok, resp = wait_ok(ser, timeout)
    if not ok:
        print("[FAIL] 이동 실패:", resp.strip())
    return ok


# =========================
# 베지어 궤적
# =========================
def make_bezier_arc_xy_z_sort(
    p_start: Vec3,
    p_end: Vec3,
    *,
    h_min: float = 40.0,
    k: float = 0.3,
    z_min: float = 40.0,
    z_max: float = 300.0,
    margin: float = 10.0,
    n_points: int = 50,
) -> List[Vec3]:
    x1, y1, z1 = p_start
    x2, y2, z2 = p_end
    d = math.hypot(x2 - x1, y2 - y1)
    h = max(h_min, k * d)

    z_mid_raw = max(z1, z2) + h
    z_mid_clamped_high = min(z_mid_raw, z_max - margin)
    z_mid = max(z_mid_clamped_high, z_min + margin)

    xm = (x1 + x2) / 2.0
    ym = (y1 + y2) / 2.0

    P0 = (x1, y1, z1)
    P1 = (xm, ym, z_mid)
    P2 = (x2, y2, z2)

    path: List[Vec3] = []
    for i in range(n_points):
        t = i / (n_points - 1)
        s = 1.0 - t
        x = s * s * P0[0] + 2.0 * s * t * P1[0] + t * t * P2[0]
        y = s * s * P0[1] + 2.0 * s * t * P1[1] + t * t * P2[1]
        z = s * s * P0[2] + 2.0 * s * t * P1[2] + t * t * P2[2]
        path.append(clamp_xyz(x, y, z))
    return path


def make_bezier_arc_xy_z_build(
    p_start: Vec3,
    p_end: Vec3,
    *,
    h_min: float = 40.0,
    k: float = 0.3,
    z_min: float = 200.0,
    z_max: float = 300.0,
    margin: float = 10.0,
    n_points: int = 50,
) -> List[Vec3]:
    x1, y1, z1 = p_start
    x2, y2, z2 = p_end
    d = math.hypot(x2 - x1, y2 - y1)
    h = max(h_min, k * d)

    z_mid_raw = max(z1, z2) + h
    z_mid_clamped_high = min(z_mid_raw, z_max - margin)
    z_mid = max(z_mid_clamped_high, z_min + margin)

    xm = (x1 + x2) / 2.0
    ym = (y1 + y2) / 2.0

    P0 = (x1, y1, z1)
    P1 = (xm, ym, z_mid)
    P2 = (x2, y2, z2)

    path: List[Vec3] = []
    for i in range(n_points):
        t = i / (n_points - 1)
        s = 1.0 - t
        x = s * s * P0[0] + 2.0 * s * t * P1[0] + t * t * P2[0]
        y = s * s * P0[1] + 2.0 * s * t * P1[1] + t * t * P2[1]
        z = s * s * P0[2] + 2.0 * s * t * P1[2] + t * t * P2[2]
        path.append(clamp_xyz(x, y, z))
    return path


# =========================
# 픽업/드랍 (phase1)
# =========================
def move_with_pump_between_points_sort(p_pick: Vec3, p_place: Vec3, *, feed: float = 2000.0, n_points: int = 50) -> bool:
    x1, y1, _ = p_pick
    x2, y2, z2 = p_place

    p_top_start = clamp_xyz(x1, y1, Z_TRAVEL)
    z_travel_place = clamp(z2 + Z_OFFSET, Z_MIN, Z_MAX)
    p_top_end = clamp_xyz(x2, y2, z_travel_place)

    path_top = make_bezier_arc_xy_z_sort(p_top_start, p_top_end, n_points=n_points)

    ser = open_try(MIROBOT_PORT, MIROBOT_BAUD)
    if not ser:
        print("[ERROR] 미로봇 시리얼 연결 실패")
        return False

    try:
        robot_init_gcode(ser)

        if not g1_move(ser, *p_top_start, feed=feed, timeout=10.0):
            return False
        if not g1_move(ser, x1, y1, Z_PICK, feed=feed, timeout=10.0):
            return False

        pump_on(ser)
        time.sleep(1.5)

        if not g1_move(ser, x1, y1, Z_TRAVEL, feed=feed, timeout=10.0):
            return False

        for (x, y, z) in path_top[1:]:
            if not g1_move(ser, x, y, z, feed=feed, timeout=8.0):
                return False
            time.sleep(0.01)

        if not g1_move(ser, x2, y2, z2, feed=feed, timeout=10.0):
            return False

        time.sleep(0.5)
        pump_off(ser)

        if not g1_move(ser, x2, y2, z_travel_place, feed=feed, timeout=10.0):
            return False

        try:
            tx(ser, "M400")
            _ = wait_ok(ser, 10.0)
        except Exception:
            pass

        print("✅ 픽업 + 이동 + 드랍 완료")
        return True

    finally:
        try:
            ser.close()
        except Exception:
            pass


# =========================
# 픽업/드랍 (phase2)
# =========================
def move_with_pump_between_points_build(p_start: Vec3, p_end: Vec3, *, feed: float = 2000.0, n_points: int = 50) -> bool:
    global LAST_POS

    x_pick, y_pick, z_pick = p_start
    x_place, y_place, z_place = p_end

    pick_lift_z = min(z_pick + Z_OFFSET, Z_MAX)
    place_lift_z = min(z_place + Z_OFFSET, Z_MAX)

    pick_lift = clamp_xyz(x_pick, y_pick, pick_lift_z)
    place_lift = clamp_xyz(x_place, y_place, place_lift_z)

    ser = open_try(MIROBOT_PORT, MIROBOT_BAUD)
    if not ser:
        print("[ERROR] 미로봇 시리얼 연결 실패")
        return False

    try:
        robot_init_gcode(ser)

        if LAST_POS is None:
            if not g1_move(ser, *pick_lift, feed=feed, timeout=10.0):
                return False
        else:
            path_to_pick = make_bezier_arc_xy_z_build(LAST_POS, pick_lift, n_points=n_points)
            for (x, y, z) in path_to_pick:
                if not g1_move(ser, x, y, z, feed=feed, timeout=8.0):
                    return False
                time.sleep(0.01)

        try:
            tx(ser, "M400")
            _ = wait_ok(ser, 10.0)
        except Exception:
            pass

        if not g1_move(ser, x_pick, y_pick, z_pick, feed=feed, timeout=8.0):
            return False

        try:
            tx(ser, "M400")
            _ = wait_ok(ser, 10.0)
        except Exception:
            pass

        pump_on(ser)
        time.sleep(1.5)

        if not g1_move(ser, x_pick, y_pick, pick_lift_z, feed=feed, timeout=8.0):
            return False

        try:
            tx(ser, "M400")
            _ = wait_ok(ser, 10.0)
        except Exception:
            pass

        path_pick_to_place = make_bezier_arc_xy_z_build(pick_lift, place_lift, n_points=n_points)
        for (x, y, z) in path_pick_to_place:
            if not g1_move(ser, x, y, z, feed=feed, timeout=8.0):
                return False
            time.sleep(0.01)

        try:
            tx(ser, "M400")
            _ = wait_ok(ser, 10.0)
        except Exception:
            pass

        if not g1_move(ser, x_place, y_place, z_place, feed=feed, timeout=8.0):
            return False

        try:
            tx(ser, "M400")
            _ = wait_ok(ser, 10.0)
        except Exception:
            pass

        time.sleep(0.5)
        pump_off(ser)

        _ = g1_move(ser, x_place, y_place, place_lift_z, feed=feed, timeout=8.0)

        try:
            tx(ser, "M400")
            _ = wait_ok(ser, 10.0)
        except Exception:
            pass

        LAST_POS = place_lift
        print("✅ phase2 한 큐브 이동 완료, LAST_POS =", LAST_POS)
        return True

    finally:
        try:
            ser.close()
        except Exception:
            pass


# =========================
# phase2 구조 인식 (YOLO)
# =========================
def detect_yolo_objects(frame_bgr, model):
    """
    관계 계산(centers)은 640기준으로 해도 OK.
    다만 시각화가 필요하면 draw할 때 스케일링해주면 됨.
    """
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    frame_rgb_640 = cv2.resize(frame_rgb, (YOLO_SIZE, YOLO_SIZE), interpolation=cv2.INTER_LINEAR)

    results = model(frame_rgb_640)
    df = results.pandas().xyxy[0]

    detections = []
    for _, row in df.iterrows():
        conf = float(row["confidence"])
        if conf < CONF_THRES_SIDE:
            continue
        cls = row["name"]
        xmin, ymin = float(row["xmin"]), float(row["ymin"])
        xmax, ymax = float(row["xmax"]), float(row["ymax"])
        detections.append({"cls": cls, "bbox": (xmin, ymin, xmax, ymax)})

    return detections


def extract_structure_relations(detections):
    X_THRESH, Y_THRESH = 25.0, 25.0
    centers = {}

    for det in detections:
        cls = det["cls"]
        if cls not in USE_CLASSES:
            continue
        xmin, ymin, xmax, ymax = det["bbox"]
        cx = (xmin + xmax) / 2.0
        cy = (ymin + ymax) / 2.0
        centers[cls] = (cx, cy)

    labels = list(centers.keys())
    relations = set()

    for A in labels:
        xA, yA = centers[A]
        best_B = None
        best_dx = None
        for B in labels:
            if B == A:
                continue
            xB, yB = centers[B]
            if abs(yA - yB) > Y_THRESH:
                continue
            dx = xB - xA
            if dx <= 0:
                continue
            if best_dx is None or dx < best_dx:
                best_dx = dx
                best_B = B
        if best_B is not None:
            relations.add(f"{A}-right-{best_B}")

    for A in labels:
        xA, yA = centers[A]
        best_B = None
        best_dy = None
        for B in labels:
            if B == A:
                continue
            xB, yB = centers[B]
            if abs(xA - xB) > X_THRESH:
                continue
            dy = yB - yA
            if dy <= 0:
                continue
            if best_dy is None or dy < best_dy:
                best_dy = dy
                best_B = B
        if best_B is not None:
            relations.add(f"{A}-top-{best_B}")

    return list(relations), centers


def compute_positions(relations):
    if not relations:
        return {}

    nodes = set()
    edges = []
    top_nodes = set()

    for rel in relations:
        if "-top-" in rel:
            A, B = rel.split("-top-")
            direction = "top"
            top_nodes.add(A)
        elif "-right-" in rel:
            A, B = rel.split("-right-")
            direction = "right"
        else:
            continue
        nodes.add(A)
        nodes.add(B)
        edges.append((A, direction, B))

    if not nodes:
        return {}

    graph = {n: [] for n in nodes}
    for A, direction, B in edges:
        if direction == "top":
            graph[A].append(("bottom", B))
            graph[B].append(("top", A))
        elif direction == "right":
            graph[A].append(("right", B))
            graph[B].append(("left", A))

    bottom_candidates = nodes - top_nodes
    root = sorted(bottom_candidates)[0] if bottom_candidates else sorted(nodes)[0]

    positions = {root: STACK_START.copy()}
    q = deque([root])

    while q:
        cur = q.popleft()
        cur_pos = positions[cur]

        for direction, nb in graph[cur]:
            if nb in positions:
                continue
            if direction == "top":
                offset = np.array([0.0, 0.0, CUBE_SIZE])
            elif direction == "bottom":
                offset = np.array([0.0, 0.0, -CUBE_SIZE])
            elif direction == "right":
                offset = np.array([CUBE_SIZE, 0.0, 0.0])
            elif direction == "left":
                offset = np.array([-CUBE_SIZE, 0.0, 0.0])
            else:
                continue
            positions[nb] = cur_pos + offset
            q.append(nb)

    return positions


def print_all_cube_positions(centers, positions):
    print("\n===== ALL CUBES =====")
    if not centers and not positions:
        print("  (no cubes)")
        print("=====================\n")
        return

    all_names = set(centers.keys()) | set(positions.keys())
    for name in sorted(all_names):
        img_pos = centers.get(name)
        world_pos = positions.get(name)

        line = f"  {name}: "
        line += f"img_center=(u={img_pos[0]:.1f}, v={img_pos[1]:.1f}) " if img_pos is not None else "img_center=(none) "
        line += f"world_pos=({world_pos[0]:.1f}, {world_pos[1]:.1f}, {world_pos[2]:.1f})" if world_pos is not None else "world_pos=(not connected)"
        print(line)
    print("=====================\n")


def draw_detections(frame_bgr, detections):
    """
    detections bbox가 640 기준이므로, 원본 프레임에 그릴 때 스케일링.
    """
    h0, w0 = frame_bgr.shape[:2]
    sx = w0 / float(YOLO_SIZE)
    sy = h0 / float(YOLO_SIZE)

    show = frame_bgr.copy()
    for det in detections:
        xmin, ymin, xmax, ymax = det["bbox"]
        cls = det["cls"]

        x1 = int(xmin * sx)
        y1 = int(ymin * sy)
        x2 = int(xmax * sx)
        y2 = int(ymax * sy)

        cv2.rectangle(show, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(show, cls, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return show


def build_with_robot(positions: Dict[str, np.ndarray]):
    tasks = []
    for name, pos in positions.items():
        if name not in COLOR_PICK_POS:
            print(f"[WARN] '{name}' 픽업 위치 미정 → 건너뜀")
            continue
        src = COLOR_PICK_POS[name]
        dst = (float(pos[0]), float(pos[1]), float(pos[2]))
        tasks.append((name, src, dst))

    if not tasks:
        print("[INFO] 실행할 이동 작업이 없습니다.")
        return

    tasks.sort(key=lambda t: t[2][2])

    for name, src, dst in tasks:
        print(f"[MOVE] {name}: {src} -> {dst}")
        ok = move_with_pump_between_points_build(src, dst, feed=2000.0, n_points=50)
        if not ok:
            print(f"[ERROR] {name} 이동 실패, 이후 작업 중단")
            break


# =========================
# phase1: 상부 카메라 정리
# =========================
def run_phase1_sort(model, H_inv, T_base_board) -> bool:
    cap = cv2.VideoCapture(TOP_CAM_ID, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"[ERROR] 위 카메라({TOP_CAM_ID})를 열 수 없습니다. (/dev/video{TOP_CAM_ID})")
        return False

    model.conf = CONF_THRES_TOP
    detections = []
    need_detect = False

    print("[PHASE1] 시작: q=종료, w=YOLO 감지, e=자동 픽업/정리")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        show = frame.copy()

        if need_detect:
            h0, w0 = frame.shape[:2]

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_rgb_640 = cv2.resize(frame_rgb, (YOLO_SIZE, YOLO_SIZE), interpolation=cv2.INTER_LINEAR)

            results = model(frame_rgb_640)
            df = results.pandas().xyxy[0]

            sx = w0 / float(YOLO_SIZE)
            sy = h0 / float(YOLO_SIZE)

            detections = []
            for _, row in df.iterrows():
                conf = float(row["confidence"])
                if conf < CONF_THRES_TOP:
                    continue

                cls_name = row["name"]

                xmin = float(row["xmin"]) * sx
                ymin = float(row["ymin"]) * sy
                xmax = float(row["xmax"]) * sx
                ymax = float(row["ymax"]) * sy

                u = (xmin + xmax) * 0.5
                v = (ymin + ymax) * 0.5

                pt_img = np.array([u, v, 1.0], dtype=np.float64).reshape(3, 1)
                pt_board_h = H_inv @ pt_img
                pt_board_h /= pt_board_h[2, 0]
                bx = pt_board_h[0, 0]
                by = pt_board_h[1, 0]
                bz = 0.0

                P_board = np.array([bx, by, bz, 1.0], dtype=np.float64)
                P_base = T_base_board @ P_board
                Xb, Yb, Zb = P_base[:3]

                Xr = round(float(Xb))
                Yr = round(float(Yb))
                Zr = Z_PICK

                detections.append({
                    "cls": cls_name,
                    "conf": conf,
                    "X": float(Xb),
                    "Y": float(Yb),
                    "Z": float(Zb),
                    "Xr": Xr,
                    "Yr": Yr,
                    "Zr": Zr,
                    "bbox": (xmin, ymin, xmax, ymax),
                })

            if detections:
                print("\n[DETECT] 감지된 물체:")
                for i, det in enumerate(detections):
                    dist_xy = math.hypot(det["Xr"], det["Yr"])
                    print(
                        f"  [{i}] {det['cls']}  "
                        f"base(mm) 실수: X={det['X']:.2f}, Y={det['Y']:.2f}, Z={det['Z']:.2f}  "
                        f"반올림+Z고정: X={det['Xr']}, Y={det['Yr']}, Z={det['Zr']}  "
                        f"conf={det['conf']:.2f}  dist={dist_xy:.1f}mm"
                    )
                print("→ e 키를 누르면 자동 픽업·정리합니다.")
            else:
                print("[DETECT] 조건을 만족하는 물체 없음")

            need_detect = False

        for det in detections:
            xmin, ymin, xmax, ymax = det["bbox"]
            cv2.rectangle(show, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (255, 255, 255), 2)
            cv2.putText(
                show,
                f"{det['cls']} {det['conf']:.2f}",
                (int(xmin), int(ymin) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

        cv2.imshow("phase1_topcam_sort", show)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            cap.release()
            cv2.destroyAllWindows()
            return False
        elif key == ord("w"):
            print("[PHASE1] 현재 프레임으로 YOLO 감지 시도...")
            need_detect = True
        elif key == ord("e"):
            if not detections:
                print("[PHASE1] 아직 감지 결과 없음. 먼저 w 키로 감지하세요.")
            else:
                drop_xy_map = {
                    "red-cube": (140.0, -90.0),
                    "green-cube": (140.0, 0.0),
                    "blue-cube": (140.0, 90.0),
                }

                stack_counts = {}
                ref_x, ref_y = 0.0, 0.0
                remaining = detections.copy()
                total = len(remaining)
                print(f"\n[PHASE1] 총 {total}개 물체를 처리합니다.")

                move_idx = 1
                while remaining:
                    best_i = None
                    best_d = float("inf")
                    for i, det in enumerate(remaining):
                        dx = det["Xr"] - ref_x
                        dy = det["Yr"] - ref_y
                        d = math.hypot(dx, dy)
                        if d < best_d:
                            best_d = d
                            best_i = i

                    target = remaining.pop(best_i)
                    cls_name = target["cls"]

                    if cls_name not in drop_xy_map:
                        print(f"[WARN] 지원하지 않는 클래스 '{cls_name}' → 스킵")
                        continue

                    base_x, base_y = drop_xy_map[cls_name]
                    count_same = stack_counts.get(cls_name, 0)

                    STACK_DZ = 28.0
                    z_place = Z_PICK + STACK_DZ * count_same
                    stack_counts[cls_name] = count_same + 1

                    print(
                        f"\n[MOVE] {move_idx}/{total}번째: {cls_name} "
                        f"(픽업: Xr={target['Xr']}, Yr={target['Yr']} → "
                        f"드랍: ({base_x}, {base_y}, {z_place}), "
                        f"거리={best_d:.1f}mm)"
                    )

                    x1 = target["Xr"]
                    y1 = target["Yr"] + 15  # 기존 Y 보정 유지
                    p_pick = (x1, y1, Z_PICK)
                    p_place = (base_x, base_y, z_place)

                    ok = move_with_pump_between_points_sort(p_pick, p_place, feed=2000.0, n_points=50)
                    if not ok:
                        print("[WARN] 이동 실패 → 이후 작업 중단")
                        break

                    ref_x, ref_y = base_x, base_y
                    move_idx += 1

                detections = []
                print("[PHASE1] 정리 완료. phase2로 넘어갑니다.")
                break

    cap.release()
    cv2.destroyAllWindows()
    return True


# =========================
# phase2: 측면 카메라 구조 인식/적층
# =========================
def run_phase2_build(model):
    global LAST_POS
    LAST_POS = None

    cap = cv2.VideoCapture(SIDE_CAM_ID, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"[ERROR] 옆 카메라({SIDE_CAM_ID})를 열 수 없습니다. (/dev/video{SIDE_CAM_ID})")
        return

    detections = []
    relations = []
    positions = {}
    centers = {}
    need_detect = False

    model.conf = CONF_THRES_SIDE
    print("[PHASE2] q: 종료, s: YOLO 감지, d: 마지막 감지 결과로 로봇 동작")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if need_detect:
            print("[PHASE2] YOLO 감지 실행")
            detections = detect_yolo_objects(frame, model)
            print(f"[INFO] 감지된 개수(필터 통과 후): {len(detections)}")

            relations, centers = extract_structure_relations(detections)
            print("Relations:", relations)

            positions = compute_positions(relations)
            print("Positions dict(keys):", positions.keys())

            print_all_cube_positions(centers, positions)
            need_detect = False

        show = draw_detections(frame, detections) if detections else frame
        cv2.imshow("phase2_sidecam_structure_detect", show)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("s"):
            print("[PHASE2] YOLO 감지 요청 (s)")
            need_detect = True
        elif key == ord("d"):
            if positions:
                print("[PHASE2] d 입력: 마지막 구조로 로봇 동작 시작")
                build_with_robot(positions)
                print("[PHASE2] 로봇 동작 완료")
            else:
                print("[PHASE2] 아직 유효한 감지 결과가 없습니다. 먼저 s로 감지하세요.")

    cap.release()
    cv2.destroyAllWindows()


# =========================
# main
# =========================
def main(args=None):
    rclpy.init(args=args)
    node = Node("vision_robot_node")
    node.get_logger().info("[ROS2] Vision Robot 노드 시작. YOLO 모델 로드 중...")

    if not os.path.exists(STATIC_BOARD_PATH):
        node.get_logger().error(f"[ROS2] static_board_pose.npz 가 없습니다: {STATIC_BOARD_PATH}")
        node.get_logger().error("data/static_board_pose.npz 경로에 보정 파일을 넣고 다시 실행하세요.")
        node.destroy_node()
        rclpy.shutdown()
        return

    pose = np.load(STATIC_BOARD_PATH)
    if "H_inv" not in pose.files or "T_base_board" not in pose.files:
        node.get_logger().error(f"[ROS2] static_board_pose.npz 키가 맞지 않습니다. keys={pose.files}")
        node.get_logger().error("npz 안에 반드시 H_inv, T_base_board가 있어야 합니다.")
        node.destroy_node()
        rclpy.shutdown()
        return

    H_inv = pose["H_inv"]
    T_base_board = pose["T_base_board"]

    model = load_yolo()
    node.get_logger().info("[ROS2] YOLO 모델 로드 완료.")

    node.get_logger().info("[ROS2] Phase1 시작...")
    phase1_ok = run_phase1_sort(model, H_inv, T_base_board)
    if not phase1_ok:
        node.get_logger().warn("[ROS2] Phase1이 완료되지 않아 종료합니다.")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("[ROS2] Phase2 시작...")
    run_phase2_build(model)

    node.get_logger().info("[ROS2] 종료합니다.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

