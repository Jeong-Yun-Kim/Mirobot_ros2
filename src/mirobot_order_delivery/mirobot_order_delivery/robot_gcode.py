from __future__ import annotations

import re
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import serial

from .geometry import Bounds, Vec3, clamp, clamp_xyz_report, make_bezier_arc

EOL_DEFAULT = "\r\n"

RE_STATUS = re.compile(
    r"Cartesian coordinate\(XYZ RxRyRz\):"
    r"\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*,\s*([-\d\.]+)",
    re.IGNORECASE,
)


def parse_state_token(raw: str) -> str:
    text = (raw or "").strip()
    if text.startswith("<"):
        return text[1:].split(",", 1)[0].strip()
    return "UNKNOWN"


@dataclass
class RobotConfig:
    port: str = "/dev/ttyUSB0"
    baud: int = 115200
    eol: str = EOL_DEFAULT
    x_min: float = 140.0
    x_max: float = 290.0
    y_min: float = -270.0
    y_max: float = 270.0
    z_min: float = 40.0
    z_max: float = 300.0
    default_feed: float = 2000.0
    pump_pwm: int = 1000
    pump_dwell_s: float = 1.5
    drop_dwell_s: float = 0.4
    command_timeout_s: float = 12.0
    toggle_dtr_rts: bool = True
    verbose_tx: bool = True
    use_m400: bool = False
    use_status_wait: bool = False
    motion_settle_s: float = 0.20
    status_wait_timeout_s: float = 1.0

    @property
    def bounds(self) -> Bounds:
        return (self.x_min, self.x_max, self.y_min, self.y_max, self.z_min, self.z_max)


class MirobotGCode:
    """Small serial G-code driver for the WLKATA Mirobot.

    M400 is disabled by default because your firmware returned
    `Error,E000,Invalid gcode ID:23`.  The old code only warned, but the warning
    made debugging noisy.  If your firmware later supports M400, set
    motion.use_m400=true in the YAML.
    """

    def __init__(self, cfg: RobotConfig, logger=None):
        self.cfg = cfg
        self.logger = logger
        self.ser: Optional[serial.Serial] = None

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

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self) -> None:
        if self.connected:
            return
        self.ser = serial.Serial(
            self.cfg.port,
            baudrate=int(self.cfg.baud),
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            write_timeout=1.5,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        if self.cfg.toggle_dtr_rts:
            try:
                self.ser.setDTR(False)
                self.ser.setRTS(False)
                time.sleep(0.05)
                self.ser.setDTR(True)
                self.ser.setRTS(True)
            except Exception:
                pass
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(0.2)
        self.log("info", f"Mirobot serial connected: {self.cfg.port}@{self.cfg.baud}")

    def close(self) -> None:
        try:
            if self.ser is not None:
                self.ser.close()
        finally:
            self.ser = None

    def rx_all(self, delay: float = 0.05) -> str:
        if self.ser is None:
            return ""
        time.sleep(delay)
        n = self.ser.in_waiting
        return self.ser.read(n).decode(errors="ignore") if n else ""

    def tx(self, cmd: str, *, log: bool = True) -> None:
        if self.ser is None:
            raise RuntimeError("serial is not connected")
        if log and self.cfg.verbose_tx:
            self.log("info", f"TX: {cmd}")
        self.ser.write((cmd + self.cfg.eol).encode("utf-8"))
        self.ser.flush()

    def wait_ok(self, timeout: Optional[float] = None) -> Tuple[bool, str]:
        timeout = float(timeout if timeout is not None else self.cfg.command_timeout_s)
        t0 = time.time()
        buf = ""
        while time.time() - t0 < timeout:
            buf += self.rx_all(delay=0.02)
            low = buf.lower()
            if "\nok" in ("\n" + low) or low.endswith("ok") or "ok\r" in low or "ok\n" in low:
                return True, buf
            if "error" in low or "alarm" in low or "lock" in low:
                return False, buf
        return False, buf

    def send_and_wait(self, cmd: str, timeout: Optional[float] = None, *, warn_only: bool = False) -> bool:
        self.tx(cmd)
        ok, resp = self.wait_ok(timeout)
        if not ok:
            text = resp.strip().replace("\n", " | ")
            if warn_only:
                self.log("warn", f"command response not ok: {cmd} / {text}")
            else:
                self.log("error", f"command failed: {cmd} / {text}")
        return ok

    def init_robot(self) -> None:
        self.connect()
        _ = self.rx_all(0.2)
        for cmd in ("M21", "M20", "G90"):
            self.send_and_wait(cmd, 1.5, warn_only=True)
        self.send_and_wait("M50", 2.0, warn_only=True)

    def wait_motion_done(self, timeout: Optional[float] = None) -> None:
        timeout = float(timeout if timeout is not None else self.cfg.status_wait_timeout_s)
        if self.cfg.use_m400:
            self.send_and_wait("M400", timeout, warn_only=True)
            return

        if self.cfg.use_status_wait:
            t0 = time.time()
            got_status = False
            while time.time() - t0 < timeout:
                ok, state, _xyz, _rpy, _raw = self.query_status_pose(log_tx=False)
                if ok:
                    got_status = True
                    if str(state).strip().lower() == "idle":
                        time.sleep(max(0.0, float(self.cfg.motion_settle_s)))
                        return
                time.sleep(0.10)
            if got_status:
                self.log("warn", f"status wait timed out after {timeout:.1f}s")

        time.sleep(max(0.0, float(self.cfg.motion_settle_s)))

    def move_xyz(self, x: float, y: float, z: float, *, feed: Optional[float] = None, timeout: Optional[float] = None) -> bool:
        raw = (float(x), float(y), float(z))
        (x, y, z), changed = clamp_xyz_report(raw[0], raw[1], raw[2], self.cfg.bounds)
        if changed:
            self.log("warn", f"requested pose {raw} was clamped to {(x, y, z)} by workspace {self.cfg.bounds}")
        feed = float(feed if feed is not None else self.cfg.default_feed)
        cmd = f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F{feed:.1f}"
        return self.send_and_wait(cmd, timeout or self.cfg.command_timeout_s)

    def pump_on(self) -> bool:
        return self.send_and_wait(f"M3S{int(self.cfg.pump_pwm)}", 2.0, warn_only=True)

    def pump_off(self) -> bool:
        return self.send_and_wait("M3S0", 2.0, warn_only=True)

    def emergency_soft_stop(self) -> None:
        try:
            self.pump_off()
        except Exception:
            pass
        try:
            self.send_and_wait("M0", 1.0, warn_only=True)
        except Exception:
            pass

    def query_status_pose(self, *, log_tx: bool = True) -> Tuple[bool, str, Optional[Vec3], Optional[Vec3], str]:
        if self.ser is None:
            return False, "SERIAL_OFF", None, None, ""
        try:
            self.ser.reset_input_buffer()
            self.tx("?", log=log_tx)
            raw = b""
            t0 = time.time()
            while time.time() - t0 < 0.6:
                n = self.ser.in_waiting
                if n:
                    raw += self.ser.read(n)
                    if b">" in raw:
                        break
                else:
                    time.sleep(0.01)
            text = raw.decode(errors="ignore")
            state = parse_state_token(text)
            m = RE_STATUS.search(text)
            if not m:
                return False, state, None, None, text
            x, y, z, rx, ry, rz = map(float, m.groups())
            return True, state, (x, y, z), (rx, ry, rz), text
        except Exception as exc:
            return False, "EXC", None, None, str(exc)

    def pick_and_place(
        self,
        pick_xyz: Vec3,
        place_xyz: Vec3,
        *,
        travel_z: float,
        place_approach_z: Optional[float] = None,
        feed: Optional[float] = None,
        n_points: int = 30,
        should_stop: Callable[[], bool] | None = None,
    ) -> bool:
        should_stop = should_stop or (lambda: False)
        feed = float(feed if feed is not None else self.cfg.default_feed)
        (x_pick, y_pick, z_pick), pick_changed = clamp_xyz_report(*pick_xyz, self.cfg.bounds)
        (x_place, y_place, z_place), place_changed = clamp_xyz_report(*place_xyz, self.cfg.bounds)
        if pick_changed:
            self.log("warn", f"pick pose {pick_xyz} clamped to {(x_pick, y_pick, z_pick)}")
        if place_changed:
            self.log("warn", f"place pose {place_xyz} clamped to {(x_place, y_place, z_place)}")

        travel_z_pick = clamp(float(travel_z), self.cfg.z_min, self.cfg.z_max)
        travel_z_place = clamp(float(place_approach_z if place_approach_z is not None else travel_z), self.cfg.z_min, self.cfg.z_max)

        p_pick_top = (x_pick, y_pick, travel_z_pick)
        p_place_top = (x_place, y_place, travel_z_place)

        def guarded_move(p: Vec3, timeout_s: float = 12.0) -> bool:
            if should_stop():
                self.log("warn", "stop requested before move")
                return False
            return self.move_xyz(*p, feed=feed, timeout=timeout_s)

        if not guarded_move(p_pick_top):
            return False
        self.wait_motion_done()
        if not guarded_move((x_pick, y_pick, z_pick), timeout_s=8.0):
            return False
        self.wait_motion_done()
        if should_stop():
            return False
        self.pump_on()
        time.sleep(max(0.0, float(self.cfg.pump_dwell_s)))

        if not guarded_move(p_pick_top, timeout_s=8.0):
            self.pump_off()
            return False
        self.wait_motion_done()

        path = make_bezier_arc(
            p_pick_top,
            p_place_top,
            bounds=self.cfg.bounds,
            h_min=40.0,
            k=0.3,
            z_mid_min=travel_z_pick,
            n_points=n_points,
        )
        for p in path[1:]:
            if not guarded_move(p, timeout_s=8.0):
                self.pump_off()
                return False
            time.sleep(0.005)
        self.wait_motion_done()

        if not guarded_move((x_place, y_place, z_place), timeout_s=8.0):
            self.pump_off()
            return False
        self.wait_motion_done()
        time.sleep(max(0.0, float(self.cfg.drop_dwell_s)))
        self.pump_off()
        time.sleep(0.1)

        ok = guarded_move(p_place_top, timeout_s=8.0)
        self.wait_motion_done()
        return ok
