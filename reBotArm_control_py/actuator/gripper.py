"""夹爪控制句柄，直接基于 motorbridge SDK。

支持使能、失能、零点设置、MIT/POS_VEL/VEL 控制模式及状态反馈。
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import yaml

from motorbridge import Controller, Mode, CallError


@dataclass
class GripperCfg:
    name: str
    motor_id: int
    feedback_id: int
    model: str
    vendor: str = "damiao"
    kp: float = 18.0
    kd: float = 2.0
    vel_kp: float = 0.0008
    vel_ki: float = 0.002
    pos_kp: float = 50.0
    pos_ki: float = 1.0
    vlim: float = 3.0


def load_cfg(path: str) -> dict:
    with open(Path(path), "r") as f:
        data = yaml.safe_load(f)

    gc = data.get("gripper", [{}])[0]
    mc = gc.get("MIT", {})
    pc = gc.get("POS_VEL", {})
    return {
        "channel": data.get("channel", "/dev/ttyACM0"),
        "gripper": GripperCfg(
            name=gc["name"],
            motor_id=int(gc["motor_id"]),
            feedback_id=int(gc["feedback_id"]),
            model=str(gc.get("model", "4310")),
            vendor=str(gc.get("vendor", "damiao")).lower(),
            kp=float(mc.get("kp", 18.0)),
            kd=float(mc.get("kd", 2.0)),
            vel_kp=float(pc.get("vel_kp", 0.0008)),
            vel_ki=float(pc.get("vel_ki", 0.002)),
            pos_kp=float(pc.get("pos_kp", 50.0)),
            pos_ki=float(pc.get("pos_ki", 1.0)),
            vlim=float(pc.get("vlim", 3.0)),
        ),
    }


class Gripper:
    """夹爪控制句柄，直接持有 motorbridge.Controller。"""

    def __init__(self, cfg_path: Optional[str] = None) -> None:
        if cfg_path is None:
            cfg_path = Path(__file__).parent.parent.parent / "config" / "gripper.yaml"
        cfg = load_cfg(str(cfg_path))

        self._channel = cfg["channel"]
        self._cfg = cfg["gripper"]
        self._mode = "mit"
        self._mit_kp = self._cfg.kp
        self._mit_kd = self._cfg.kd

        self._ctrl: Optional[Controller] = None
        self._mot = None
        self._setup_motor()

        self._loop_thread: Optional[threading.Thread] = None
        self._loop_running = False
        self._loop_stop = threading.Event()
        self._rate = 100.0

    def _make_controller(self) -> Controller:
        if self._channel.startswith("/dev/tty"):
            return Controller.from_dm_serial(self._channel, 921600)
        return Controller(self._channel)

    def _setup_motor(self) -> None:
        self._ctrl = self._make_controller()
        vendor = self._cfg.vendor
        if vendor == "damiao":
            self._mot = self._ctrl.add_damiao_motor(
                self._cfg.motor_id, self._cfg.feedback_id, self._cfg.model)
        elif vendor == "myactuator":
            self._mot = self._ctrl.add_myactuator_motor(
                self._cfg.motor_id, self._cfg.feedback_id, self._cfg.model)
        elif vendor == "robstride":
            self._mot = self._ctrl.add_robstride_motor(
                self._cfg.motor_id, self._cfg.feedback_id, self._cfg.model)
        else:
            raise ValueError(f"Unsupported vendor: {vendor}")

    @property
    def mode(self) -> str:
        return self._mode

    def connect(self) -> None:
        pass

    # ------------------------------------------------------------------
    # 使能 / 失能
    # ------------------------------------------------------------------

    def enable(self, retries: int = 10, poll_interval: float = 0.1) -> bool:
        try:
            self._ctrl.enable_all()
        except CallError as e:
            print(f"[enable] 调用 enable_all() 失败: {e}")
            return False

        for _ in range(retries):
            self._poll()
            try:
                st = self._mot.get_state()
                if st is not None and st.status_code == 1:
                    return True
            except Exception:
                pass
            time.sleep(poll_interval)

        try:
            st = self._mot.get_state()
            print(f"[enable] 使能失败: status_code={st.status_code if st else None}")
        except Exception:
            print("[enable] 使能失败")
        return False

    def disable(self, retries: int = 10, poll_interval: float = 0.1) -> bool:
        self.stop_control_loop()
        try:
            self._ctrl.disable_all()
        except CallError as e:
            print(f"[disable] 调用 disable_all() 失败: {e}")
            return False

        for _ in range(retries):
            self._poll()
            try:
                st = self._mot.get_state()
                if st is not None and st.status_code == 0:
                    return True
            except Exception:
                pass
            time.sleep(poll_interval)

        try:
            st = self._mot.get_state()
            print(f"[disable] 失能失败: status_code={st.status_code if st else None}")
        except Exception:
            print("[disable] 失能失败")
        return False

    # ------------------------------------------------------------------
    # 零点设置
    # ------------------------------------------------------------------

    def set_zero(self, poll_max: int = 200, poll_interval: float = 0.05) -> bool:
        self.disable()
        time.sleep(0.3)

        ready = False
        for _ in range(poll_max):
            self._poll()
            try:
                st = self._mot.get_state()
                if st is not None and st.status_code == 0:
                    ready = True
                    break
            except Exception:
                pass
            time.sleep(poll_interval)

        if not ready:
            print("[set_zero] 等待状态就绪超时")
            return False

        try:
            self._mot.set_zero_position()
            print("[set_zero] OK")
            return True
        except CallError as e:
            print(f"[set_zero] {e}")
            return False

    # ------------------------------------------------------------------
    # 反馈请求与轮询（分离 request / poll，与 arm.py 一致）
    # ------------------------------------------------------------------

    def _request(self) -> None:
        """主动向电机发送反馈请求。"""
        try:
            self._mot.request_feedback()
        except Exception:
            pass

    def _poll(self) -> None:
        """轮询总线，将电机响应写入本地缓存。"""
        try:
            self._ctrl.poll_feedback_once()
        except Exception:
            pass

    def _request_and_poll(self) -> None:
        """请求反馈 + 轮询总线（调用后需再调用 _poll 一次使数据就绪）。"""
        self._request()
        self._poll()

    def get_state(self, request: bool = True) -> tuple[float, float, float]:
        """返回 (position, velocity, torque)。

        Args:
            request: 是否主动请求反馈。控制循环中为 False，
                     因为控制命令已请求反馈；独立调用时为 True。
        """
        if request:
            self._request_and_poll()
        self._poll()
        try:
            st = self._mot.get_state()
            if st is not None:
                return (st.pos, st.vel, st.torq)
        except Exception:
            pass
        return (0.0, 0.0, 0.0)

    def get_position(self, request: bool = True) -> float:
        pos, _, _ = self.get_state(request=request)
        return pos

    def get_velocity(self, request: bool = True) -> float:
        _, vel, _ = self.get_state(request=request)
        return vel

    def get_torque(self, request: bool = True) -> float:
        _, _, torq = self.get_state(request=request)
        return torq

    # ------------------------------------------------------------------
    # 模式切换
    # ------------------------------------------------------------------

    def _ensure_mode(self, mode: Mode, timeout_ms: int = 1000) -> bool:
        try:
            self._mot.ensure_mode(mode, timeout_ms)
            return True
        except CallError as e:
            print(f"[ensure_mode] {e}")
            return False

    def mode_mit(self, kp: Optional[float] = None,
                 kd: Optional[float] = None,
                 stabilize_delay: float = 0.2) -> bool:
        self._mode = "mit"
        if kp is not None:
            self._mit_kp = kp
        if kd is not None:
            self._mit_kd = kd
        ok = self._ensure_mode(Mode.MIT)
        time.sleep(stabilize_delay)
        return ok

    def mode_pos_vel(self, stabilize_delay: float = 0.2) -> bool:
        self._mode = "pos_vel"
        try:
            self._mot.write_register_f32(25, self._cfg.vel_kp)
            self._mot.write_register_f32(26, self._cfg.vel_ki)
            self._mot.write_register_f32(27, self._cfg.pos_kp)
            self._mot.write_register_f32(28, self._cfg.pos_ki)
            time.sleep(0.02)
        except Exception as e:
            print(f"[mode_pos_vel] 写 PI 参数失败: {e}")
        ok = self._ensure_mode(Mode.POS_VEL)
        time.sleep(stabilize_delay)
        return ok

    def mode_vel(self, stabilize_delay: float = 0.2) -> bool:
        self._mode = "vel"
        ok = self._ensure_mode(Mode.VEL)
        time.sleep(stabilize_delay)
        return ok

    # ------------------------------------------------------------------
    # 控制命令（供控制循环回调使用）
    # ------------------------------------------------------------------

    def mit(self, pos: float, vel: float = 0.0,
            kp: Optional[float] = None,
            kd: Optional[float] = None,
            tau: float = 0.0) -> None:
        if kp is None:
            kp = self._mit_kp
        if kd is None:
            kd = self._mit_kd
        try:
            self._mot.send_mit(float(pos), float(vel), float(kp), float(kd), float(tau))
        except CallError:
            pass
        self._request()
        self._poll()

    def pos_vel(self, pos: float, vlim: Optional[float] = None) -> None:
        if vlim is None:
            vlim = self._cfg.vlim
        try:
            self._mot.send_pos_vel(float(pos), float(vlim))
        except CallError:
            pass
        self._request()
        self._poll()

    def set_vel(self, vel: float) -> None:
        try:
            self._mot.send_vel(float(vel))
        except CallError:
            pass
        self._request()
        self._poll()

    # ------------------------------------------------------------------
    # 多线程控制循环
    # ------------------------------------------------------------------

    def start_control_loop(
        self,
        controller: Callable[["Gripper", float], None],
        rate: float = 100.0,
    ) -> None:
        """启动后台控制循环。

        Args:
            controller: 回调函数，签名 f(gripper, dt)，在循环中调用 gripper.mit/pos_vel/set_vel。
            rate: 控制频率（Hz）。
        """
        if self._loop_running:
            print("[start_control_loop] 已在运行，先停止")
            self.stop_control_loop()
            time.sleep(0.3)

        self._rate = rate
        dt = 1.0 / rate
        self._loop_running = True
        self._loop_stop.clear()

        def loop():
            last = time.perf_counter()
            while not self._loop_stop.is_set():
                now = time.perf_counter()
                elapsed = now - last
                if elapsed >= dt:
                    last += dt
                    controller(self, elapsed)
                else:
                    time.sleep(1e-4)

        self._loop_thread = threading.Thread(target=loop, daemon=True)
        self._loop_thread.start()

    def stop_control_loop(self) -> None:
        """停止后台控制循环。"""
        if not self._loop_running:
            return
        self._loop_stop.set()
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=1.0)
            self._loop_thread = None
        self._loop_running = False

    # ------------------------------------------------------------------
    # 连接 / 断开
    # ------------------------------------------------------------------

    def disconnect(self) -> None:
        self.stop_control_loop()
        self.disable()
        time.sleep(0.5)
        if self._ctrl is not None:
            self._ctrl.shutdown()
            self._ctrl.close()
            self._ctrl = None

    def __enter__(self) -> "Gripper":
        return self

    def __exit__(self, *args) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        return f"Gripper({self._cfg.name!r}, mode={self._mode})"
