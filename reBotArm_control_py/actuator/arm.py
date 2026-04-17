"""机械臂控制句柄，直接基于 motorbridge SDK。

支持多厂商电机（Damiao / MyActuator / RobStride），
通过 `yaml` 配置文件导入所有参数，提供 MIT / POS_VEL / VEL 三种控制模式，
使能、回零、多线程高频控制等功能。
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional, Dict

import numpy as np
import yaml

from motorbridge import Controller, Mode, CallError


# ------------------------------------------------------------------
# 配置加载
# ------------------------------------------------------------------

@dataclass
class JointCfg:
    name: str
    motor_id: int
    feedback_id: int
    model: str
    vendor: str = "damiao"
    kp: float = 0.0
    kd: float = 0.0
    vel_kp: float = 0.0
    vel_ki: float = 0.0
    pos_kp: float = 0.0
    pos_ki: float = 0.0
    vlim: float = 0.0


def load_cfg(path: str) -> dict:
    with open(Path(path), "r") as f:
        data = yaml.safe_load(f)

    joints = []
    for j in data.get("joints", []):
        mc = j.get("MIT", {})
        pc = j.get("POS_VEL", {})
        joints.append(JointCfg(
            name=j["name"],
            motor_id=int(j["motor_id"]),
            feedback_id=int(j["feedback_id"]),
            model=str(j.get("model", "4340P")),
            vendor=str(j.get("vendor", "damiao")).lower(),
            kp=float(mc.get("kp", 0.0)),
            kd=float(mc.get("kd", 0.0)),
            vel_kp=float(pc.get("vel_kp", 0.0)),
            vel_ki=float(pc.get("vel_ki", 0.0)),
            pos_kp=float(pc.get("pos_kp", 0.0)),
            pos_ki=float(pc.get("pos_ki", 0.0)),
            vlim=float(pc.get("vlim", 2.0)),
        ))

    return {
        "name": data.get("name", "reBotArm"),
        "channel": data.get("channel", "/dev/ttyACM0"),
        "rate": float(data.get("rate", 500.0)),
        "joints": joints,
    }


# ------------------------------------------------------------------
# RobotArm
# ------------------------------------------------------------------

class RobotArm:
    """机械臂控制句柄，直接持有 motorbridge.Controller，按厂商分组。

    严格按照 motorbridge 官方示例的用法：
        - Controller 始终用 with 语句管理生命周期
        - ensure_mode / enable_all / send_* 等操作全部包裹 try-except
        - 所有 CallError 打印提示后继续，不中断流程
    """

    def __init__(self, cfg_path: Optional[str] = None) -> None:
        if cfg_path is None:
            cfg_path = Path(__file__).parent.parent.parent / "config" / "arm.yaml"
        cfg = load_cfg(str(cfg_path))

        self._name = cfg["name"]
        self._channel = cfg["channel"]
        self._rate = cfg["rate"]
        self._joints: List[JointCfg] = cfg["joints"]
        self._mode = "mit"

        # 按 vendor 分组控制器，同一 channel 同厂商共用一个 Controller
        # vendor -> Controller（外部持有，不在这里 open/close）
        self._ctrl_map: Dict[str, Controller] = {}
        # name -> motor handle
        self._motor_map: Dict[str, any] = {}

        self._running = False
        self._ctrl_thread: Optional[threading.Thread] = None
        self._ctrl_fn: Optional[Callable] = None

        # 运行时参数（由 mode_mit / mode_pos_vel 填充）
        self._mit_kp: Optional[np.ndarray] = None
        self._mit_kd: Optional[np.ndarray] = None
        self._pv_vlim: Optional[np.ndarray] = None

        self._setup_motors()

    # ------------------------------------------------------------------
    # 内部：初始化电机
    # ------------------------------------------------------------------

    def _make_controller(self, vendor: str) -> Controller:
        """按 channel 类型创建 Controller，与 motorbridge 官方示例一致。"""
        if self._channel.startswith("/dev/tty"):
            return Controller.from_dm_serial(self._channel, 921600)
        return Controller(self._channel)

    def _setup_motors(self) -> None:
        for jc in self._joints:
            vendor = jc.vendor
            if vendor not in self._ctrl_map:
                self._ctrl_map[vendor] = self._make_controller(vendor)

            ctrl = self._ctrl_map[vendor]

            if vendor == "damiao":
                mot = ctrl.add_damiao_motor(jc.motor_id, jc.feedback_id, jc.model)
            elif vendor == "myactuator":
                mot = ctrl.add_myactuator_motor(jc.motor_id, jc.feedback_id, jc.model)
            elif vendor == "robstride":
                mot = ctrl.add_robstride_motor(jc.motor_id, jc.feedback_id, jc.model)
            else:
                raise ValueError(f"Unsupported vendor: {vendor}")

            self._motor_map[jc.name] = mot

    # ------------------------------------------------------------------
    # 属性
    # ------------------------------------------------------------------

    @property
    def num_joints(self) -> int:
        return len(self._joints)

    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self._joints]

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def control_loop_active(self) -> bool:
        t = getattr(self, "_ctrl_thread", None)
        return t is not None and t.is_alive()

    # ------------------------------------------------------------------
    # 连接 / 断开
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """预留扩展口，Controller 在 __init__ 时已建立连接。"""
        pass

    def disconnect(self) -> None:
        """安全关闭所有控制器：停止循环 → 失能 → 等待 → 关闭。

        在 shutdown/close 之前留出充足时间，让电机固件完成失能处理。
        """
        self.stop_control_loop()
        self.disable()
        time.sleep(0.5)   # 等待电机固件完成失能指令
        for ctrl in self._ctrl_map.values():
            ctrl.shutdown()
            time.sleep(0.1)   # 每个控制器关闭前短暂等待
            ctrl.close()
        self._ctrl_map.clear()
        self._motor_map.clear()

    def reconnect(self,
                init_delay: float = 1.0,
                post_setup_delay: float = 0.5) -> None:
        """重新初始化全部控制器和电机。

        用于连接意外断开后恢复。调用时会先关闭已有连接，再重建。

        Args:
            init_delay: 控制器创建后等待固件初始化的时间（秒）。
            post_setup_delay: 全部电机注册完成后等待总线稳定的时间（秒）。
        """
        self.disconnect()
        time.sleep(init_delay)

        for vendor in self._ctrl_map.keys():
            ctrl = self._make_controller(vendor)
            self._ctrl_map[vendor] = ctrl

        self._motor_map.clear()
        for jc in self._joints:
            vendor = jc.vendor
            ctrl = self._ctrl_map[vendor]
            if vendor == "damiao":
                mot = ctrl.add_damiao_motor(jc.motor_id, jc.feedback_id, jc.model)
            elif vendor == "myactuator":
                mot = ctrl.add_myactuator_motor(jc.motor_id, jc.feedback_id, jc.model)
            elif vendor == "robstride":
                mot = ctrl.add_robstride_motor(jc.motor_id, jc.feedback_id, jc.model)
            elif vendor == "hightorque":
                mot = ctrl.add_hightorque_motor(jc.motor_id, jc.feedback_id, jc.model)
            else:
                raise ValueError(f"Unsupported vendor: {vendor}")
            self._motor_map[jc.name] = mot
            time.sleep(0.05)

        time.sleep(post_setup_delay)
        print("[reconnect] 控制器和电机已重新初始化")

    # ------------------------------------------------------------------
    # 使能
    # ------------------------------------------------------------------

    def enable(self, vendor: Optional[str] = None,
                delay_per_motor: float = 0.05,
                retries: int = 10,
                poll_interval: float = 0.1) -> None:
        """使能全部电机，逐电机延迟并重试直到电机响应。

        Args:
            vendor: 指定厂商，None 表示所有。
            delay_per_motor: 每使能一个电机后等待的时间（秒）。
            retries: 每电机最大重试次数。
            poll_interval: 每次重试间隔（秒）。
        """
        if vendor is not None:
            vendors = [vendor]
        else:
            vendors = list(self._ctrl_map.keys())

        for v in vendors:
            try:
                self._ctrl_map[v].enable_all()
            except CallError as e:
                print(f"[enable/{v}] 调用 enable_all() 失败: {e}")
            time.sleep(delay_per_motor)

        if retries <= 0:
            return

        enabled: Dict[str, bool] = {jc.name: False for jc in self._joints}
        failed: List[str] = []

        for attempt in range(retries):
            self._request_and_poll()
            all_done = True
            for jc in self._joints:
                if enabled[jc.name]:
                    continue
                try:
                    st = self._motor_map[jc.name].get_state()
                    if st is not None and st.status_code == 1:
                        enabled[jc.name] = True
                    else:
                        all_done = False
                except Exception:
                    all_done = False
            if all_done:
                break
            time.sleep(poll_interval)

        for jc in self._joints:
            if not enabled[jc.name]:
                try:
                    st = self._motor_map[jc.name].get_state()
                    status = st.status_code if st is not None else None
                except Exception:
                    status = None
                print(f"[enable] {jc.name} 使能失败（已重试 {retries} 次）: "
                      f"status_code={status}")
                failed.append(jc.name)

        if failed:
            print(f"[enable] 未就绪电机: {failed}")

    def disable(self, vendor: Optional[str] = None,
                delay_per_motor: float = 0.05,
                retries: int = 10,
                poll_interval: float = 0.1) -> None:
        """失能全部电机，逐电机延迟确保指令被接收。

        注意：如果控制循环正在运行，会先停止循环（调用 stop_control_loop），
        避免 MIT 命令与 disable 指令在 CAN 总线上竞争。

        Args:
            vendor: 指定厂商，None 表示所有。
            delay_per_motor: 每失能一个电机后等待的时间（秒）。
            retries: 每电机最大重试次数。
            poll_interval: 每次重试间隔（秒）。
        """
        t = getattr(self, "_ctrl_thread", None)
        if t is not None and t.is_alive():
            self.stop_control_loop()
        vendors = [vendor] if vendor is not None else list(self._ctrl_map.keys())

        for v in vendors:
            try:
                self._ctrl_map[v].disable_all()
            except CallError as e:
                print(f"[disable/{v}] 调用 disable_all() 失败: {e}")
            time.sleep(delay_per_motor)

        if retries <= 0:
            return

        disabled: Dict[str, bool] = {jc.name: False for jc in self._joints}

        for attempt in range(retries):
            self._request_and_poll()
            all_disabled = True
            for jc in self._joints:
                if disabled[jc.name]:
                    continue
                try:
                    st = self._motor_map[jc.name].get_state()
                    if st is not None and st.status_code == 0:
                        disabled[jc.name] = True
                    else:
                        all_disabled = False
                except Exception:
                    all_disabled = False
            if all_disabled:
                break
            time.sleep(poll_interval)

        failed: List[str] = []
        for jc in self._joints:
            if disabled[jc.name]:
                continue
            try:
                st = self._motor_map[jc.name].get_state()
                status = st.status_code if st is not None else None
            except Exception as e:
                status = None
                exc_info = str(e)
            else:
                exc_info = None
            print(f"[disable] {jc.name} 失能失败（已重试 {retries} 次）: "
                  f"status_code={status}, vendor={jc.vendor}"
                  + (f", error={exc_info}" if exc_info else ""))
            failed.append(jc.name)

        if failed:
            print(f"[disable] 仍处使能状态的电机: {failed}")

    # ------------------------------------------------------------------
    # 零点设置
    # ------------------------------------------------------------------

    def set_zero(self,
                poll_max: int = 200,
                poll_interval: float = 0.05,
                set_zero_delay: float = 0.1) -> None:
        """将全部关节的当前位置设为零点（先 disable，设完后保持失能）。

        Args:
            poll_max: 轮询反馈最多次数，等待电机 status_code==0。
            poll_interval: 每次轮询间隔（秒）。
            set_zero_delay: 每个电机设零后等待时间（秒）。
        """
        self.disable()
        time.sleep(0.3)

        for jc in self._joints:
            mot = self._motor_map[jc.name]
            ctrl = None
            for v, ctrl in self._ctrl_map.items():
                if v == jc.vendor:
                    break

            ready = False
            for _ in range(poll_max):
                self._request_and_poll()
                try:
                    st = mot.get_state()
                    if st is not None and st.status_code == 0:
                        ready = True
                except Exception:
                    pass
                if ready:
                    break
                time.sleep(poll_interval)

            if not ready:
                print(f"[set_zero] {jc.name}: 等待状态就绪超时，跳过")
                time.sleep(set_zero_delay)
                continue

            try:
                mot.set_zero_position()
                print(f"[set_zero] {jc.name}: OK")
            except CallError as e:
                print(f"[set_zero] {jc.name}: {e}")
            time.sleep(set_zero_delay)

    def set_zero_single(self, name: str,
                        poll_max: int = 200,
                        poll_interval: float = 0.05) -> bool:
        """将单个关节的当前位置设为零点（先 disable，设完后保持失能）。

        Args:
            name: 关节名称。
            poll_max: 轮询反馈最多次数。
            poll_interval: 每次轮询间隔（秒）。

        Returns:
            是否成功设置零点。
        """
        if name not in self._motor_map:
            raise KeyError(f"Unknown joint: {name}")

        self.disable()
        time.sleep(0.3)

        jc = next(j for j in self._joints if j.name == name)
        mot = self._motor_map[name]
        ctrl = self._ctrl_map.get(jc.vendor)

        if ctrl is not None:
            for _ in range(poll_max):
                self._request_and_poll()
                try:
                    st = mot.get_state()
                    if st is not None and st.status_code == 0:
                        break
                except Exception:
                    pass
                time.sleep(poll_interval)

        try:
            mot.set_zero_position()
            return True
        except CallError as e:
            print(f"[set_zero] {name}: {e}")
            return False

    # ------------------------------------------------------------------
    # 状态读取
    # ------------------------------------------------------------------

    def _poll_all(self, dt: float = 0.0) -> None:
        """轮询所有控制器的反馈数据（仅读取，不主动发送请求）。

        反馈数据应在 mit() 或其他发送命令时同步请求，避免总线争用。
        """
        for ctrl in self._ctrl_map.values():
            try:
                ctrl.poll_feedback_once()
            except Exception:
                pass

    def _ctrl_to_motors(self) -> Dict[any, List[any]]:
        """返回 {Controller: [Motor, ...]} 反查表，用于逐控制器遍历其下所有电机。"""
        m: Dict[any, List[any]] = {}
        for jc in self._joints:
            ctrl = self._ctrl_map[jc.vendor]
            mot = self._motor_map[jc.name]
            m.setdefault(ctrl, []).append(mot)
        return m

    def _request_and_poll(self) -> None:
        """主动请求并轮询所有控制器的反馈（用于非控制循环场景）。"""
        for ctrl, motors in self._ctrl_to_motors().items():
            for mot in motors:
                try:
                    mot.request_feedback()
                except Exception:
                    pass
            try:
                ctrl.poll_feedback_once()
            except Exception:
                pass

    def get_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """读取当前所有关节的状态（位置、速度、力矩）。

        注意：如果在控制循环中使用，默认已通过 mit() 请求反馈，此处仅轮询读取。
              如果单独使用（无控制循环），会主动请求反馈。
        """
        self._poll_all()
        pos, vel, torq = [], [], []
        for jc in self._joints:
            st = self._motor_map[jc.name].get_state()
            if st is not None:
                pos.append(st.pos)
                vel.append(st.vel)
                torq.append(st.torq)
            else:
                pos.append(0.0)
                vel.append(0.0)
                torq.append(0.0)
        return (
            np.array(pos, dtype=np.float64),
            np.array(vel, dtype=np.float64),
            np.array(torq, dtype=np.float64),
        )

    def get_positions(self, request: bool = False) -> np.ndarray:
        """读取当前所有关节的位置。

        Args:
            request: 是否主动请求反馈。控制循环运行时默认为 False，
                     因为 mit() 已请求反馈。单独调用时设为 True。
        """
        if request:
            self._request_and_poll()
        else:
            self._poll_all()
        pos = []
        for jc in self._joints:
            st = self._motor_map[jc.name].get_state()
            pos.append(st.pos if st is not None else 0.0)
        return np.array(pos, dtype=np.float64)

    def get_velocities(self, request: bool = False) -> np.ndarray:
        """读取当前所有关节的速度。

        Args:
            request: 是否主动请求反馈。
        """
        if request:
            self._request_and_poll()
        else:
            self._poll_all()
        vel = []
        for jc in self._joints:
            st = self._motor_map[jc.name].get_state()
            vel.append(st.vel if st is not None else 0.0)
        return np.array(vel, dtype=np.float64)

    def get_torques(self, request: bool = False) -> np.ndarray:
        """读取当前所有关节的力矩。

        Args:
            request: 是否主动请求反馈。
        """
        if request:
            self._request_and_poll()
        else:
            self._poll_all()
        torq = []
        for jc in self._joints:
            st = self._motor_map[jc.name].get_state()
            torq.append(st.torq if st is not None else 0.0)
        return np.array(torq, dtype=np.float64)

    # ------------------------------------------------------------------
    # 模式切换（官方风格：try-except ensure_mode，跳过失败的电机）
    # ------------------------------------------------------------------

    def _ensure_mode(self, name: str, mot: any, mode: Mode,
                     timeout_ms: int = 1000) -> bool:
        """对单个电机 ensure_mode，失败打印提示但不抛异常。"""
        try:
            mot.ensure_mode(mode, timeout_ms)
            return True
        except CallError as e:
            print(f"[ensure_mode/{name}] 跳过: {e}")
            return False

    def mode_mit(self, kp: Optional[np.ndarray] = None,
                 kd: Optional[np.ndarray] = None,
                 stabilize_delay: float = 0.2) -> bool:
        """切换到 MIT 模式，返回是否全部成功。

        Args:
            stabilize_delay: 全部电机切换完成后等待稳定的时间（秒）。
        """
        self._mode = "mit"
        if kp is None:
            kp = np.array([j.kp for j in self._joints], dtype=np.float64)
        if kd is None:
            kd = np.array([j.kd for j in self._joints], dtype=np.float64)
        self._mit_kp = np.asarray(kp, dtype=np.float64)
        self._mit_kd = np.asarray(kd, dtype=np.float64)

        ok = True
        for jc in self._joints:
            if not self._ensure_mode(jc.name, self._motor_map[jc.name],
                                      Mode.MIT, 1000):
                ok = False
            time.sleep(0.05)
        time.sleep(stabilize_delay)
        return ok

    def mode_pos_vel(self, vlim: Optional[np.ndarray] = None,
                     stabilize_delay: float = 0.2) -> bool:
        """切换到 POS_VEL 模式（位置+速度环 PI），返回是否全部成功。

        Args:
            stabilize_delay: 全部电机切换完成后等待稳定的时间（秒）。
        """
        self._mode = "pos_vel"
        if vlim is None:
            vlim = np.array([j.vlim for j in self._joints], dtype=np.float64)
        self._pv_vlim = np.asarray(vlim, dtype=np.float64)

        ok = True
        for jc in self._joints:
            m = self._motor_map[jc.name]
            try:
                m.write_register_f32(25, jc.vel_kp)   # KP_ASR  — 速度环 Kp
                m.write_register_f32(26, jc.vel_ki)   # KI_ASR  — 速度环 Ki
                m.write_register_f32(27, jc.pos_kp)   # KP_APR  — 位置环 Kp
                m.write_register_f32(28, jc.pos_ki)   # KI_APR  — 位置环 Ki
                time.sleep(0.02)
            except Exception as e:
                print(f"[mode_pos_vel/{jc.name}] 写 PI 参数失败: {e}")
            if not self._ensure_mode(jc.name, m, Mode.POS_VEL, 1000):
                ok = False
            time.sleep(0.05)
        time.sleep(stabilize_delay)
        return ok

    def mode_vel(self, stabilize_delay: float = 0.2) -> bool:
        """切换到纯速度模式。

        Args:
            stabilize_delay: 全部电机切换完成后等待稳定的时间（秒）。
        """
        self._mode = "vel"
        ok = True
        for jc in self._joints:
            if not self._ensure_mode(jc.name, self._motor_map[jc.name],
                                      Mode.VEL, 1000):
                ok = False
            time.sleep(0.05)
        time.sleep(stabilize_delay)
        return ok

    # ------------------------------------------------------------------
    # MIT 控制
    # ------------------------------------------------------------------

    def mit(self, pos: np.ndarray,
            vel: Optional[np.ndarray] = None,
            kp: Optional[np.ndarray] = None,
            kd: Optional[np.ndarray] = None,
            tau: Optional[np.ndarray] = None,
            request_feedback: bool = True) -> None:
        """发送 MIT 控制命令，同时可选地请求电机反馈。

        Args:
            request_feedback: 是否在控制命令后请求反馈。
                              设为 True 可在控制循环中使用，让主线程
                              直接读取最新状态，避免总线争用导致的延迟。
        """
        n = self.num_joints
        pos = np.asarray(pos, dtype=np.float64).reshape(-1)
        if vel is None:
            vel = np.zeros(n)
        if tau is None:
            tau = np.zeros(n)
        if kp is None:
            kp = getattr(self, "_mit_kp",
                         np.array([j.kp for j in self._joints], dtype=np.float64))
        if kd is None:
            kd = getattr(self, "_mit_kd",
                         np.array([j.kd for j in self._joints], dtype=np.float64))

        for i, jc in enumerate(self._joints):
            try:
                self._motor_map[jc.name].send_mit(
                    float(pos[i]), float(vel[i]),
                    float(kp[i]), float(kd[i]), float(tau[i]),
                )
            except CallError:
                pass

        if request_feedback:
            for ctrl, motors in self._ctrl_to_motors().items():
                for mot in motors:
                    try:
                        mot.request_feedback()
                    except Exception:
                        pass
                try:
                    ctrl.poll_feedback_once()
                except Exception:
                    pass

    # ------------------------------------------------------------------
    # POS_VEL 控制
    # ------------------------------------------------------------------

    def pos_vel(self, pos: np.ndarray,
                vlim: Optional[np.ndarray] = None) -> None:
        pos = np.asarray(pos, dtype=np.float64).reshape(-1)
        if vlim is None:
            vlim = getattr(self, "_pv_vlim",
                           np.array([j.vlim for j in self._joints], dtype=np.float64))
        for i, jc in enumerate(self._joints):
            try:
                self._motor_map[jc.name].send_pos_vel(
                    float(pos[i]), float(vlim[i]),
                )
            except CallError:
                pass

    # ------------------------------------------------------------------
    # 纯速度控制
    # ------------------------------------------------------------------

    def set_vel(self, vel: np.ndarray) -> None:
        vel = np.asarray(vel, dtype=np.float64).reshape(-1)
        for i, jc in enumerate(self._joints):
            try:
                self._motor_map[jc.name].send_vel(float(vel[i]))
            except CallError:
                pass

    # ------------------------------------------------------------------
    # 紧急停止
    # ------------------------------------------------------------------

    def estop(self) -> None:
        self.disable()

    # ------------------------------------------------------------------
    # 多线程控制循环
    # ------------------------------------------------------------------

    def start_control_loop(
        self,
        control_fn: Callable[["RobotArm", float], None],
        rate: Optional[float] = None,
    ) -> None:
        if self.control_loop_active:
            raise RuntimeError("控制循环已在运行，请先调用 stop_control_loop()")
        self._running = True
        self._ctrl_rate = rate if rate is not None else self._rate
        self._ctrl_fn = control_fn
        self._ctrl_thread = threading.Thread(
            target=self._control_loop_impl,
            name="robotarm-control-loop",
            daemon=True,
        )
        self._ctrl_thread.start()

    def _control_loop_impl(self) -> None:
        dt = 1.0 / self._ctrl_rate
        while self._running:
            t0 = time.perf_counter()
            try:
                self._ctrl_fn(self, dt)
            except Exception:
                if self._running:
                    raise
            elapsed = time.perf_counter() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop_control_loop(self) -> None:
        self._running = False
        t = getattr(self, "_ctrl_thread", None)
        if t is not None and t.is_alive():
            t.join(timeout=5.0)
            if t.is_alive():
                try:
                    t._stop()
                except Exception:
                    pass
        time.sleep(0.05)

    # ------------------------------------------------------------------
    # 上下文管理器
    # ------------------------------------------------------------------

    def __enter__(self) -> "RobotArm":
        return self

    def __exit__(self, *args) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        return (f"RobotArm({self._name!r}, joints={self.num_joints}, "
                f"mode={self._mode}, rate={self._ctrl_rate}Hz)")
