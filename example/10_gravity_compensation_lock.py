#!/usr/bin/env python3
"""重力补偿控制演示（末端速度锁止版）。

在基础重力补偿的基础上，加入末端速度检测：
- 持续计算末端执行器的线速度和角速度
- 当末端速度 ||v_ee|| < 阈值 时：目标关节角度保持锁定（被推也纹丝不动）
- 当末端速度 ||v_ee|| > 阈值 时：目标关节角度更新为当前关节角度（用力才能换位置）

控制律（MIT 模式）：
    tau = g(q) + kp·(q_target - q) + kd·(0 - q̇)

末端速度锁止规则：
    v_ee  = J_lin(q)  · q̇   — 末端线速度
    w_ee  = J_ang(q)  · q̇   — 末端角速度
    if ||v_ee|| > vel_threshold or ||w_ee|| > vel_threshold:
        q_target = q
"""
import signal
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RobotArm
from reBotArm_control_py.dynamics import (
    load_dynamics_model,
    compute_generalized_gravity,
    get_default_gravity,
)
from reBotArm_control_py.kinematics import load_robot_model


# --------------------------------------------------------------------------- #
# 全局控制标志 & 目标关节角度（锁止状态）
# --------------------------------------------------------------------------- #

_running = True
_q_target: np.ndarray = None
_lock_counter = 0
_integral: np.ndarray = None  # 积分累积项，单位 N·m（每个关节）


def _sigint_handler(signum, frame):
    global _running
    print("\n[gravity_comp] 收到 Ctrl+C，准备停止...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


# --------------------------------------------------------------------------- #
# 可调参数
# --------------------------------------------------------------------------- #

_VEL_THRESHOLD = 0.04   # 末端速度阈值 (m/s)，超过才更新目标角度
_W_VEL_THRESHOLD = 0.08   # 末端角速度阈值 (rad/s)，超过才更新目标角度
_EE_FRAME = "end_link"
_KP = 8.0
_KD = 1.0


# --------------------------------------------------------------------------- #
# 动力学模型（模块级初始化）
# --------------------------------------------------------------------------- #

_model = load_robot_model()
_data = _model.createData()
_ee_frame_id = _model.getFrameId(_EE_FRAME)


# --------------------------------------------------------------------------- #
# 控制回调
# --------------------------------------------------------------------------- #

def gravity_compensation_controller(arm: RobotArm) -> None:
    """重力补偿控制回调（末端速度锁止版）。

    读取当前关节位置/速度 → Pinocchio 计算 g(q) 和末端雅可比
    → 判断末端速度是否超过阈值 → 决定是否更新目标关节角度
    → 积分项修正重力 → MIT 前馈力矩。
    """
    global _q_target, _lock_counter, _integral

    q = arm.get_positions()
    qd = arm.get_velocities()

    tau_g = compute_generalized_gravity(q=q)

    # 计算位置误差
    q_error = _q_target - q

    # 初始化积分项
    if _integral is None:
        _integral = np.zeros_like(q)

    # 积分项: 误差越大积分增加越快，上限 0.35 N·m 每个关节
    _integral += q_error * 1.0   # 积分增益，误差 1rad 每秒累积约 1.0 N·m
    np.clip(_integral, -0.5, 0.5, out=_integral)  # 限幅

    pin.computeJointJacobians(_model, _data, q)
    pin.updateFramePlacements(_model, _data)
    J = pin.getFrameJacobian(_model, _data, _ee_frame_id, pin.ReferenceFrame.WORLD)
    v_spatial = J @ qd
    v_ee_norm = float(np.linalg.norm(v_spatial[:3]))
    w_ee_norm = float(np.linalg.norm(v_spatial[3:]))

    if v_ee_norm > _VEL_THRESHOLD or w_ee_norm > _W_VEL_THRESHOLD:
        _q_target = q.copy()
        _lock_counter = 0
        _integral *= 0.9  # 速度超阈值时积分衰减，避免锁止后误差积累
    else:
        _lock_counter += 1

    arm.mit(
        pos=_q_target,
        vel=np.zeros(arm.num_joints),
        kp=np.full(arm.num_joints, _KP),
        kd=np.full(arm.num_joints, _KD),
        tau=tau_g + _integral,
    )

    gravity_compensation_controller._counter += 1
    if gravity_compensation_controller._counter % 20 == 0:
        lock_status = "LOCKED" if _lock_counter > 0 else "UPDATE"
        v_ee = v_spatial[:3]
        w_ee = v_spatial[3:]
        print(
            f"[{gravity_compensation_controller._counter:4d}] "
            f"{lock_status}  "
            f"err=" + "  ".join(f"{e:+.3f}" for e in q_error) + "  "
            f"Int=" + "  ".join(f"{i:+.3f}" for i in _integral) + "  "
            f"v={float(np.linalg.norm(v_ee)):.4f}m/s  "
            f"w={float(np.linalg.norm(w_ee)):.4f}rad/s  "
            f"tau_g=" + "  ".join(f"{t:+.3f}" for t in tau_g) + "  N·m"
        )


gravity_compensation_controller._counter = 0


# --------------------------------------------------------------------------- #
# 主程序
# --------------------------------------------------------------------------- #

def main() -> None:
    global _q_target

    print("=" * 65)
    print("  reBotArm 重力补偿演示（末端速度锁止版）")
    print(f"  末端速度阈值: {_VEL_THRESHOLD} m/s（超过才更新目标角度）")
    print("  预计行为: 机械臂锁止在当前位置，"
          "用力推才能改变目标角度")
    print("  Ctrl+C 停止并断开连接")
    print("=" * 65)

    dyn_model = load_dynamics_model()
    g_vec = get_default_gravity()
    print(f"\n[模型] nq={dyn_model.nq}, nv={dyn_model.nv}")
    print(f"[重力] {g_vec}  m/s²")

    arm = RobotArm()
    arm.connect()
    print("\n[连接] OK")

    arm.enable()
    print("[使能] OK")

    _q_target = arm.get_positions(request=True)
    print(f"[目标角度] 初始锁定: {np.rad2deg(_q_target).round(2)} deg")

    arm.mode_mit(
        kp=np.full(arm.num_joints, _KP),
        kd=np.full(arm.num_joints, _KD),
    )
    print(f"[MIT模式] OK（kp={_KP}, kd={_KD}）")

    try:
        while _running:
            gravity_compensation_controller(arm)
            time.sleep(0.002)
    finally:
        print("\n[停止] 关闭控制循环...")
        arm.disconnect()
        print("[完成] 已安全断开连接")


if __name__ == "__main__":
    main()
