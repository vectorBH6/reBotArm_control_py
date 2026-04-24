#!/usr/bin/env python3
"""重力补偿控制演示。

使用 Pinocchio 计算当前关节构型下的广义重力向量 g(q)，
通过 MIT 模式的前馈力矩直接补偿重力，使机械臂可以在任意姿态下
"漂浮"，即松开后不会因自重坠落。

控制律（MIT 位置闭环 + 重力前馈）：
    tau = g(q)          — 重力前馈
    pos = 当前电机位置   — 关节位置目标跟随当前位置
    kp   = 0,  kd = 1.0   — 所有电机统一刚度/阻尼

终端持续打印每个关节的期望力矩（N·m）。
"""
import signal
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RobotArm
from reBotArm_control_py.dynamics import (
    load_dynamics_model,
    compute_generalized_gravity,
    get_default_gravity,
)


# --------------------------------------------------------------------------- #
# 全局控制标志
# --------------------------------------------------------------------------- #

_running = True


def _sigint_handler(signum, frame):
    global _running
    print("\n[gravity_comp] 收到 Ctrl+C，准备停止...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


# --------------------------------------------------------------------------- #
# 控制回调（每周期调用一次，由 RobotArm 控制循环驱动）
# --------------------------------------------------------------------------- #

def gravity_compensation_controller(arm: RobotArm, dt: float) -> None:
    """重力补偿控制回调。

    读取当前关节位置 → Pinocchio 计算 g(q) → MIT 前馈力矩。
    """
    # 1. 读取当前关节位置
    q = arm.get_positions()          # shape=(6,), 单位: rad

    # 2. Pinocchio 计算广义重力向量
    tau_g = compute_generalized_gravity(q=q)   # shape=(6,), 单位: N·m

    # 3. MIT 前馈: 位置目标跟随当前电机位置，kp=0, kd=1，重力补偿
    arm.mit(
        pos=q,
        vel=np.zeros(arm.num_joints),
        kp=np.full(arm.num_joints, 0.0),
        kd=np.full(arm.num_joints, 1.0),
        tau=tau_g,
        request_feedback=True,
    )

    # 4. 终端打印（每隔 ~20 个周期打印一次，避免刷屏）
    gravity_compensation_controller._counter += 1
    if gravity_compensation_controller._counter % 20 == 0:
        print(
            f"[{gravity_compensation_controller._counter:4d}] "
            f"tau_g = " + "  ".join(f"{t:+.3f}" for t in tau_g) + "  N·m"
        )


gravity_compensation_controller._counter = 0


# --------------------------------------------------------------------------- #
# 主程序
# --------------------------------------------------------------------------- #

def main() -> None:
    print("=" * 60)
    print("  reBotArm 重力补偿演示")
    print("  预计行为: 机械臂维持位置不动，可以手动掰动至任何位置）")
    print("  Ctrl+C 停止并断开连接")
    print("=" * 60)

    # 动力学模型初始化
    model = load_dynamics_model()
    g_vec = get_default_gravity()
    print(f"\n[模型] nq={model.nq}, nv={model.nv}")
    print(f"[重力] {g_vec}  m/s²")

    # 机器人连接
    arm = RobotArm()
    arm.connect()
    print("\n[连接] OK")

    # 使能电机
    arm.enable()
    print("[使能] OK")
    # arm.disable()  # 先失能测试

    # 切换到 MIT 模式（kp=0, kd=1，位置目标跟随当前电机位置）
    arm.mode_mit(
        kp=np.full(arm.num_joints, 2.0),
        kd=np.full(arm.num_joints, 1.0),
    )
    arm.fresh()
    print("[MIT模式] OK（kp=2, kd=1）")

    # 启动控制循环，频率使用配置默认值 (500 Hz)
    arm.start_control_loop(gravity_compensation_controller, rate=arm._rate)
    print(f"[控制循环] 启动 @ {arm._rate} Hz")
    print("-" * 60)
    print(f"{'step':>4}  tau_g (N·m)")
    print("-" * 60)

    try:
        while _running:
            time.sleep(0.01)   # 主线程只负责保活，打印在回调中完成
    finally:
        print("\n[停止] 关闭控制循环...")
        arm.disconnect()
        print("[完成] 已安全断开连接")


if __name__ == "__main__":
    main()
