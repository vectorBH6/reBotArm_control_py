#!/usr/bin/env python3
"""单电机控制测试 — 直接使用 motorbridge SDK。

用法:
    python example/0x01damiao_test.py

直接创建一个 Controller，加载 yaml 中对应关节的配置，
依次演示 MIT / POS_VEL / VEL 三种模式，支持使能、回零、状态读取。

交互命令:
    mit <pos_deg> [<vel> <kp> <kd> <tau>]  — MIT 模式指令
    posvel <pos_deg> [<vlim>]              — POS_VEL 模式指令
    vel <vel_rad_s>                         — 纯速度模式指令
    enable                                  — 使能
    disable                                 — 去使能
    set_zero                                — 电机零位设置
    mode <mit|posvel|vel>                   — 切换控制模式
    state                                   — 打印当前状态
    q / quit                                — 退出
"""

import sys
import signal
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from motorbridge import Controller, Mode

CHANNEL = "/dev/ttyACM0"         
MOTOR_ID = 0x01
FEEDBACK_ID = 0x11
MODEL = "4340P"


def signal_handler(sig, frame):
    print("\n[ctrl+c] 退出")
    sys.exit(0)


def main() -> None:
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print(f"连接到 {CHANNEL} ...")
    if CHANNEL.startswith("/dev/tty"):
        ctrl = Controller.from_dm_serial(CHANNEL, 921600)
    else:
        ctrl = Controller(CHANNEL)
    motor = ctrl.add_damiao_motor(MOTOR_ID, FEEDBACK_ID, MODEL)
    print(f"电机已注册: id={MOTOR_ID:#04x} feedback={FEEDBACK_ID:#04x} model={MODEL}")

    def do_enable() -> None:
        ctrl.enable_all()
        time.sleep(0.3)
        print("电机已使能")

    def do_disable() -> None:
        ctrl.disable_all()
        print("电机已去使能")

    def do_set_zero() -> None:
        st = motor.get_state()
        for _ in range(10):
            motor.request_feedback()
            st = motor.get_state()
            if st is not None and st.status_code == 0:
                break
            time.sleep(0.05)
        motor.set_zero_position()
        print("软件零位已设置")

    pv_pos_kp = 150.0
    pv_pos_ki = 0.5
    pv_vel_kp = 0.0125
    pv_vel_ki = 0.004

    def do_mode(args: list) -> None:
        nonlocal pv_pos_kp, pv_pos_ki, pv_vel_kp, pv_vel_ki
        if not args:
            print("用法: mode <mit|posvel|vel> [pos_kp] [pos_ki] [vel_kp] [vel_ki]")
            return
        m = args[0].lower()
        if m == "mit":
            motor.ensure_mode(Mode.MIT, 1000)
            print("切换到 MIT 模式")
        elif m == "posvel":
            if len(args) >= 5:
                pv_pos_kp = float(args[1])
                pv_pos_ki = float(args[2])
                pv_vel_kp = float(args[3])
                pv_vel_ki = float(args[4])
            motor.write_register_f32(25, pv_vel_kp)  # KP_ASR   速度环 Kp
            motor.write_register_f32(26, pv_vel_ki)  # KI_ASR   速度环 Ki
            motor.write_register_f32(27, pv_pos_kp)  # KP_APR   位置环 Kp
            motor.write_register_f32(28, pv_pos_ki)  # KI_APR   位置环 Ki
            time.sleep(0.02)
            print(f"PID 参数已写入: pos_kp={pv_pos_kp} pos_ki={pv_pos_ki} "
                  f"vel_kp={pv_vel_kp} vel_ki={pv_vel_ki}")
            motor.ensure_mode(Mode.POS_VEL, 1000)
            print("切换到 POS_VEL 模式")
        elif m == "vel":
            motor.ensure_mode(Mode.VEL, 1000)
            print("切换到 VEL 模式")
        else:
            print(f"未知模式: {m}，可用: mit / posvel / vel")

    def do_state() -> None:
        st = None
        for _ in range(10):
            motor.request_feedback()         # 再发新请求
            ctrl.poll_feedback_once()        # 立即处理新请求
            time.sleep(0.005)                # 等待响应（CAN 周期约 1ms）
            st = motor.get_state()          # 取新数据
            if st is not None and st.status_code == 0:
                break
        if st is None:
            print("无反馈数据")
            return
        print(f"pos={st.pos*180/3.14159:+.4f}deg  "
              f"vel={st.vel*180/3.14159:+.4f}deg/s  "
              f"torq={st.torq:+.4f}  "
              f"status={st.status_code}")

    def do_mit(args: list) -> None:
        if not args:
            print("用法: mit <pos_deg> [<vel> <kp> <kd> <tau>]")
            return
        pos = float(args[0]) * 3.14159265358979 / 180.0
        vel = float(args[1]) if len(args) > 1 else 0.0
        kp = float(args[2]) if len(args) > 2 else 10.0
        kd = float(args[3]) if len(args) > 3 else 2.0
        tau = float(args[4]) if len(args) > 4 else 0.0
        motor.send_mit(pos, vel, kp, kd, tau)

    def do_posvel(args: list) -> None:
        nonlocal pv_pos_kp, pv_pos_ki, pv_vel_kp, pv_vel_ki
        if not args:
            print("用法: posvel <pos_deg> [<vlim>] 或 posvel <pos_deg> <vlim> <pos_kp> <pos_ki> <vel_kp> <vel_ki>")
            return
        pos = float(args[0]) * 3.14159265358979 / 180.0
        vlim = float(args[1]) if len(args) > 1 else 2.0
        if len(args) >= 6:
            pv_pos_kp = float(args[2])
            pv_pos_ki = float(args[3])
            pv_vel_kp = float(args[4])
            pv_vel_ki = float(args[5])
            motor.write_register_f32(25, pv_vel_kp)  # KP_ASR   速度环 Kp
            motor.write_register_f32(26, pv_vel_ki)  # KI_ASR   速度环 Ki
            motor.write_register_f32(27, pv_pos_kp)  # KP_APR   位置环 Kp
            motor.write_register_f32(28, pv_pos_ki)  # KI_APR   位置环 Ki
            print(f"PID 参数已更新: pos_kp={pv_pos_kp} pos_ki={pv_pos_ki} "
                  f"vel_kp={pv_vel_kp} vel_ki={pv_vel_ki}")
            time.sleep(0.02)
        motor.send_pos_vel(pos, vlim)

    def do_vel(args: list) -> None:
        if not args:
            print("用法: vel <vel_rad_s>")
            return
        vel = float(args[0])
        motor.send_vel(vel)

    COMMANDS = {
        "enable": (do_enable, []),
        "disable": (do_disable, []),
        "set_zero": (do_set_zero, []),
        "state": (do_state, []),
        "mode": (do_mode, ""),
        "mit": (do_mit, "<pos_deg> [<vel> <kp> <kd> <tau>]"),
        "posvel": (do_posvel, "<pos_deg> [<vlim>]"),
        "vel": (do_vel, "<vel_rad_s>"),
    }

    print("\n命令: enable / disable / set_zero / mode / mit / posvel / vel / state / q")
    print("提示: mode 会自动在下一条控制指令前生效\n")

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()
            args = parts[1:]

            if cmd in ("q", "quit", "exit"):
                print("退出")
                break

            if cmd not in COMMANDS:
                print(f"未知命令: {cmd}，可用: {' / '.join(COMMANDS)}")
                continue

            fn, help_hint = COMMANDS[cmd]
            if help_hint and not args and fn in (do_mode, do_mit, do_posvel, do_vel):
                print(f"用法: {cmd} {help_hint}")
                continue

            try:
                if help_hint and fn not in (do_mode, do_mit, do_posvel, do_vel):
                    fn()
                elif fn == do_mode:
                    fn(args)
                elif fn == do_mit:
                    fn(args)
                elif fn == do_posvel:
                    fn(args)
                elif fn == do_vel:
                    fn(args)
                else:
                    fn()
            except Exception as e:
                print(f"错误: {e}")

    finally:
        ctrl.disable_all()
        ctrl.shutdown()
        ctrl.close()


if __name__ == "__main__":
    main()
