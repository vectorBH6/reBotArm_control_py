"""夹爪交互式测试工具。"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from reBotArm_control_py.actuator import Gripper

HELP = """
夹爪交互测试
-----------
z  - 设零
m  - 切换控制模式 (MIT / POS_VEL / VEL)
c  - 发送控制指令
s  - 显示当前状态
h  - 显示帮助
q  - 停止循环 → 失能 → 退出
"""


class GripperTerminal:
    def __init__(self):
        self.g = Gripper()
        self.g.enable()
        print(f"使能完成，当前模式: {self.g.mode}")
        self._show_state()

        self._target_pos = 0.0
        self._target_vel = 0.0
        self._mit_tau = 0.0

        self._running = True
        self.g.start_control_loop(self._loop, rate=100.0)
        print(f"控制循环已启动 {self.g._rate} Hz")

    def _loop(self, gripper, dt: float):
        if self.g.mode == "mit":
            self.g.mit(pos=self._target_pos, vel=self._target_vel, tau=self._mit_tau)
        elif self.g.mode == "pos_vel":
            self.g.pos_vel(pos=self._target_pos)
        elif self.g.mode == "vel":
            self.g.set_vel(vel=self._target_vel)

    def _show_state(self):
        pos, vel, torq = self.g.get_state(request=True)
        print(f"  pos={pos:+.4f} rad  vel={vel:+.4f} rad/s  torq={torq:+.4f} Nm  [mode={self.g.mode}]")

    def run(self):
        print(HELP)
        while self._running:
            try:
                cmd = input("\n> ").strip()
            except (EOFError, KeyboardInterrupt):
                cmd = "q"

            if not cmd:
                continue

            if cmd == "q":
                print("停止控制循环 → 失能 → 退出...")
                self.g.stop_control_loop()
                self.g.disable()
                self.g.disconnect()
                self._running = False
                break

            elif cmd == "h":
                print(HELP)

            elif cmd == "s":
                self._show_state()

            elif cmd == "z":
                print("设零...")
                self.g.set_zero()

            elif cmd == "m":
                print(f"当前模式: {self.g.mode}，切换到: [0]MIT  [1]POS_VEL  [2]VEL")
                sel = input("  > ").strip()
                if sel == "0":
                    self.g.mode_mit()
                    print("已切换到 MIT")
                elif sel == "1":
                    self.g.mode_pos_vel()
                    print("已切换到 POS_VEL")
                elif sel == "2":
                    self.g.mode_vel()
                    print("已切换到 VEL")
                else:
                    print("无效选择")

            elif cmd == "c":
                if self.g.mode == "mit":
                    try:
                        p = float(input("  pos (rad): ").strip() or "0.0")
                        v = float(input("  vel (rad/s) [0.0]: ").strip() or "0.0")
                        tau = float(input("  tau (Nm) [0.0]: ").strip() or "0.0")
                        self._target_pos, self._target_vel, self._mit_tau = p, v, tau
                        print(f"已更新: pos={p}, vel={v}, tau={tau}")
                    except ValueError:
                        print("输入无效")
                elif self.g.mode == "pos_vel":
                    try:
                        p = float(input("  pos (rad): ").strip() or "0.0")
                        self._target_pos = p
                        print(f"已更新: pos={p}")
                    except ValueError:
                        print("输入无效")
                elif self.g.mode == "vel":
                    try:
                        v = float(input("  vel (rad/s): ").strip() or "0.0")
                        self._target_vel = v
                        print(f"已更新: vel={v}")
                    except ValueError:
                        print("输入无效")
                self._show_state()

            else:
                print(f"未知指令: {cmd}，按 h 查看帮助")


if __name__ == "__main__":
    GripperTerminal().run()
