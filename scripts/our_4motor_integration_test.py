#!/usr/bin/env python3
"""roboparty RobotInterface 4 电机集成测试（config/our_4motor.yaml）。

流程：init_motors（驱动内部：失能→切MIT→使能）→ 保持 2s 确认 → 四电机
小幅扫动（can1 单台正弦 + can2 三台 120° 相位差）→ 读反馈 → deinit_motors。

⚠️ 输出轴全部空载；Ctrl+C 全部失能。
"""
import os
import sys
import time
import math

import robot_py

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "config", "our_4motor.yaml")
POS_RANGE = float(os.environ.get("POS_RANGE", "0.2"))
PERIOD = float(os.environ.get("PERIOD", "4.0"))
SWEEP_CYCLES = int(os.environ.get("SWEEP_CYCLES", "3"))
HOLD_TIME = 2.0


def main():
    print("=" * 60)
    print(f"roboparty 集成测试 · {CONFIG}")
    print(f"±{POS_RANGE} rad, 周期 {PERIOD:.1f}s × {SWEEP_CYCLES}, 保持 {HOLD_TIME}s")
    print("⚠️ 输出轴全部空载。随时 Ctrl+C 全部失能")
    print("=" * 60)

    robot = robot_py.RobotInterface(CONFIG)

    print("\n[1] init_motors（驱动内部：失能→切MIT→使能）...")
    robot.init_motors()
    time.sleep(0.5)
    robot.read_joints()
    q0 = robot.get_joint_q()
    print(f"    初始位置: {[f'{q:+.3f}' for q in q0]}")

    # 2. 保持确认
    print(f"[2] 保持 {HOLD_TIME}s（观察 4 台 LED 应变绿）...")
    t0 = time.time()
    while time.time() - t0 < HOLD_TIME:
        robot.apply_action(q0)
        time.sleep(0.005)

    # 3. 扫动：can2 三台 120° 相位差，can1 单台正弦
    print(f"[3] 扫动（can1 正弦 + can2 三台 120° 相位差）...")
    # 电机顺序（setup_motors 按接口顺序）：idx0=can1 ID1；idx1/2/3=can2 ID1/2/3
    phases = [0.0, 0.0, 2 * math.pi / 3, 4 * math.pi / 3]
    t0 = time.time()
    done = False
    cycles_done = 0
    try:
        while not done:
            t = time.time() - t0
            phase = (t % PERIOD) / PERIOD * 2 * math.pi
            q = [q0[i] + POS_RANGE * math.sin(phase + phases[i]) for i in range(4)]
            robot.apply_action(q)
            time.sleep(0.005)
            robot.read_joints()
            c = int(t / PERIOD)
            if c >= SWEEP_CYCLES:
                done = True
            elif c != cycles_done:
                cycles_done = c
                qq = robot.get_joint_q()
                tau = robot.get_joint_tau()
                print(f"  [{c}/{SWEEP_CYCLES}] q={[f'{x:+.3f}' for x in qq]} "
                      f"tau={[f'{x:+.2f}' for x in tau]}")
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        print("\n[4] deinit_motors（全部失能）...")
        robot.deinit_motors()
        print("完成 ✓")
    return 0


if __name__ == "__main__":
    sys.exit(main())
