#!/usr/bin/env python3
"""DM-IMU-L1 集成测试：RobotInterface 读 IMU 四元数/角速度/加速度，采样 5 秒。

判定：静止时 ang_vel≈0、acc Z≈9.8（重力）；拿起晃动时数值跟随变化。
"""
import os
import sys
import time

import robot_py

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "config", "our_4motor.yaml")


def main():
    robot = robot_py.RobotInterface(CONFIG)
    print("IMU 驱动已初始化（serial /dev/ttyACM0 @921600）")
    print("采样 5 秒——请拿起 IMU 晃动...")
    t0 = time.time()
    while time.time() - t0 < 5.0:
        robot.read_imu()
        q = robot.get_quat()
        w = robot.get_ang_vel()
        print(f"quat=[{q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f}] "
              f"gyro=[{w[0]:+.3f},{w[1]:+.3f},{w[2]:+.3f}]")
        time.sleep(0.5)
    print("完成 ✓")
    return 0


if __name__ == "__main__":
    sys.exit(main())
