#!/usr/bin/env python3
import os
import sys
import yaml
import motors_py
import time
import termios
import tty


def read_key_nonblocking(timeout=0.05):
    import select
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        if select.select([sys.stdin], [], [], timeout)[0]:
            ch = sys.stdin.read(1)
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def load_config(config_path: str) -> dict:
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    return config


def create_motors(config: dict) -> list:
    motors = []
    motor_ids = config['motor_id']
    motor_interface_type = config['motor_interface_type']
    motor_interfaces = config['motor_interface']
    motor_num = config['motor_num']
    motor_type = config['motor_type']
    motor_models = config['motor_model']
    master_id_offset = config['master_id_offset']
    motor_zero_offsets = config['motor_zero_offset']
    
    motor_idx = 0
    for interface_idx, num in enumerate(motor_num):
        interface = motor_interfaces[interface_idx]
        for _ in range(num):
            if motor_idx >= len(motor_ids):
                break
            motor_id = motor_ids[motor_idx]
            motor_model = motor_models[motor_idx] if motor_idx < len(motor_models) else 0
            motor_zero_offset = motor_zero_offsets[motor_idx] if motor_idx < len(motor_zero_offsets) else 0.0
            
            motor = motors_py.MotorDriver.create_motor(
                motor_id=motor_id,
                interface_type=motor_interface_type,
                interface=interface,
                motor_type=motor_type,
                motor_model=motor_model,
                master_id_offset=master_id_offset,
                motor_zero_offset=motor_zero_offset,
            )
            motors.append({
                'motor': motor,
                'motor_id': motor_id,
                'interface': interface,
                'index': motor_idx
            })
            motor_idx += 1
    
    return motors


def calibrate_motor(motor_info: dict) -> str:
    """标定一台电机的零点。

    Returns:
        "ok"     标零成功（驱动校验通过）
        "failed" 标零失败（驱动返回 False / 标完读数仍超容差）
        "skipped" 用户按空格跳过

    ⚠️ 2026-09-17 修复：原先 `motor.set_motor_zero()` 的**返回值被丢弃**，
    只要按了 Enter 就一律记为"已标零"。而驱动内部其实是校验过的
    （`dm_motor_driver.cpp` 的 set_motor_zero_dm：标完后读回位置，
    只要 |pos| > judgment_accuracy_threshold 就 return false）。
    ⇒ 症状是"标完零后发现偏差仍在"，且**不知道是哪台失败了**。
    见 robot-deploy-toolkit/docs/02-标定 的常见症状表。
    """
    motor = motor_info['motor']
    motor_id = motor_info['motor_id']
    interface = motor_info['interface']

    print(f"\n{'='*50}")
    print(f"正在标定电机 ID: {motor_id} (接口: {interface})")
    print(f"{'='*50}")

    print("使能电机...")
    motor.init_motor()
    time.sleep(0.3)

    print("设置纯阻尼控制模式...")
    motor.set_motor_control_mode(motors_py.MotorControlMode.MIT)
    time.sleep(0.1)

    print("\n>>> 请手动将电机摆到零位 <<<")
    print("提示: 电机现在处于阻尼模式，可以自由转动")
    print("操作: 按 [Enter] 确认标零 | 按 [空格] 跳过此电机")

    result = "skipped"
    try:
        while True:
            motor.motor_mit_cmd(0.0, 0.0, 0.0, 1.0, 0.0)

            pos = motor.get_motor_pos()
            err = motor.get_error_id()
            print(f"\r当前位置: {pos:+.6f} rad | 错误码: {err} | [Enter]标零 / [空格]跳过", end='', flush=True)

            key = read_key_nonblocking(0.05)
            if key == '\r' or key == '\n':  # Enter
                # ⭐ 必须接返回值：驱动内部会读回校验，失败返回 False
                ok = motor.set_motor_zero()
                time.sleep(0.3)
                after = motor.get_motor_pos()
                if ok:
                    # 二次确认：驱动说成功，再看一眼读回来的位置
                    # ⚠️ 0.05 这个数在【三处】各写了一遍，改一处要记得改另外两处：
                    #      robot-deploy-toolkit/scripts/probe_set_zero.py
                    #      robot-deploy-toolkit/scripts/probe_set_zero_all.py
                    # ⚠️ 而驱动自己的判据是 0.01（motor_driver.hpp 的
                    #    judgment_accuracy_threshold）⇒ 这里比驱动松 5 倍，
                    #    只用来抓"驱动放行之后、隔 0.3s 又漂走"，不是更严的闸。
                    if abs(after) < 0.05:
                        print(f"\n  ✅ 标零成功（标后读数 {after:+.6f} rad）")
                        result = "ok"
                    else:
                        print(f"\n  ⚠️ 驱动报成功，但标后读数 {after:+.6f} rad 偏大 —— 请复核")
                        result = "failed"
                else:
                    print(f"\n  ❌ 标零【失败】（驱动返回 False，标后读数 {after:+.6f} rad）")
                    result = "failed"
                break
            elif key == ' ':  # Space
                result = "skipped"
                break
            elif key == '\x03':  # Ctrl+C
                raise KeyboardInterrupt

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n\n用户中断标定")
        motor.deinit_motor()
        raise

    print("失能电机...")
    motor.deinit_motor()
    time.sleep(0.2)

    return result


def main():
    import argparse

    script_dir = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="电机零点标定（把当前姿态写进电机硬件零点，不可逆）")
    ap.add_argument(
        "--config",
        default=os.path.join(script_dir, 'config', 'set_zero.yaml'),
        help="电机配置文件（默认 scripts/config/set_zero.yaml；dm10 用 set_zero_dm10.yaml）",
    )
    ap.add_argument("-y", "--yes", action="store_true", help="跳过开头的确认（非交互环境用）")
    args = ap.parse_args()
    config_path = args.config

    print("="*60)
    print("           电机零点标定工具")
    print("="*60)
    print(f"\n配置文件: {config_path}\n")
    print("⚠️ 本操作把【当前姿态】写进电机硬件零点，**不可逆**。")
    print("   标定前请确认：机器人已架住/吊住、姿态已摆到刻线、旁边有人。\n")

    try:
        config = load_config(config_path)
    except Exception as e:
        print(f"加载配置文件失败: {e}")
        return 1
    
    print("配置信息:")
    print(f"  - 电机ID列表: {config['motor_id']}")
    print(f"  - 电机类型: {config['motor_type']}")
    print(f"  - 接口类型: {config['motor_interface_type']}")
    print(f"  - 接口: {config['motor_interface']}")
    print(f"  - 电机型号: {config['motor_model']}")
    print("\n" + "-"*60)
    if not args.yes:
        input("按 Enter 开始标定流程...")
    print("-"*60)
    
    try:
        motors = create_motors(config)
        print(f"\n成功创建 {len(motors)} 个电机对象")
    except Exception as e:
        print(f"创建电机失败: {e}")
        return 1
    
    records = []
    try:
        zeroed_ids, failed_ids, skipped_ids = [], [], []
        for motor_info in motors:
            result = calibrate_motor(motor_info)
            records.append({
                "index": motor_info['index'],
                "motor_id": motor_info['motor_id'],
                "interface": motor_info['interface'],
                "result": result,
            })
            if result == "ok":
                zeroed_ids.append(motor_info['motor_id'])
                print(f"电机 {motor_info['motor_id']} 标定完成!")
            elif result == "failed":
                failed_ids.append(motor_info['motor_id'])
                print(f"电机 {motor_info['motor_id']} 标定【失败】—— 请重试")
            else:
                skipped_ids.append(motor_info['motor_id'])
                print(f"电机 {motor_info['motor_id']} 已跳过")
    except KeyboardInterrupt:
        print("\n\n标定被用户中断")
        _dump_records(records, config_path)
        return 1
    except Exception as e:
        print(f"\n标定过程出错: {e}")
        _dump_records(records, config_path)
        return 1

    print("\n" + "="*60)
    print("         标定流程完成!")
    print("="*60)
    if zeroed_ids:
        print(f"  ✅ 已标零: {zeroed_ids}")
    if failed_ids:
        print(f"  ❌ 失败(需重试): {failed_ids}")
    if skipped_ids:
        print(f"  ⏭  已跳过: {skipped_ids}")

    out = _dump_records(records, config_path)
    if out:
        print(f"\n📄 记录已落盘: {out}")

    print("\n标定流程结束")
    return 0 if not failed_ids else 1


def _dump_records(records: list, config_path: str) -> str | None:
    """落盘标定记录。

    ⚠️ 2026-09-17 新增：原先**完全没有落盘** —— 标完只 print，关掉终端就没了，
    无法追溯"哪台什么时候标的、成没成、标后读数多少"。
    （对照 `scripts/joint_test.py` 就是落盘的，本脚本此前没有。）
    """
    if not records:
        return None
    import json
    from datetime import datetime

    payload = {
        "schema": "dm-set-zero-record/1",
        "created": datetime.now().isoformat(),
        "config": os.path.abspath(config_path),
        "note": "set_motor_zero() 把【当前姿态】写进电机硬件零点；不可逆、需谨慎",
        "results": records,
    }
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"set_zero_{datetime.now():%Y%m%d-%H%M%S}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    return os.path.abspath(path)


if __name__ == '__main__':
    sys.exit(main())
