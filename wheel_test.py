# -*- coding: utf-8 -*-
"""小车轮子测试: 串口发指令, 每个动作 2 秒后停止, 观察轮子是否转动
用法: cd ~/car && venv/bin/python wheel_test.py
前提: 3588 UART8(TX/RX/GND)已接小车控制板, 小车已供电
"""
import sys
import time

try:
    from action_package import send_serial_data
    from constants import Instruction_set
except ImportError:
    print("请 cd 到 ~/car 目录再运行 (依赖 action_package.py / constants.py)")
    sys.exit(1)


def run(key, name, sec=2):
    print(">>> 指令[%s] %s, 持续 %d 秒" % (key, name, sec))
    try:
        send_serial_data(Instruction_set[key])
        time.sleep(sec)
        send_serial_data(Instruction_set['s'])
        print("    已发停止指令")
    except Exception as e:
        print("    发送失败:", type(e).__name__, e)
        return False
    time.sleep(0.8)
    return True


print("=== 小车轮子测试 (串口 /dev/ttyS8) ===")
print("确认: 3588 UART8 TX -> 小车RX, GND 共地, 小车已供电")
print("10 秒后开始\n")
time.sleep(10)

ok = True
ok &= run('w', '前进')
ok &= run('e', '右转')
ok &= run('q', '左转')
ok &= run('x', '后退')
print("\n=== 测试完成 ===")
if ok:
    print("指令全部发出。轮子有转动 = 接线成功; 完全不动 = 查接线/供电/波特率")
else:
    print("有指令发送失败, 看上面错误 (串口权限? dialout 组?)")
