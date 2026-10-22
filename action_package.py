import os
import glob
import serial
from constants import Instruction_set


# This car is wired to physical pins 24/21 (GPIO8/9), which are UART4.
# Keep UART4 first; serial0 is the usual GPIO14/15 UART and is not this cable.
SERIAL_PORT_OVERRIDE = os.environ.get("CAR_SERIAL_PORT", "").strip()
SERIAL_BAUDRATE = int(os.environ.get("CAR_SERIAL_BAUDRATE", "9600"))


def _serial_candidates():
    if SERIAL_PORT_OVERRIDE:
        return [SERIAL_PORT_OVERRIDE]
    # When the motor controller is connected by USB it is normally an ACM
    # device.  GPIO UART is the fallback for the direct TX/RX wiring.
    # On Raspberry Pi Ubuntu, UART4 is exposed as ttyAMA1 (not ttyAMA4).
    return ["/dev/ttyAMA1", "/dev/ttyAMA4"] + sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*")) + [
        "/dev/serial0",
        "/dev/ttyS0",
    ]


def _open_motor_serial():
    errors = []
    for port in _serial_candidates():
        try:
            return serial.Serial(
                port,
                baudrate=SERIAL_BAUDRATE,
                timeout=0.2,
                write_timeout=0.5,
            )
        except (serial.SerialException, OSError) as exc:
            errors.append(f"{port}: {exc}")
    raise serial.SerialException("; ".join(errors) or "no serial device found")


def send_serial_data(Instruction: bytes):  # main_function
    ser = None
    try:
        ser = _open_motor_serial()
        ser.write(Instruction)
        ser.flush()
        print(f"serial sent {Instruction.hex()} via {ser.port}")
        return True
    except (serial.SerialException, OSError) as exc:
        print(f"not connect with car: {exc}")
        return False
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


def left():
    '''
        向左移动
        Param:
        second: 移动多少秒，默认移动2秒停止
    '''
    print("move left")
    send_serial_data(Instruction_set['q'])


def right():
    '''
        向右移动
        Param:
            second: 移动多少秒，默认移动2秒停止
    '''
    print("move right")
    send_serial_data(Instruction_set['e'])


def move_forward(second=5):
    '''
        前进
        Param:
            second: 移动多少秒，默认移动5秒停止
    '''
    print('move forward...')
    send_serial_data(Instruction_set['w'])


def backward(second=5):
    '''
        后退
        Param:
            second: 移动多少秒，默认移动5秒停止
    '''
    print("backward...")
    send_serial_data(Instruction_set['x'])


def decelerate():
    '''
        减速
    '''
    print("decelerate...")
    send_serial_data(Instruction_set['d'])


def accelerate():
    '''
        加速
    '''
    print("accelerate...")
    send_serial_data(Instruction_set['a'])


def suspend():
    '''
        停止
    '''
    print("suspend...")
    send_serial_data(Instruction_set['s'])
