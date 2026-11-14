"""ID 1～4を位置2048にそろえる。実行: python center_servos.py

組み立て前に、ホーンが自由に回る状態で実行してください。
ボードのジャンパーはB側。サーボ用の外部電源も接続します。
test.pyなど、COM10を使うプログラムは先に終了してください。
到着後もトルクを保持します。原点やサーボの設定は書き換えません。
"""

import struct
import sys
import time

import serial

# ---------- 設定 ----------
PORT = "COM10"
BAUDRATE = 115200
SERVO_IDS = [1, 2, 3, 4]
TARGET_POSITION = 2048
SPEED = 200
ACCELERATION = 10


# ---------- 通信処理 ----------
def send(usb, servo_id, instruction, parameters):
    body = bytes([servo_id, len(parameters) + 2, instruction]) + parameters
    checksum = bytes([~sum(body) & 255])
    usb.write(bytes([255, 255]) + body + checksum)
    usb.flush()


def read_data(usb, servo_id, address, size):
    usb.reset_input_buffer()
    send(usb, servo_id, 2, bytes([address, size]))
    reply = usb.read(size + 6)
    if len(reply) != size + 6:
        raise RuntimeError(f"ID {servo_id}: 応答がありません。電源・配線・IDを確認してください。")
    if reply[:4] != bytes([255, 255, servo_id, size + 2]) or sum(reply[2:]) & 255 != 255:
        raise RuntimeError(f"ID {servo_id}: 通信データが不正です。")
    if reply[4] != 0:
        raise RuntimeError(f"ID {servo_id}: サーボ異常 0x{reply[4]:02X}")
    return reply[5:-1]


def write_data(usb, servo_id, address, data):
    # SYNC WRITEで指定IDだけに書き込む。数値のリストで指定できる。
    parameters = bytes([address, len(data), servo_id]) + bytes(data)
    send(usb, 254, 131, parameters)


def read_position(usb, servo_id):
    raw = int.from_bytes(read_data(usb, servo_id, 56, 2), "little")
    return -(raw & 32767) if raw & 32768 else raw


def set_position(usb, servo_id, position):
    data = [ACCELERATION] + list(struct.pack("<HHH", position, 0, SPEED))
    write_data(usb, servo_id, 41, data)


# ---------- 全サーボを中央へ移動 ----------
def center_servos(usb):
    # 全IDの状態を先に確認し、不適切な設定のまま移動しないようにする。
    for servo_id in SERVO_IDS:
        if read_data(usb, servo_id, 33, 1)[0] != 0:
            raise RuntimeError(f"ID {servo_id}: 位置制御モードではありません。")
        minimum, maximum = struct.unpack("<HH", read_data(usb, servo_id, 9, 4))
        if not 0 <= minimum <= TARGET_POSITION <= maximum <= 4095:
            raise RuntimeError(f"ID {servo_id}: 位置2048が設定された可動範囲外です。")
        position = read_position(usb, servo_id)
        if not 0 <= position <= 4095:
            raise RuntimeError(f"ID {servo_id}: 現在位置が単回転の範囲外です。")
        print(f"ID {servo_id}: 現在位置 {position}")

    # 古い目標位置へ動かないよう、現在位置を目標にして保持を有効化する。
    for servo_id in SERVO_IDS:
        set_position(usb, servo_id, read_position(usb, servo_id))
        write_data(usb, servo_id, 40, [1])  # 1: トルク保持を有効化
        if read_data(usb, servo_id, 40, 1)[0] != 1:
            raise RuntimeError(f"ID {servo_id}: トルク保持を有効化できませんでした。")

    for servo_id in SERVO_IDS:
        set_position(usb, servo_id, TARGET_POSITION)
    print("ID 1～4を位置2048へ移動しています...")

    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        positions = {servo_id: read_position(usb, servo_id) for servo_id in SERVO_IDS}
        if all(abs(position - TARGET_POSITION) <= 5 for position in positions.values()):
            for servo_id, position in positions.items():
                print(f"ID {servo_id}: 到着（現在位置 {position}）")
            print("全サーボの位置合わせが完了しました。")
            return
        time.sleep(0.1)
    raise RuntimeError(f"30秒以内に到着しませんでした。現在位置: {positions}")


def main():
    try:
        print(f"接続先: {PORT} / {BAUDRATE}bps / 対象ID: {SERVO_IDS}")
        with serial.Serial(PORT, BAUDRATE, timeout=0.5, write_timeout=0.5) as usb:
            center_servos(usb)
        return 0
    except (serial.SerialException, RuntimeError) as error:
        print(f"エラー: {error}")
        return 1
    except KeyboardInterrupt:
        print("\n位置合わせの確認を中断しました。")
        return 1
    finally:
        print("終了操作ではサーボの移動やトルク保持は解除しません。")


if __name__ == "__main__":
    sys.exit(main())
