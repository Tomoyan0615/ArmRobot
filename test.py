"""STS3215をUSB接続で操作する。実行: python test.py

準備: python -m pip install pyserial pygame
ボードのジャンパーはB側。サーボ定格に合う外部電源も接続する。
角度はサーボの絶対角度。アームに取り付けた場合は可動範囲に注意。
通信仕様: https://docs.waveshare.com/Servo_Communication_Protocol
"""

import math
import struct
import time

import serial
import pygame
from xbox_controller import Gamepad

# ---------- 設定（変更するときはここだけ編集） ----------
PORT = "COM10"
BAUDRATE = 115200
SERVO_IDS = [1, 2, 3]
ID1_MIN_ANGLE = 90    # ID1の操作範囲（度）
ID1_MAX_ANGLE = 270
ID2_MIN_ANGLE = 125   # ID2の操作範囲（度）
ID2_MAX_ANGLE = 210
ID3_MIN_ANGLE = 135   # ID3の絶対範囲（相対制限との共通範囲で動かす）
ID3_MAX_ANGLE = 225
# 向かい合わせのID2・3は角度の和で制限を計算する。
# 相対制限だけならID2=180度のときID3=125～215度。和は305～395度。
ID2_ID3_MIN_ANGLE_SUM = 305
ID2_ID3_MAX_ANGLE_SUM = 395
SPEED = 1000          # 速度の設定値（1～32767）
ACCELERATION = 100    # 加速度の設定値（1～254）
GRIPPER_ID = 4
# 手先の開閉位置（0～4095ステップ。1回転=4096ステップ）
GRIPPER_OPEN_POSITION = 2300    # 開く位置
GRIPPER_CLOSE_POSITION = 1750   # 閉じる位置
GRIPPER_SPEED = 200            # ID4だけゆっくり動かす
GRIPPER_ACCELERATION = 20
GRIPPER_CLOSE_STEP = 10        # 1回の確認で閉じる最大ステップ数
GRIPPER_POLL_INTERVAL = 0.05   # 負荷を確認する間隔（秒）
GRIPPER_LOAD_THRESHOLD = 200   # 負荷しきい値（0.1%単位。200=20%。実機で調整）
GRIPPER_LOAD_SAMPLES = 2       # 連続して超えた回数。瞬間的な変動を除く
GRIPPER_CLOSE_TIMEOUT = 15     # 閉じ動作の最大時間（秒）
GAMEPAD_SPEED = 300           # スティック最大入力時の位置変更量（ステップ/秒）
GAMEPAD_ID1_DIRECTION = -1     # ID1の回転方向（1または-1）
GAMEPAD_LOOKAHEAD = 30         # 現在位置から先に指令する最大ステップ数
JOINT_POLL_INTERVAL = 0.05     # ID2・3の位置確認間隔（秒）


# ---------- 通信処理 ----------
def send(usb, servo_id, instruction, parameters):
    """命令をサーボの通信形式に変換して送る。"""
    body = bytes([servo_id, len(parameters) + 2, instruction]) + parameters
    checksum = bytes([~sum(body) & 0xFF])
    usb.write(bytes([255, 255]) + body + checksum)
    usb.flush()


def read_data(usb, servo_id, address, size):
    """指定アドレスからデータを読み取る。"""
    usb.reset_input_buffer()
    send(usb, servo_id, 0x02, bytes([address, size]))
    reply = usb.read(size + 6)  # ヘッダー等の6バイト + データ
    if len(reply) != size + 6:
        raise RuntimeError(f"ID {servo_id}: 応答がありません。電源・ジャンパーB・サーボIDを確認してください。")
    if reply[:4] != bytes([0xFF, 0xFF, servo_id, size + 2]):
        raise RuntimeError(f"ID {servo_id}: 応答の形式が不正です。配線・IDの重複を確認してください。")
    if sum(reply[2:]) & 0xFF != 0xFF:
        raise RuntimeError(f"ID {servo_id}: 通信データが破損しています。配線を確認してください。")
    if reply[4] != 0:
        raise RuntimeError(f"ID {servo_id}: サーボが異常を検出しました: 0x{reply[4]:02X}")
    return reply[5:-1]


def write_data(usb, servo_id, address, data):
    """数値リストやバイト列を指定IDに書き込む（応答なし）。"""
    data = bytes(data)
    parameters = bytes([address, len(data), servo_id]) + data
    send(usb, 0xFE, 0x83, parameters)


def read_position(usb, servo_id):
    """現在位置をステップ単位で返す（1回転=4096ステップ）。"""
    raw = int.from_bytes(read_data(usb, servo_id, 56, 2), "little")
    return -(raw & 0x7FFF) if raw & 0x8000 else raw


# ---------- サーボの操作 ----------
def set_position(usb, servo_id, position):
    """到着を待たず、目標位置・速度・加速度を送る。"""
    speed = GRIPPER_SPEED if servo_id == GRIPPER_ID else SPEED
    acceleration = GRIPPER_ACCELERATION if servo_id == GRIPPER_ID else ACCELERATION
    data = [acceleration] + list(struct.pack("<HHH", position, 0, speed))
    write_data(usb, servo_id, 41, data)


def enable_torque(usb, servo_id):
    # 古い目標位置へ動かないよう、現在位置を目標にしてから保持を有効化。
    position = read_position(usb, servo_id)
    if not 0 <= position <= 4095:
        raise RuntimeError(f"ID {servo_id}: 現在位置が単回転の範囲外です。設定を確認してください。")
    set_position(usb, servo_id, position)
    write_data(usb, servo_id, 40, [1])  # 1: トルク保持を有効化
    if read_data(usb, servo_id, 40, 1)[0] != 1:
        raise RuntimeError(f"ID {servo_id}: トルク保持を有効化できませんでした。")


# ---------- スライダー画面 ----------
EXIT_BUTTON = pygame.Rect(670, 843, 180, 48)
GRIPPER_BUTTON = pygame.Rect(430, 711, 235, 48)
GRIPPER_OFF_BUTTON = pygame.Rect(710, 711, 145, 48)


def create_gripper(usb):
    """ID4の開閉位置を確認する。起動時には移動しない。"""
    gripper = {"ready": False, "enabled": False, "next_closed": True,
               "current": None, "load": 0, "closing": False, "load_hits": 0,
               "last_feedback": 0, "close_started": 0,
               "status": "コード冒頭に開く位置・閉じる位置を設定してください。"}
    opened, closed = GRIPPER_OPEN_POSITION, GRIPPER_CLOSE_POSITION
    if any(not isinstance(position, int) or not 0 <= position <= 4095 for position in (opened, closed)):
        gripper["status"] = "開閉位置は0～4095の整数で設定してください。"
        return gripper
    minimum, maximum = struct.unpack("<HH", read_data(usb, GRIPPER_ID, 9, 4))
    if opened == closed or not (0 <= minimum <= opened <= maximum <= 4095 and minimum <= closed <= maximum):
        gripper["status"] = "開閉位置は異なる値で、ID4の可動範囲内に設定してください。"
        return gripper
    if read_data(usb, GRIPPER_ID, 33, 1)[0] != 0:
        raise RuntimeError(f"ID {GRIPPER_ID}: 位置制御モードではありません。")
    current = read_position(usb, GRIPPER_ID)
    if not 0 <= current <= 4095:
        raise RuntimeError(f"ID {GRIPPER_ID}: 現在位置が単回転の範囲外です。")
    gripper.update(ready=True, opened=opened, closed=closed, current=current,
                   status="最初のクリックで閉じます。起動時の位置は変更しません。")
    return gripper


def toggle_gripper(usb, gripper):
    """閉じるときは負荷監視を開始。次のクリックでは開く。"""
    if not gripper["ready"]:
        return
    closing = gripper["next_closed"]
    if not gripper["enabled"]:
        enable_torque(usb, GRIPPER_ID)
        gripper["enabled"] = True
    gripper["closing"] = closing
    gripper["load_hits"] = 0
    gripper["next_closed"] = not closing
    if closing:
        gripper["close_started"] = time.monotonic()
        gripper["last_feedback"] = 0  # 次の画面更新ですぐ負荷を確認する
        gripper["status"] = "負荷を監視しながら閉じています。"
    else:
        set_position(usb, GRIPPER_ID, gripper["opened"])
        gripper["status"] = f"開いています（目標位置 {gripper['opened']}）。"


def hold_gripper(usb, gripper, reason):
    """現在位置を新しい目標にして、閉じる動作を止めて保持する。"""
    if not 0 <= gripper["current"] <= 4095:
        raise RuntimeError(f"ID {GRIPPER_ID}: 現在位置が単回転の範囲外です。")
    set_position(usb, GRIPPER_ID, gripper["current"])
    gripper["closing"] = False
    gripper["load_hits"] = 0
    gripper["status"] = f"{reason}: 位置 {gripper['current']} で保持"


def update_gripper(usb, gripper, now):
    """位置と負荷をまとめて読み、閉じている間だけ接触を判定する。"""
    if not gripper["ready"] or now - gripper["last_feedback"] < GRIPPER_POLL_INTERVAL:
        return
    # 56～61: 現在位置(2バイト)、現在速度(2バイト)、現在負荷(2バイト)
    position, _, raw_load = struct.unpack("<HHH", read_data(usb, GRIPPER_ID, 56, 6))
    position = -(position & 32767) if position & 32768 else position
    if not 0 <= position <= 4095:
        raise RuntimeError(f"ID {GRIPPER_ID}: 現在位置が単回転の範囲外です。")
    gripper["current"] = position
    # 負荷はBIT10が方向。下位10ビットが大きさ（0.1%単位）。実測トルクではない。
    gripper["load"] = raw_load & 1023
    gripper["last_feedback"] = now
    if not gripper["closing"]:
        return
    if gripper["load"] >= GRIPPER_LOAD_THRESHOLD:
        gripper["load_hits"] += 1
        if gripper["load_hits"] >= GRIPPER_LOAD_SAMPLES:
            hold_gripper(usb, gripper, "負荷検知")
        # 確認中も、それ以上閉じる目標は送らない。
        return
    gripper["load_hits"] = 0
    if now - gripper["close_started"] >= GRIPPER_CLOSE_TIMEOUT:
        hold_gripper(usb, gripper, "時間切れ")
        return
    remaining = gripper["closed"] - position
    if abs(remaining) <= 3:
        gripper["closing"] = False
        gripper["status"] = "閉じ位置に到着（把持は未検知）。次のクリックで開きます。"
        return
    # 全閉位置を一度に送らず、現在位置から最大10ステップだけ先に進める。
    step = max(-GRIPPER_CLOSE_STEP, min(GRIPPER_CLOSE_STEP, remaining))
    set_position(usb, GRIPPER_ID, position + step)


def create_controls(usb):
    """各IDの可動範囲・現在位置を読み取り、スライダーを用意する。"""
    controls = []
    angle_limits = {1: (ID1_MIN_ANGLE, ID1_MAX_ANGLE),
                    2: (ID2_MIN_ANGLE, ID2_MAX_ANGLE),
                    3: (ID3_MIN_ANGLE, ID3_MAX_ANGLE)}
    for index, servo_id in enumerate(SERVO_IDS):
        if read_data(usb, servo_id, 33, 1)[0] != 0:
            raise RuntimeError(f"ID {servo_id}: 位置制御モードではありません。")
        minimum, maximum = struct.unpack("<HH", read_data(usb, servo_id, 9, 4))
        if not 0 <= minimum < maximum <= 4095:
            raise RuntimeError(f"ID {servo_id}: 単回転の可動範囲が設定されていません。")
        if servo_id in angle_limits:
            # サーボ本体とアプリ側の角度制限を両方守る。
            min_angle, max_angle = angle_limits[servo_id]
            minimum = max(minimum, math.ceil(min_angle * 4096 / 360))
            maximum = min(maximum, math.floor(max_angle * 4096 / 360))
            if minimum >= maximum:
                raise RuntimeError(f"ID {servo_id}: サーボの可動範囲と指定した角度範囲が重なりません。")
        current = read_position(usb, servo_id)
        if not 0 <= current <= 4095:
            raise RuntimeError(f"ID {servo_id}: 現在位置が単回転の範囲外です。")
        y = 110 + index * 190
        controls.append({
            "id": servo_id, "minimum": minimum, "maximum": maximum,
            "base_minimum": minimum, "base_maximum": maximum, "blocked": False,
            "current": current, "target": max(minimum, min(maximum, current)),
            "sent_target": current,
            "pending": False, "enabled": False, "last_send": 0,
            "gamepad_active": False, "gamepad_target": float(current),
            "status": "スライダーを操作すると動きます。",
            "slider": pygame.Rect(170, y + 66, 500, 32),
            "off_button": pygame.Rect(710, y + 55, 145, 46),
        })
    update_joint_limits(controls)
    return controls


def id3_position_limits(id2_position, id2_target=None):
    """他方の現在位置と移動先から許可範囲を計算する。和なのでID2側にも使える。"""
    if id2_target is None:
        id2_target = id2_position
    # 移動中も範囲を守れるよう、他方の現在位置～移動先の全体を考慮する。
    minimum = math.ceil(ID2_ID3_MIN_ANGLE_SUM * 4096 / 360) - min(id2_position, id2_target)
    maximum = math.floor(ID2_ID3_MAX_ANGLE_SUM * 4096 / 360) - max(id2_position, id2_target)
    return minimum, maximum


def update_joint_limits(controls):
    """ID2・3の表示・入力範囲を更新する。サーボには命令を送らない。"""
    joints = {control["id"]: control for control in controls}
    if 2 not in joints or 3 not in joints:
        return
    for servo_id, other_id in ((2, 3), (3, 2)):
        control, other = joints[servo_id], joints[other_id]
        other_target = other["sent_target"] if other["enabled"] else other["current"]
        lower, upper = id3_position_limits(other["current"], other_target)
        lower = max(control["base_minimum"], lower)
        upper = min(control["base_maximum"], upper)
        was_blocked = control["blocked"]
        control["blocked"] = lower > upper
        if control["blocked"]:
            # 自動で他方を動かさず、この軸の新しい移動を受け付けない。
            lower = upper = max(control["base_minimum"], min(control["base_maximum"], control["current"]))
            control["status"] = f"移動不可 / ID{other_id}の位置を確認してください"
        elif was_blocked:
            control["status"] = "可動範囲が戻りました。操作できます。"
        control["minimum"], control["maximum"] = lower, upper


def refresh_joint_positions(usb, controls):
    for control in controls:
        if control["id"] in (2, 3):
            position = read_position(usb, control["id"])
            if not 0 <= position <= 4095:
                raise RuntimeError(f"ID {control['id']}: 現在位置が単回転の範囲外です。")
            control["current"] = position
    update_joint_limits(controls)


def send_arm_target(usb, controls, control, now):
    """送信直前にも範囲を確認し、操作されたIDだけを動かす。"""
    if control["id"] in (2, 3):
        refresh_joint_positions(usb, controls)
    control["pending"] = False
    if control["blocked"]:
        return
    requested = control["target"]
    target = max(control["minimum"], min(control["maximum"], requested))
    control["target"] = target
    if target != requested:
        control["gamepad_target"] = float(target)
    if not control["enabled"]:
        enable_torque(usb, control["id"])
        control["enabled"] = True
    set_position(usb, control["id"], target)
    control["sent_target"] = target
    control["last_send"] = now
    control["status"] = "角度制限で移動先を調整 / 保持中" if target != requested else "トルク保持中 / 目標位置へ移動"
    update_joint_limits(controls)


def slider_position(mouse_x, control):
    """マウス位置を、そのサーボの可動範囲内の位置に変換する。"""
    slider = control["slider"]
    ratio = max(0, min(1, (mouse_x - slider.left) / slider.width))
    return round(control["minimum"] + ratio * (control["maximum"] - control["minimum"]))


def hold_arm(usb, control):
    position = read_position(usb, control["id"])
    if not 0 <= position <= 4095:
        raise RuntimeError(f"ID {control['id']}: 現在位置が単回転の範囲外です。")
    set_position(usb, control["id"], position)
    control.update(current=position, target=position, sent_target=position, gamepad_target=float(position),
                   gamepad_active=False, pending=False, status="操作停止 / 現在位置を保持")


def apply_gamepad(usb, controls, gripper, inputs, elapsed):
    """スティックは倒した量に応じた速度で位置を変更する。"""
    if inputs["stop"] or inputs["interrupted"]:
        for control in controls:
            if control["enabled"]:
                hold_arm(usb, control)
            control["pending"] = control["gamepad_active"] = False
        if gripper["enabled"]:
            gripper["current"] = read_position(usb, GRIPPER_ID)
            hold_gripper(usb, gripper, "操作停止")
        return
    if inputs["toggle"]:
        toggle_gripper(usb, gripper)
    # 通信待ちなどで時間が空いても、目標位置を一度に大きく変更しない。
    elapsed = max(0, min(0.05, elapsed))
    for control in controls:
        value = inputs["axes"].get(control["id"], 0)
        if control["id"] == 1:
            value *= GAMEPAD_ID1_DIRECTION
        if control.get("blocked", False):
            control["pending"] = False
            continue
        if value == 0:
            if control["gamepad_active"]:
                hold_arm(usb, control)
            continue
        if not control["gamepad_active"]:
            position = read_position(usb, control["id"])
            if not 0 <= position <= 4095:
                raise RuntimeError(f"ID {control['id']}: 現在位置が単回転の範囲外です。")
            control["current"] = position
            control["gamepad_target"] = float(position)
            control["gamepad_active"] = True
        target = control["gamepad_target"] + value * GAMEPAD_SPEED * elapsed
        lower = max(control["minimum"], control["current"] - GAMEPAD_LOOKAHEAD)
        upper = min(control["maximum"], control["current"] + GAMEPAD_LOOKAHEAD)
        target = max(control["minimum"], min(control["maximum"], target))
        if lower <= upper:
            target = max(lower, min(upper, target))
        control["gamepad_target"] = target
        if round(target) != control["target"]:
            control["target"] = round(target)
            control["pending"] = True


def draw_screen(screen, font, large_font, controls, gripper, gamepad_status="マウス操作"):
    screen.fill((22, 28, 40))

    def text(message, x, y, color=(230, 237, 247), large=False):
        image = (large_font if large else font).render(message, True, color)
        screen.blit(image, (x, y))

    text("STS3215  /  アーム・手先の操作", 40, 12, large=True)
    text(f"{PORT}  |  {BAUDRATE} bps  |  ID 1 / 2 / 3 + 手先 ID {GRIPPER_ID}", 40, 50, (154, 170, 194))
    text(gamepad_status, 40, 79, (112, 225, 172))
    for control in controls:
        slider = control["slider"]
        row_y = slider.top - 66
        minimum, maximum = control["minimum"], control["maximum"]
        target, current = control["target"], control["current"]
        pygame.draw.rect(screen, (30, 38, 52), (40, row_y, 820, 172), border_radius=12)
        text(f"ID {control['id']}", 60, row_y + 22, large=True)
        text(f"目標 {target * 360 / 4096:.1f}°", 170, row_y + 12, large=True)
        text(f"現在 {current * 360 / 4096:.1f}°", 460, row_y + 20)

        span = max(1, maximum - minimum)
        x = slider.left + round(max(0, min(1, (target - minimum) / span)) * slider.width)
        actual_x = slider.left + round(max(0, min(1, (current - minimum) / span)) * slider.width)
        y = slider.centery
        pygame.draw.line(screen, (66, 78, 99), (slider.left, y), (slider.right, y), 8)
        pygame.draw.line(screen, (70, 171, 234), (slider.left, y), (x, y), 8)
        pygame.draw.circle(screen, (244, 247, 255), (x, y), 13)
        pygame.draw.line(screen, (112, 225, 172), (actual_x, y + 20), (actual_x, y + 28), 3)
        text(f"{minimum * 360 / 4096:.1f}°", slider.left, row_y + 112)
        text(f"{maximum * 360 / 4096:.1f}°", slider.right - 85, row_y + 112)
        button = control["off_button"]
        pygame.draw.rect(screen, (126, 63, 55), button, border_radius=8)
        text("保持解除", button.x + 28, button.y + 9)
        text(control["status"], 170, row_y + 142, (112, 225, 172))

    pygame.draw.rect(screen, (30, 38, 52), (40, 682, 820, 146), border_radius=12)
    text(f"ID {GRIPPER_ID}", 60, 703, large=True)
    text("手先の開閉", 170, 696, large=True)
    current_label = "未設定" if gripper["current"] is None else f"{gripper['current'] * 360 / 4096:.1f}°"
    text(f"現在 {current_label}", 170, 737)
    color = (40, 121, 158) if gripper["ready"] else (66, 73, 85)
    pygame.draw.rect(screen, color, GRIPPER_BUTTON, border_radius=8)
    label = ("閉じる" if gripper["next_closed"] else "開く") if gripper["ready"] else "開閉位置を確認"
    text(label, GRIPPER_BUTTON.x + 30, GRIPPER_BUTTON.y + 9)
    pygame.draw.rect(screen, (126, 63, 55), GRIPPER_OFF_BUTTON, border_radius=8)
    text("保持解除", GRIPPER_OFF_BUTTON.x + 28, GRIPPER_OFF_BUTTON.y + 9)
    text(f"負荷 {gripper['load'] / 10:.1f}% / 検知しきい値 {GRIPPER_LOAD_THRESHOLD / 10:.1f}%", 430, 769)
    text(gripper["status"], 170, 800, (112, 225, 172))

    pygame.draw.rect(screen, (53, 65, 86), EXIT_BUTTON, border_radius=8)
    text("終了 / Esc", EXIT_BUTTON.x + 32, EXIT_BUTTON.y + 9)
    text("保持解除時はアームの落下に注意してください。", 40, 840, (255, 192, 145))
    text("左スティック: ID1・2 / 右スティック上下: ID3 / A: ハンド開閉", 40, 882, (154, 170, 194))
    text("B: 全軸停止・保持 / 終了後も保持 / 緑の印: 現在位置", 40, 908, (154, 170, 194))
    pygame.display.flip()


def run_slider(usb, screen, font, large_font, gamepad=None):
    controls = create_controls(usb)
    gripper = create_gripper(usb)
    dragging = None  # 操作中のサーボの情報。操作していないときはNone。
    last_read = time.monotonic()
    last_joint_read = last_read
    last_frame = last_read
    focused = bool(pygame.key.get_focused())
    gamepad_status = "マウス操作"
    clock = pygame.time.Clock()
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return
            if event.type == pygame.WINDOWFOCUSLOST:
                focused = False
                dragging = None
                for control in controls:
                    control["pending"] = False
            elif event.type == pygame.WINDOWFOCUSGAINED:
                focused = True
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if EXIT_BUTTON.collidepoint(event.pos):
                    return
                if GRIPPER_BUTTON.collidepoint(event.pos):
                    toggle_gripper(usb, gripper)
                elif GRIPPER_OFF_BUTTON.collidepoint(event.pos):
                    if gamepad is not None:
                        gamepad.disarm()
                    write_data(usb, GRIPPER_ID, 40, [0])
                    if read_data(usb, GRIPPER_ID, 40, 1)[0] != 0:
                        raise RuntimeError(f"ID {GRIPPER_ID}: トルク保持を解除できませんでした。")
                    gripper["enabled"] = False
                    gripper["closing"] = False
                    gripper["load_hits"] = 0
                    gripper["next_closed"] = True
                    gripper["status"] = "保持解除済み / 次の開閉操作では閉じます。"
                for control in controls:
                    if control["off_button"].collidepoint(event.pos):
                        if gamepad is not None:
                            gamepad.disarm()
                        if dragging is control:
                            dragging = None
                        control["pending"] = control["enabled"] = False
                        control["gamepad_active"] = False
                        servo_id = control["id"]
                        write_data(usb, servo_id, 40, [0])  # 0: このIDだけ保持を解除
                        if read_data(usb, servo_id, 40, 1)[0] != 0:
                            raise RuntimeError(f"ID {servo_id}: トルク保持を解除できませんでした。")
                        control["status"] = "保持解除済み / スライダー操作で再び有効"
                    elif control["slider"].collidepoint(event.pos) and not control["blocked"]:
                        dragging = control
                        control["target"] = slider_position(event.pos[0], control)
                        control["pending"] = True
            elif event.type == pygame.MOUSEMOTION and dragging is not None:
                dragging["target"] = slider_position(event.pos[0], dragging)
                dragging["pending"] = True
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if dragging is not None:
                    dragging["target"] = slider_position(event.pos[0], dragging)
                    dragging["pending"] = True
                dragging = None

        now = time.monotonic()
        if now - last_joint_read >= JOINT_POLL_INTERVAL:
            refresh_joint_positions(usb, controls)
            last_joint_read = now
        if gamepad is not None:
            inputs = gamepad.read(focused)
            gamepad_status = inputs["status"]
            apply_gamepad(usb, controls, gripper, inputs, now - last_frame)
        last_frame = now
        # 手先の負荷確認を先に行い、しきい値を超えたらその場で保持する。
        update_gripper(usb, gripper, now)
        # 各IDに最大20回/秒で最新の目標を送る。到着は待たない。
        for control in controls:
            if control["pending"] and now - control["last_send"] >= 0.05:
                send_arm_target(usb, controls, control, now)
        if now - last_read >= 0.25:
            for control in controls:
                if control["id"] not in (2, 3):
                    control["current"] = read_position(usb, control["id"])
            last_read = now
        draw_screen(screen, font, large_font, controls, gripper, gamepad_status)
        clock.tick(60)


def show_error(screen, font, error):
    """通信失敗時は追加の命令を送らず、画面に理由を表示する。"""
    print(f"エラー: {error}")
    screen.fill((35, 25, 30))
    lines = ["通信エラー（追加の命令は送信していません）", ""]
    # 長いエラーも画面内に収める。
    line = ""
    for character in str(error):
        if font.size(line + character)[0] > 710:
            lines.append(line)
            line = ""
        line += character
    lines += [line, "", "終了するにはウィンドウを閉じるか、Escを押してください。",
              "サーボの移動・保持が継続している場合があります。"]
    for index, line in enumerate(lines):
        screen.blit(font.render(line, True, (255, 205, 205)), (40, 40 + index * 32))
    pygame.display.flip()
    while True:
        event = pygame.event.wait()
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            return


def main():
    pygame.display.init()
    pygame.font.init()
    screen = pygame.display.set_mode((900, 940))
    pygame.display.set_caption("STS3215 Servo Control")
    font = pygame.font.SysFont("meiryo,yugothic,msgothic", 19)
    large_font = pygame.font.SysFont("meiryo,yugothic", 27)
    gamepad = Gamepad()
    try:
        screen.fill((22, 28, 40))
        screen.blit(font.render("サーボへ接続しています...", True, (230, 237, 247)), (40, 40))
        pygame.display.flip()
        with serial.Serial(PORT, BAUDRATE, timeout=0.5, write_timeout=0.5) as usb:
            run_slider(usb, screen, font, large_font, gamepad)
    except (serial.SerialException, RuntimeError) as error:
        show_error(screen, font, error)
    finally:
        gamepad.close()
        pygame.quit()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pygame.quit()
    finally:
        print("通信終了。終了操作ではサーボの移動やトルク保持は解除しません。")

