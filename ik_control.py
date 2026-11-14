"""XYZ座標で操作するアプリ。実行: python ik_control.py

test.pyの通信処理・角度制限・ハンドの負荷検知を共用する。
制御点はハンド側リンクの回転軸中心で、指先の位置ではない。
原点は台座旋回軸上の肩軸高さ。X前方、Z上方、YはID1減少方向。
"""

import math
import time

import pygame
import serial

import test as hardware
from inverse_kinematics import ArmGeometry, ServoLimits, forward_kinematics, inverse_kinematics
from xbox_controller import Gamepad

# ---------- 寸法・基準姿勢 ----------
GEOMETRY = ArmGeometry(
    upper_length_mm=120,
    forearm_length_mm=120,
    upper_at_180_deg=45,
    forearm_at_180_deg=-45,
    upper_direction=1,
    forearm_direction=-1,
    yaw_direction=-1,  # ID1の回転方向を反転
)
MOVE_SPEED_MM_S = 30
POLL_INTERVAL = 0.05
MAX_GAMEPAD_LEAD_MM = 3
ARRIVAL_TOLERANCE_MM = 0.5
MOVE_TIMEOUT = 30
XYZ_RANGES = ((0, 240), (-240, 240), (-240, 240))
XYZ_SLIDERS = [pygame.Rect(170, 176 + index * 190, 500, 32) for index in range(3)]
STOP_BUTTON = pygame.Rect(650, 12, 200, 36)


def positions_of(controls):
    return tuple(next(c["current"] for c in controls if c["id"] == sid) for sid in (1, 2, 3))


def make_state(controls):
    current = forward_kinematics(positions_of(controls), GEOMETRY)
    limits = ServoLimits(
        tuple((c["base_minimum"] * 360 / 4096, c["base_maximum"] * 360 / 4096)
              for c in sorted(controls, key=lambda c: c["id"])),
        (hardware.ID2_ID3_MIN_ANGLE_SUM, hardware.ID2_ID3_MAX_ANGLE_SUM),
    )
    return {"current": current, "goal": current, "limits": limits, "moving": False,
            "gamepad_active": False, "last_update": 0, "started": 0,
            "status": "現在姿勢を読み取りました。起動時には動きません。"}


def refresh_positions(usb, controls, state):
    for control in controls:
        position = hardware.read_position(usb, control["id"])
        if not 0 <= position <= 4095:
            raise RuntimeError(f"ID {control['id']}: 現在位置が単回転の範囲外です。")
        control["current"] = position
    hardware.update_joint_limits(controls)
    state["current"] = forward_kinematics(positions_of(controls), GEOMETRY)


def request_target(xyz, controls, state, now):
    """届く目標だけを受け付ける。ここではUSBに命令を送らない。"""
    positions = positions_of(controls)
    if not state["limits"].allows(positions):
        state["status"] = "現在姿勢が角度範囲外です。test.pyで姿勢を調整してください。"
        return False
    solutions = inverse_kinematics(tuple(xyz), GEOMETRY, state["limits"], positions)
    if not solutions:
        state["status"] = "到達不可: リンク長・絶対角度・相対角度の制限を超えます。"
        return False
    state.update(goal=tuple(xyz), moving=True, started=now, status="目標座標へ移動します。")
    return True


def stop_motion(usb, controls, gripper, state, reason="停止・保持", stop_hand=False):
    """トルクを切らず、現在位置で保持する。"""
    for control in controls:
        if control["enabled"]:
            hardware.hold_arm(usb, control)
        control["pending"] = control["gamepad_active"] = False
    if stop_hand and gripper["enabled"]:
        gripper["current"] = hardware.read_position(usb, hardware.GRIPPER_ID)
        hardware.hold_gripper(usb, gripper, reason)
    refresh_positions(usb, controls, state)
    state.update(goal=state["current"], moving=False, gamepad_active=False, status=reason)


def update_ik(usb, controls, gripper, state, now):
    """短い距離ずつIKを解き、既存の送信直前チェックを通して動かす。"""
    elapsed = now - state["last_update"]
    if elapsed < POLL_INTERVAL:
        return
    state["last_update"] = now
    refresh_positions(usb, controls, state)
    if not state["moving"]:
        return
    if not state["limits"].allows(positions_of(controls)):
        stop_motion(usb, controls, gripper, state, "現在姿勢が角度範囲外のため停止")
        return
    distance = math.dist(state["current"], state["goal"])
    if distance <= ARRIVAL_TOLERANCE_MM:
        if state["gamepad_active"]:
            return  # 微小入力を蓄積し、ゆっくり倒した場合も動かせるようにする。
        # 最後の命令が残っていても、この時点の位置で止める。
        stop_motion(usb, controls, gripper, state, "目標座標に到着・保持")
        return
    if now - state["started"] >= MOVE_TIMEOUT:
        stop_motion(usb, controls, gripper, state, "時間切れ: 負荷・経路を確認してください")
        return
    step = min(distance, MOVE_SPEED_MM_S * min(0.05, elapsed))
    next_xyz = tuple(current + (goal - current) * step / distance
                     for current, goal in zip(state["current"], state["goal"]))
    solutions = inverse_kinematics(next_xyz, GEOMETRY, state["limits"], positions_of(controls))
    if not solutions:
        stop_motion(usb, controls, gripper, state, "途中の座標が到達範囲外のため停止")
        return
    desired = dict(zip((1, 2, 3), solutions[0].positions))
    # 相対制限の端では、角度の和を範囲の中央へ戻す方向の軸から送る。
    total = sum(c["current"] for c in controls if c["id"] in (2, 3)) * 360 / 4096
    midpoint = sum(state["limits"].angle_sum_deg) / 2
    joints = sorted((c for c in controls if c["id"] in (2, 3)),
                    key=lambda c: (desired[c["id"]] - c["current"]) * (1 if total >= midpoint else -1))
    ordered = [next(c for c in controls if c["id"] == 1)] + joints
    for control in ordered:
        target = desired[control["id"]]
        if target == control["sent_target"] and control["enabled"]:
            continue
        control.update(target=target, pending=True)
        hardware.send_arm_target(usb, controls, control, now)
    state["status"] = f"移動中 / 残り {distance:.1f} mm"


def apply_gamepad(usb, controls, gripper, state, inputs, elapsed, now):
    if inputs["stop"] or inputs["interrupted"]:
        stop_motion(usb, controls, gripper, state, stop_hand=True)
        return
    if inputs["toggle"]:
        hardware.toggle_gripper(usb, gripper)
    # Xbox入力: 左スティック上下=X、左右=Y、右スティック上下=Z。
    axes = inputs["axes"]
    velocity = (axes.get(2, 0), axes.get(1, 0), axes.get(3, 0))
    magnitude = math.sqrt(sum(value * value for value in velocity))
    if magnitude == 0:
        if state["gamepad_active"]:
            stop_motion(usb, controls, gripper, state)
        return
    if not state["gamepad_active"]:
        stop_motion(usb, controls, gripper, state)
    delta = MOVE_SPEED_MM_S * max(0, min(0.05, elapsed)) / max(1, magnitude)
    goal = tuple(p + value * delta for p, value in zip(state["goal"], velocity))
    lead = math.dist(goal, state["current"])
    if lead > MAX_GAMEPAD_LEAD_MM:
        goal = tuple(p + (g - p) * MAX_GAMEPAD_LEAD_MM / lead
                     for p, g in zip(state["current"], goal))
    was_moving = state["moving"]
    if not request_target(goal, controls, state, now) and was_moving:
        stop_motion(usb, controls, gripper, state, state["status"])
    state["gamepad_active"] = True


def slider_xyz(mouse_x, index, state):
    slider = XYZ_SLIDERS[index]
    lower, upper = XYZ_RANGES[index]
    ratio = max(0, min(1, (mouse_x - slider.left) / slider.width))
    xyz = list(state["goal"])
    xyz[index] = lower + (upper - lower) * ratio
    return xyz


def draw_screen(screen, font, large_font, state, gripper, gamepad_status):
    screen.fill((22, 28, 40))

    def text(message, x, y, large=False, color=(230, 237, 247)):
        screen.blit((large_font if large else font).render(message, True, color), (x, y))

    text("STS3215 / XYZ・逆運動学", 40, 12, large=True)
    text(f"{hardware.PORT} / {hardware.BAUDRATE} bps / 制御点: ハンド側リンク軸中心", 40, 50)
    text(gamepad_status, 40, 79, color=(112, 225, 172))
    pygame.draw.rect(screen, (126, 63, 55), STOP_BUTTON, border_radius=8)
    text("停止・保持 [B]", 674, 16)
    for index, name in enumerate(("X", "Y", "Z")):
        slider = XYZ_SLIDERS[index]
        row_y = slider.top - 66
        lower, upper = XYZ_RANGES[index]
        goal, current = state["goal"][index], state["current"][index]
        pygame.draw.rect(screen, (30, 38, 52), (40, row_y, 820, 172), border_radius=12)
        text(name, 65, row_y + 22, large=True)
        text(f"目標 {goal:.1f} mm", 170, row_y + 12, large=True)
        text(f"現在 {current:.1f} mm", 455, row_y + 20)
        x = slider.left + round(max(0, min(1, (goal - lower) / (upper - lower))) * slider.width)
        actual_x = slider.left + round(max(0, min(1, (current - lower) / (upper - lower))) * slider.width)
        y = slider.centery
        pygame.draw.line(screen, (66, 78, 99), (slider.left, y), (slider.right, y), 8)
        pygame.draw.line(screen, (70, 171, 234), (slider.left, y), (x, y), 8)
        pygame.draw.circle(screen, (244, 247, 255), (x, y), 13)
        pygame.draw.line(screen, (112, 225, 172), (actual_x, y + 20), (actual_x, y + 28), 3)
        text(f"{lower} mm", slider.left, row_y + 112)
        text(f"{upper} mm", slider.right - 85, row_y + 112)
        labels = ("前後", "旋回正方向", "上下")
        text(labels[index], 710, row_y + 67)
        text(state["status"] if index == 2 else ("肩軸高さをZ=0とします。" if index == 0
             else "届かない座標は受け付けません。"), 170, row_y + 142, color=(112, 225, 172))
    pygame.draw.rect(screen, (30, 38, 52), (40, 682, 820, 146), border_radius=12)
    text("ID 4", 60, 703, large=True)
    text("ハンド開閉 [A]", 170, 696, large=True)
    pygame.draw.rect(screen, (40, 121, 158), hardware.GRIPPER_BUTTON, border_radius=8)
    text("閉じる" if gripper["next_closed"] else "開く", 480, 720)
    pygame.draw.rect(screen, (126, 63, 55), hardware.GRIPPER_OFF_BUTTON, border_radius=8)
    text("保持解除", 738, 720)
    text(f"負荷 {gripper['load'] / 10:.1f}% / 検知しきい値 {hardware.GRIPPER_LOAD_THRESHOLD / 10:.1f}%", 430, 769)
    text(gripper["status"], 170, 800, color=(112, 225, 172))
    pygame.draw.rect(screen, (53, 65, 86), hardware.EXIT_BUTTON, border_radius=8)
    text("終了 / Esc", 702, 852)
    text("左スティック: X前後・Y左右 / 右スティック上下: Z上下", 40, 840)
    text("A: ハンド開閉 / B: 停止・保持 / スティック中央で停止", 40, 882)
    text("座標は計算値です。指先オフセットは含みません。", 40, 908, color=(154, 170, 194))
    pygame.display.flip()


def run(usb, screen, font, large_font, gamepad):
    controls = hardware.create_controls(usb)
    gripper = hardware.create_gripper(usb)
    state = make_state(controls)
    dragging = None
    focused = bool(pygame.key.get_focused())
    last_frame = time.monotonic()
    clock = pygame.time.Clock()
    while True:
        now = time.monotonic()
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                stop_motion(usb, controls, gripper, state, stop_hand=True)
                return
            if event.type == pygame.WINDOWFOCUSLOST:
                focused = False
                dragging = None
                stop_motion(usb, controls, gripper, state, stop_hand=True)
            elif event.type == pygame.WINDOWFOCUSGAINED:
                focused = True
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if hardware.EXIT_BUTTON.collidepoint(event.pos):
                    stop_motion(usb, controls, gripper, state, stop_hand=True)
                    return
                if STOP_BUTTON.collidepoint(event.pos):
                    gamepad.disarm()
                    dragging = None
                    stop_motion(usb, controls, gripper, state, stop_hand=True)
                elif hardware.GRIPPER_BUTTON.collidepoint(event.pos):
                    hardware.toggle_gripper(usb, gripper)
                elif hardware.GRIPPER_OFF_BUTTON.collidepoint(event.pos):
                    hardware.write_data(usb, hardware.GRIPPER_ID, 40, [0])
                    if hardware.read_data(usb, hardware.GRIPPER_ID, 40, 1)[0] != 0:
                        raise RuntimeError("ID4の保持を解除できませんでした。")
                    gripper.update(enabled=False, closing=False, load_hits=0, next_closed=True,
                                   status="保持解除済み / 次は閉じます。")
                else:
                    for index, slider in enumerate(XYZ_SLIDERS):
                        if slider.collidepoint(event.pos):
                            dragging = index
            if dragging is not None and event.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEMOTION, pygame.MOUSEBUTTONUP):
                if not request_target(slider_xyz(event.pos[0], dragging, state), controls, state, now) and state["moving"]:
                    stop_motion(usb, controls, gripper, state, state["status"])
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                dragging = None
        inputs = gamepad.read(focused)
        apply_gamepad(usb, controls, gripper, state, inputs, now - last_frame, now)
        last_frame = now
        hardware.update_gripper(usb, gripper, now)
        update_ik(usb, controls, gripper, state, now)
        draw_screen(screen, font, large_font, state, gripper, inputs["status"])
        clock.tick(60)


def main():
    pygame.display.init()
    pygame.font.init()
    screen = pygame.display.set_mode((900, 940))
    pygame.display.set_caption("STS3215 Cartesian IK Control")
    font = pygame.font.SysFont("meiryo,yugothic,msgothic", 19)
    large_font = pygame.font.SysFont("meiryo,yugothic", 27)
    gamepad = Gamepad()
    try:
        screen.fill((22, 28, 40))
        screen.blit(font.render("サーボへ接続しています...", True, (230, 237, 247)), (40, 40))
        pygame.display.flip()
        with serial.Serial(hardware.PORT, hardware.BAUDRATE, timeout=0.5, write_timeout=0.5) as usb:
            run(usb, screen, font, large_font, gamepad)
    except (serial.SerialException, RuntimeError, ValueError) as error:
        hardware.show_error(screen, font, error)
    finally:
        gamepad.close()
        pygame.quit()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pygame.quit()
