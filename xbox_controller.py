"""Xbox形式の入力を読み取る。サーボへの命令はこのファイルでは送らない。"""

import pygame
from pygame._sdl2 import controller

DEADZONE = 0.18  # 中央付近の小さな揺れを無視する
# サーボID: (スティックの軸, 向き)。向きは1と-1で反転できる。
AXES = {
    1: (pygame.CONTROLLER_AXIS_LEFTX, 1),
    2: (pygame.CONTROLLER_AXIS_LEFTY, -1),
    3: (pygame.CONTROLLER_AXIS_RIGHTY, -1),
}


def stick_value(raw):
    value = max(-1, min(1, raw / 32767))
    if abs(value) <= DEADZONE:
        return 0.0
    magnitude = (abs(value) - DEADZONE) / (1 - DEADZONE)
    return magnitude if value > 0 else -magnitude


class Gamepad:
    def __init__(self):
        controller.init()
        self.device = None
        self.ready = False
        self.previous_a = self.previous_b = False

    def read(self, focused=True):
        was_ready = self.ready
        result = {"axes": {sid: 0.0 for sid in AXES}, "toggle": False,
                  "stop": False, "interrupted": False, "status": "コントローラー未接続 / USB接続してください"}
        try:
            if self.device is not None and not self.device.attached():
                self.device.quit()
                self.device = None
                self.ready = False
                # 接続が切れたフレームでは別の機器に切り替えず、先に停止する。
                result["interrupted"] = was_ready
                return result
            if self.device is None:
                self.ready = False
                for index in range(controller.get_count()):
                    if controller.is_controller(index):
                        self.device = controller.Controller(index)
                        break
            if self.device is None:
                return result
            name = self.device.name
            axes = {sid: stick_value(self.device.get_axis(axis)) * sign
                    for sid, (axis, sign) in AXES.items()}
            a = bool(self.device.get_button(pygame.CONTROLLER_BUTTON_A))
            b = bool(self.device.get_button(pygame.CONTROLLER_BUTTON_B))
            if not focused:
                self.ready = False
                result["status"] = "画面が非アクティブ / 操作停止・保持"
            else:
                # 起動・再接続後、スティックとボタンを離してから受け付ける。
                if not self.ready and not any(axes.values()) and not a and not b:
                    self.ready = True
                result["status"] = f"接続: {name}" if self.ready else "スティックを中央に戻し、A・Bを離してください"
                if self.ready:
                    result["axes"] = axes
                    result["stop"] = b and not self.previous_b
                    result["toggle"] = a and not self.previous_a and not b
                    if result["stop"]:
                        self.ready = False
                        result["axes"] = {sid: 0.0 for sid in AXES}
                        result["status"] = "Bで停止 / スティックを中央に戻して操作を再開"
            self.previous_a, self.previous_b = a, b
        except pygame.error:
            if self.device is not None:
                self.device.quit()
            self.device = None
            self.ready = False
            result["status"] = "コントローラーを読み取れません / 再接続してください"
        result["interrupted"] = was_ready and not self.ready
        return result

    def disarm(self):
        self.ready = False

    def close(self):
        if self.device is not None:
            self.device.quit()
        controller.quit()
