"""台座旋回＋平行リンクアームの順運動学・逆運動学。USB操作は行わない。

ID2は上腕の絶対角、ID3はリンクを介した前腕の絶対角に対応するモデル。
制御点はハンド側リンク軸の中心。把持点までのオフセットは含まない。
原点は台座旋回軸上の肩軸高さ。+XはID1=180度の前方、+Zは上。
+Yはyaw_direction=1ならID1の値が増える側。
"""

from dataclasses import dataclass
from itertools import product
import math

STEP_DEGREES = 360 / 4096


@dataclass(frozen=True)
class ArmGeometry:
    upper_length_mm: float
    forearm_length_mm: float
    upper_at_180_deg: float
    forearm_at_180_deg: float
    upper_direction: int
    forearm_direction: int
    yaw_direction: int = 1

    def __post_init__(self):
        values = (self.upper_length_mm, self.forearm_length_mm,
                  self.upper_at_180_deg, self.forearm_at_180_deg)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("リンク寸法と基準角度は有限の数値で指定してください。")
        if self.upper_length_mm <= 0 or self.forearm_length_mm <= 0:
            raise ValueError("リンク長は0より大きい値が必要です。")
        if any(value not in (-1, 1) for value in
               (self.upper_direction, self.forearm_direction, self.yaw_direction)):
            raise ValueError("回転方向は1または-1で指定してください。")


@dataclass(frozen=True)
class ServoLimits:
    angles_deg: tuple  # ID1、ID2、ID3の(下限, 上限)
    angle_sum_deg: tuple  # ID2+ID3の(下限, 上限)

    def __post_init__(self):
        if len(self.angles_deg) != 3 or len(self.angle_sum_deg) != 2:
            raise ValueError("ID1～3の絶対範囲とID2+ID3の範囲が必要です。")
        for lower, upper in (*self.angles_deg, self.angle_sum_deg):
            if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
                raise ValueError("角度制限の下限・上限を確認してください。")

    def allows(self, positions):
        if len(positions) != 3:
            return False
        angles = [position * STEP_DEGREES for position in positions]
        return (all(0 <= position <= 4095 for position in positions)
                and all(lower <= angle <= upper
                        for angle, (lower, upper) in zip(angles, self.angles_deg))
                and self.angle_sum_deg[0] <= angles[1] + angles[2] <= self.angle_sum_deg[1])


@dataclass(frozen=True)
class IKSolution:
    positions: tuple
    xyz_mm: tuple
    error_mm: float


def forward_kinematics(positions, geometry):
    """ID1～3の位置値からハンド側リンク軸のXYZ(mm)を計算する。"""
    if len(positions) != 3 or not all(math.isfinite(p) and 0 <= p <= 4095 for p in positions):
        raise ValueError("ID1～3の位置値を0～4095で指定してください。")
    s1, s2, s3 = [position * STEP_DEGREES for position in positions]
    yaw = math.radians((s1 - 180) * geometry.yaw_direction)
    upper = math.radians(geometry.upper_at_180_deg + (s2 - 180) * geometry.upper_direction)
    forearm = math.radians(geometry.forearm_at_180_deg + (s3 - 180) * geometry.forearm_direction)
    radius = geometry.upper_length_mm * math.cos(upper) + geometry.forearm_length_mm * math.cos(forearm)
    height = geometry.upper_length_mm * math.sin(upper) + geometry.forearm_length_mm * math.sin(forearm)
    return radius * math.cos(yaw), radius * math.sin(yaw), height


def inverse_kinematics(xyz_mm, geometry, limits, current_positions=None):
    """角度制限内の解を返す。到達不可なら空リスト。現在姿勢に近い解を優先する。"""
    if len(xyz_mm) != 3 or not all(math.isfinite(v) for v in xyz_mm):
        raise ValueError("XYZは有限の数値3つで指定してください。")
    if current_positions is not None:
        forward_kinematics(current_positions, geometry)  # 入力を検証
    x, y, z = xyz_mm
    radius = math.hypot(x, y)
    l1, l2 = geometry.upper_length_mm, geometry.forearm_length_mm
    distance = math.hypot(radius, z)
    if distance > l1 + l2 + 1e-8 or distance < abs(l1 - l2) - 1e-8:
        return []
    if distance < 1e-8:
        return []  # 完全に折り畳んだ特異姿勢は選ばない
    cosine = (distance * distance - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    bend = math.acos(max(-1, min(1, cosine)))
    if radius < 1e-8:
        s1 = 180 if current_positions is None else current_positions[0] * STEP_DEGREES
        base_yaw = math.radians((s1 - 180) * geometry.yaw_direction)
    else:
        base_yaw = math.atan2(y, x)
    solutions = {}
    # 肘の2解と、台座の前後の解を調べる。角度制限に合わないものは捨てる。
    for radial_sign, elbow_sign in product((1, -1), (1, -1)):
        r = radius * radial_sign
        yaw = base_yaw + (math.pi if radial_sign < 0 else 0)
        relative_elbow = bend * elbow_sign
        upper = math.atan2(z, r) - math.atan2(l2 * math.sin(relative_elbow), l1 + l2 * math.cos(relative_elbow))
        forearm = upper + relative_elbow
        angles = (
            (180 + math.degrees(yaw) / geometry.yaw_direction) % 360,
            (180 + (math.degrees(upper) - geometry.upper_at_180_deg) / geometry.upper_direction) % 360,
            (180 + (math.degrees(forearm) - geometry.forearm_at_180_deg) / geometry.forearm_direction) % 360,
        )
        # 1ステップに丸めた後も、絶対制限・角度の和を守る候補だけにする。
        neighbors = [sorted({math.floor(angle / STEP_DEGREES), math.ceil(angle / STEP_DEGREES)})
                     for angle in angles]
        branch = []
        for positions in product(*neighbors):
            if not limits.allows(positions):
                continue
            actual = forward_kinematics(positions, geometry)
            error = math.dist(actual, xyz_mm)
            branch.append(IKSolution(positions, actual, error))
        if branch:
            best = min(branch, key=lambda solution: solution.error_mm)
            solutions[best.positions] = best
    if current_positions is None:
        return sorted(solutions.values(), key=lambda solution: solution.error_mm)
    return sorted(solutions.values(), key=lambda solution:
                  (sum((p - current) ** 2 for p, current in zip(solution.positions, current_positions)),
                   solution.error_mm))
