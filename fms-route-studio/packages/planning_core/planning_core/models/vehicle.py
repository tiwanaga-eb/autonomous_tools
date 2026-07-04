from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, model_validator

KinematicType = Literal["rigid_bicycle", "articulated", "tracked_skid"]


class VehicleProfile(BaseModel):
    """車両プロファイル（設計書 §8 / §8.1）。

    運動学は3タイプ: rigid_bicycle(HD785/HD605), articulated(HM400), tracked_skid(CD110R)。
    自転車前提のフィールド(wheel_base/max_steer_angle)は rigid_bicycle のみ必須。
    """

    id: str
    name: str
    kinematic_type: KinematicType

    # 諸元（共通）
    overall_length: float
    overall_width: float
    overall_height: float
    min_turning_radius: float | None = None
    kappa_max_fwd: float | None = None
    kappa_max_rev: float | None = None
    kappa_rate_max: float | None = None
    max_speed_fwd: float | None = None
    max_speed_rev: float | None = None
    footprint_polygon: list[tuple[float, float]] | None = None  # 基準点(姿勢x,y,yaw)基準 [m]。HM400は後輪軸中心
    footprint_radius: float | None = None
    road_width: float | None = None
    max_steer_rate: float | None = None

    # 安全検証(設計 Stage 5)のしきい値
    max_grade_pct: float | None = None    # 許容縦断勾配[%]（超過で安全違反）
    min_clearance_m: float | None = None  # 要求最小離隔[m]（経路に沿うクリアランス下限）

    # 速度プロファイル(設計 Stage 6) / 運動特性（§8.1。HM400 は実測表より）
    max_lateral_accel: float | None = None  # 運用上の許容横加速度[m/s^2]（荷こぼれしない。コーナー速度 v=sqrt(a/κ)）
    max_accel: float | None = None          # 最大加速度[m/s^2]（アクセル100%相当）
    max_decel: float | None = None          # 通常減速度[m/s^2]（快適減速・停止可能距離に使用）
    max_decel_loaded: float | None = None   # 積載時の通常減速度[m/s^2]（未設定=max_decel。fleet の予約距離等は保守側=積載値で計算）
    accel_start: float | None = None        # 発進時加速度[m/s^2]（0〜10km/h の緩発進）
    decel_emergency: float | None = None    # 急制動減速度[m/s^2]（タイヤロックしない範囲の最大減速）
    lateral_accel_limit: float | None = None  # 横加速度の構造限界[m/s^2]（転倒/構造限界。運用上限ではない）
    # 速度依存の最大操舵速度 [(速度km/h, 最大操舵速度rad/s), ...]（高速ほど操舵を制限＝ジャーク抑制）
    steer_rate_profile: list[tuple[float, float]] | None = None
    max_speed_downhill_empty: float | None = None   # 下り勾配での最大速度(空車)[m/s]
    max_speed_downhill_loaded: float | None = None  # 下り勾配での最大速度(積荷)[m/s]

    # rigid_bicycle 用
    wheel_base: float | None = None
    max_steer_angle: float | None = None

    # articulated 用（HM400）
    front_length: float | None = None
    rear_length: float | None = None
    max_articulation_angle: float | None = None

    # tracked_skid 用（CD110R）
    track_width: float | None = None
    can_turn_in_place: bool = False

    # 諸元の出所（measured=実値, estimated=暫定）
    spec_status: Literal["measured", "estimated"] = "measured"

    @model_validator(mode="after")
    def _validate_kinematics(self) -> "VehicleProfile":
        if self.kinematic_type == "rigid_bicycle" and self.wheel_base is None:
            raise ValueError(f"{self.id}: rigid_bicycle requires wheel_base")
        if self.kinematic_type == "tracked_skid" and self.wheel_base is not None:
            raise ValueError(f"{self.id}: tracked_skid must not define wheel_base (no bicycle model)")
        return self

    def supports_steering_angle(self) -> bool:
        """steer_deg = atan(wheel_base * kappa) が意味を持つのは rigid_bicycle のみ。"""
        return self.kinematic_type == "rigid_bicycle"
