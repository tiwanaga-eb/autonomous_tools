from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

ViolationKind = Literal[
    "min_radius", "kappa_rate", "steer_rate", "road_width", "grade", "drivable_out", "footprint"
]


class Violation(BaseModel):
    kind: ViolationKind
    s_start: float
    s_end: float
    measured: float
    limit: float


class SafetyCheck(BaseModel):
    """安全検証(設計 Stage 5)の個別チェック結果。"""
    name: str                      # footprint / clearance / min_radius / kappa_rate / steer / grade
    label: str                     # 表示名（日本語）
    ok: bool
    applicable: bool = True        # 機種で評価対象か（例: skid は min_radius 非適用）
    measured: float | None = None
    limit: float | None = None
    detail: str = ""


class SafetyReport(BaseModel):
    """経路の安全検証サマリ。passed=全適用チェック合格。reasons=不可理由(日本語)。"""
    passed: bool
    checks: list[SafetyCheck] = Field(default_factory=list)
    reasons: list[str] = Field(default_factory=list)


class AnalysisResult(BaseModel):
    min_radius_m: float | None = None
    max_curvature: float = 0.0
    max_curvature_rate: float = 0.0           # dkappa/ds max（numeric源では参考値）
    max_steer_rate_required: float | None = None   # rigid_bicycle only
    max_grade_pct: float | None = None
    feasible: bool = True
    not_applicable: list[str] = Field(default_factory=list)  # 機種で評価対象外の指標
    violations: list[Violation] = Field(default_factory=list)
    # 速度プロファイル(Stage 6)
    max_speed_mps: float | None = None
    time_total_s: float | None = None
    stopping_distance_m: float | None = None
