"""安全検証（設計 Stage 5）。

ルール・幾何・物理制約で経路の安全性を判定し、合否(pass/fail)と**不可理由**(日本語)を返す。
AI生成のブラックボックスに最終安全判定を委ねず、ここで車両包絡線・最小旋回半径・操舵・
勾配・最小離隔を統合チェックする（設計の「制約ベースを主、AIを補助」を体現）。

`feasible`(運動学＋包含の成立) とは別に、`SafetyReport.passed` を「車両へ配信してよいか」の
総合判定とする。不可なら reasons を提示し、人手介入/目標姿勢変更/エリア再設計へ落とす運用を想定。
"""
from __future__ import annotations

import math

from ..models.analysis import AnalysisResult, SafetyReport, SafetyCheck
from ..models.route import Trajectory
from ..models.vehicle import VehicleProfile


def _fmt(v: float | None, unit: str) -> str:
    return "—" if v is None else f"{v:.2f}{unit}"


def verify_safety(
    traj: Trajectory,
    analysis: AnalysisResult,
    vehicle: VehicleProfile | None,
    *,
    clearance_m: float | None = None,
    advisory_kinds: tuple[str, ...] = (),
    footprint_evaluated: bool = True,
    approach_error_m: float | None = None,
    approach_error_deg: float | None = None,
    approach_pos_tol_m: float = 0.5,
    approach_yaw_tol_deg: float = 5.0,
) -> SafetyReport:
    """軌跡＋解析＋車両から安全検証レポートを作る。clearance_m は経路に沿う実測最小離隔[m]。

    advisory_kinds: 参考扱いにするチェック名（合否に影響させない）。例: 寄り付き等の低速
    マニューバでは操舵レート dκ/ds は速度が低く非拘束なので "kappa_rate" を参考扱いにする。

    approach_error_m / approach_error_deg: 寄り付き（spotting）の一発到達誤差。与えると
    上位要求 P-008「一発到達精度 水平±0.5m・方位±5°」の合否チェックを追加する
    （しきい値は approach_pos_tol_m / approach_yaw_tol_deg で変更可）。

    footprint_evaluated: 走行可能領域 mask を与えて車両包絡線/離隔を実評価したか。False の場合は
    包絡線チェックを「評価不可（applicable=False）」として扱い、安全側（fail-closed）に倒す。

    **fail-closed 方針**: 実評価できたチェックが 1 つも無い（車両未指定・走行可能領域なし等で
    包絡線も運動学も評価できない）場合は、安易に passed=True とせず passed=False を返す。
    安全判定はルール/幾何/物理で固定し、情報不足を「合格」と誤認させない（設計の根幹）。
    """
    vios = analysis.violations
    checks: list[SafetyCheck] = []

    def has(kind: str) -> bool:
        return any(v.kind == kind for v in vios)

    def worst(kind: str) -> float | None:
        ms = [v.measured for v in vios if v.kind == kind]
        return max(ms) if ms else None

    skid = vehicle is not None and vehicle.kinematic_type == "tracked_skid"

    # 車両包絡線（実車体が走行可能領域に収まるか）。走行可能領域 mask 未供給なら評価不可。
    checks.append(SafetyCheck(
        name="footprint", label="車両包絡線（走行可能領域内）",
        ok=not has("footprint"), applicable=footprint_evaluated,
        measured=worst("footprint"), limit=0.0,
        detail=("走行可能領域 未供給で評価不可" if not footprint_evaluated
                else ("車体矩形がはみ出す区間あり" if has("footprint") else "車体は領域内")),
    ))

    # 最小旋回半径
    if vehicle is not None and not skid and vehicle.min_turning_radius:
        checks.append(SafetyCheck(
            name="min_radius", label="最小旋回半径 R_min",
            ok=not has("min_radius"), measured=analysis.min_radius_m, limit=vehicle.min_turning_radius,
        ))
    else:
        checks.append(SafetyCheck(
            name="min_radius", label="最小旋回半径 R_min", ok=True, applicable=False,
            detail="スキッド/半径制約なし（評価対象外）",
        ))

    # 操舵レート（曲率変化率）
    if vehicle is not None and vehicle.kappa_rate_max:
        checks.append(SafetyCheck(
            name="kappa_rate", label="操舵レート dκ/ds",
            ok=not has("kappa_rate"), measured=analysis.max_curvature_rate, limit=vehicle.kappa_rate_max,
        ))

    # 操舵角（rigid_bicycle のみ）
    if vehicle is not None and vehicle.supports_steering_angle() and vehicle.max_steer_angle:
        checks.append(SafetyCheck(
            name="steer", label="操舵角",
            ok=not has("steer_rate"), measured=analysis.max_steer_rate_required,
            limit=math.degrees(vehicle.max_steer_angle),
        ))

    # 縦断勾配
    if vehicle is not None and vehicle.max_grade_pct:
        mg = analysis.max_grade_pct
        ok = mg is None or abs(mg) <= vehicle.max_grade_pct
        checks.append(SafetyCheck(
            name="grade", label="縦断勾配",
            ok=ok, applicable=mg is not None, measured=mg, limit=vehicle.max_grade_pct,
            detail="DSM未供給で評価不可" if mg is None else "",
        ))

    # 最小離隔
    req_clear = vehicle.min_clearance_m if vehicle is not None else None
    if clearance_m is not None and req_clear:
        checks.append(SafetyCheck(
            name="clearance", label="最小離隔",
            ok=clearance_m >= req_clear, measured=clearance_m, limit=req_clear,
        ))

    # 寄り付き一発到達精度（P-008: 水平±0.5m・方位±5°）。誤差が与えられた場合のみ評価。
    if approach_error_m is not None:
        checks.append(SafetyCheck(
            name="approach_pos", label="寄り付き到達精度（位置）",
            ok=approach_error_m <= approach_pos_tol_m,
            measured=approach_error_m, limit=approach_pos_tol_m,
        ))
    if approach_error_deg is not None:
        checks.append(SafetyCheck(
            name="approach_yaw", label="寄り付き到達精度（方位）",
            ok=approach_error_deg <= approach_yaw_tol_deg,
            measured=approach_error_deg, limit=approach_yaw_tol_deg,
        ))

    # 参考扱い: 指定チェックは合否に影響させない（applicable=False ＝ 表示は残すが NG にしない）。
    for c in checks:
        if c.name in advisory_kinds:
            c.applicable = False
            note = "参考（低速マニューバでは非拘束）"
            c.detail = f"{c.detail} ／ {note}".lstrip(" ／") if c.detail else note

    reasons: list[str] = []
    units = {"footprint": "", "min_radius": "m", "kappa_rate": "", "steer": "°", "grade": "%", "clearance": "m",
             "approach_pos": "m", "approach_yaw": "°"}
    for c in checks:
        if c.applicable and not c.ok:
            reasons.append(f"{c.label}: 不適合（実測 {_fmt(c.measured, units.get(c.name, ''))} / 限界 {_fmt(c.limit, units.get(c.name, ''))}）")

    # fail-closed: 実評価できた（applicable な）チェックが皆無なら「合格」にしない。
    applicable_checks = [c for c in checks if c.applicable]
    if not applicable_checks:
        reasons.append(
            "安全検証に必要な情報が不足（車両・走行可能領域が未指定のため配信可否を判定できません）"
        )
        return SafetyReport(passed=False, checks=checks, reasons=reasons)

    passed = all(c.ok for c in applicable_checks)
    return SafetyReport(passed=passed, checks=checks, reasons=reasons)
