"""AI アシスタント API（設計書 §14）。

会話型コパイロット。Claude の tool-use ループをサーバ側で回し、既存の
planning / drivable / costmap / spotting 操作をツールとして公開する。
UI と同じ Command Bus 相当の「操作面」を共有するため、状態を変える結果は
`actions` として返し、FE がストアへ適用する（FE はそれを描画する）。

鍵は環境変数 ANTHROPIC_API_KEY（または ANTHROPIC_AUTH_TOKEN）から読む。
鍵が無い場合は 503 を返し、FE が「鍵を設定してください」を表示する。
"""
from __future__ import annotations

import json
import os
import re
from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from planning_core.vehicle import BUILTIN_IDS

from .. import store
from . import costmap as costmap_router
from . import drivable as drivable_router
from . import planning as planning_router
from . import simulate as simulate_router
from . import vehicles as vehicles_router

router = APIRouter(prefix="/api/agent", tags=["agent"])

MODEL = os.environ.get("FRS_AI_MODEL", "claude-opus-4-8")
MAX_TOOL_ITERS = 8
# 生成系（ディスク書込を伴う）ツールの1リクエスト内実行上限。暴走/プロンプト注入での連続実行を抑止。
MAX_GENERATE_CALLS = 3


# ---------------------------------------------------------------------------
# リクエスト/レスポンス
# ---------------------------------------------------------------------------
class ChatMessage(BaseModel):
    role: str          # "user" | "assistant"
    content: str


class ChatRequest(BaseModel):
    messages: list[ChatMessage]
    context: dict[str, Any] = {}   # FE スナップショット（vehicle_id / plan / waypoints / spot）


# ---------------------------------------------------------------------------
# 状態（FE コンテキストから初期化し、書き込みツールで更新する作業コピー）
# ---------------------------------------------------------------------------
def _init_state(ctx: dict) -> dict:
    plan = ctx.get("plan") or {}
    spot = ctx.get("spot") or {}
    return {
        "vehicle_id": ctx.get("vehicle_id"),
        "plan": {
            "mode": plan.get("mode", "waypoint_guided"),
            "algorithm": plan.get("algorithm", "spline"),
            "spacing_m": plan.get("spacing_m", 2.0),
            "road_width_m": plan.get("road_width_m", 0.0),
            "min_turn_radius_m": plan.get("min_turn_radius_m"),
            "enforce_footprint": plan.get("enforce_footprint", True),
            "enforce_min_radius": plan.get("enforce_min_radius", True),
            "allow_reverse": plan.get("allow_reverse", False),
            "refine_elastic_band": plan.get("refine_elastic_band", False),
        },
        "waypoints": list(ctx.get("waypoints") or []),
        "areas": list(ctx.get("areas") or []),  # 進入禁止(NoGoZone)多角形 [[ [x,y], ... ], ...]
        "spot": {
            "start": spot.get("start"),
            "target": spot.get("target"),
            "max_switchbacks": spot.get("max_switchbacks", 1),
            "require_switchback": spot.get("require_switchback", False),
            "road_width_m": spot.get("road_width_m", 0.0),
            "method": spot.get("method", "auto"),
            "smooth_path": spot.get("smooth_path", True),
            "weights": spot.get("weights")
            or {"w_distance": 1.0, "w_time": 0.0, "w_reverse": 1.0, "w_switchback": 8.0, "w_costmap": 2.0},
        },
    }


def _latest(kind: str) -> dict | None:
    """FE と同じ解決: 同種レイヤの最新（最後に登録されたもの）。"""
    matches = [m for m in store.list_layers() if m.get("kind") == kind]
    return matches[-1] if matches else None


def _layers_summary() -> list[dict]:
    out = []
    for m in store.list_layers():
        out.append(
            {
                "id": m["id"],
                "kind": m.get("kind"),
                "version": m.get("version"),
                "filename": m.get("filename"),
                "epsg": m.get("epsg"),
                "stats": m.get("stats"),
            }
        )
    return out


def _state_snapshot(state: dict) -> dict:
    cost = _latest("cost")
    drivable = _latest("drivable")
    las = _latest("las")
    return {
        "layers": _layers_summary(),
        "resolved_layers": {
            "cost_layer_id": cost["id"] if cost else None,
            "drivable_layer_id": drivable["id"] if drivable else None,
            "las_layer_id": las["id"] if las else None,
        },
        "vehicle_id": state["vehicle_id"],
        "plan_options": state["plan"],
        "waypoints": state["waypoints"],
        "spotting": state["spot"],
        "crs": "EPSG:6677 (meters)",
    }


# ---------------------------------------------------------------------------
# ツール定義（Claude へ渡す JSON スキーマ）
# ---------------------------------------------------------------------------
TOOLS: list[dict] = [
    {
        "name": "get_app_state",
        "description": (
            "現在のアプリ状態を取得する。利用可能なレイヤ（cost/drivable/las/ortho）、選択中の車両、"
            "経路の生成オプション、配置済みウェイポイント、寄り付き(spotting)の姿勢/重みを返す。"
            "操作の前に状態を確認したいとき、または『今どうなってる？』と聞かれたら必ず呼ぶ。"
        ),
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "list_vehicles",
        "description": "選択可能な車両プロファイル（HD785/HD605/HM400/CD110R）の諸元（最小旋回半径・全幅・全長など）を返す。車両の比較や選定を相談されたら呼ぶ。",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "set_vehicle",
        "description": "経路生成・寄り付きで使う車両を選択する。ユーザーが車種を指定/変更したら呼ぶ。",
        "input_schema": {
            "type": "object",
            "properties": {"vehicle_id": {"type": "string", "enum": list(BUILTIN_IDS)}},
            "required": ["vehicle_id"],
        },
    },
    {
        "name": "set_plan_options",
        "description": (
            "経路生成オプションを更新する。mode=auto は A*（hybrid_astar/grid_astar）で自動探索、"
            "waypoint_guided は配置点に沿う。road_width_m>0 で A* が道幅を確保。指定した項目だけ更新される。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "mode": {"type": "string", "enum": ["auto", "waypoint_guided"]},
                "algorithm": {"type": "string", "enum": ["spline", "dubins", "hybrid_astar", "grid_astar"]},
                "spacing_m": {"type": "number", "description": "経路リサンプル間隔[m]"},
                "road_width_m": {"type": "number", "description": "道幅[m]（0=確保しない）"},
                "min_turn_radius_m": {"type": "number", "description": "最小旋回半径[m]（車両既定を上書き）"},
                "enforce_footprint": {"type": "boolean", "description": "hybrid A* で実車体フットプリント(向き付き矩形)を厳密な走行可能制約にする(既定true)。狭い領域で経路が出ないときは false に。"},
                "enforce_min_radius": {"type": "boolean", "description": "R_min を持つ車種(HD785/HD605/HM400)で最小旋回半径を保証する(既定true)。違反コーナーだけ局所平滑化して開く。CD110R(スキッド)は無関係。"},
                "allow_reverse": {"type": "boolean", "description": "hybrid A* で後進(切り返し)を許可する(既定false)。狭い/行き止まりのある領域で到達性が上がる。"},
                "refine_elastic_band": {"type": "boolean", "description": "経路生成後に Elastic Band で洗練（コリドー中央へ寄せ車体の余裕を確保）。走行可能領域(drivable)が必要。"},
            },
        },
    },
    {
        "name": "set_waypoints",
        "description": (
            "経路のウェイポイント一式を置き換える。各点は EPSG:6677 のメートル座標 (x,y)、role は "
            "start/via/goal、heading_deg は任意（方位）。最低 start と goal の2点が必要。"
            "座標が不明な場合は勝手に作らず、ユーザーに座標を尋ねるか地図でのクリック配置を促すこと。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "waypoints": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "role": {"type": "string", "enum": ["start", "via", "goal"]},
                            "heading_deg": {"type": "number"},
                        },
                        "required": ["x", "y", "role"],
                    },
                }
            },
            "required": ["waypoints"],
        },
    },
    {
        "name": "plan_route",
        "description": (
            "現在のウェイポイント・車両・オプション・レイヤで経路を生成する。生成結果は地図に表示され、"
            "曲率/勾配/feasibility 解析の要約を返す。ユーザーが『経路を作って/引いて』と言ったら呼ぶ。"
            "auto/A* には cost レイヤが必要。"
        ),
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "generate_drivable",
        "description": (
            "最新の cost レイヤから走行可能領域(drivable)を生成する。threshold 以下を通行可能とし、"
            "形態学(close/open)・最小面積・クリアランスで整形する。指定した項目だけ既定から変更される。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "threshold": {"type": "number", "description": "通行可能とする cost の上限（既定150）"},
                "close_m": {"type": "number"},
                "open_m": {"type": "number"},
                "min_area_m2": {"type": "number"},
                "clearance_m": {"type": "number", "description": "確保するクリアランス[m]"},
                "smooth_m": {"type": "number", "description": "境界平滑(メディアン)半径[m]。ギザギザ/ノイズ除去。"},
                "max_hole_m2": {"type": "number", "description": "この面積以下の閉じ穴を埋める[m²]（0=埋めない）。"},
                "keep_largest": {"type": "boolean", "description": "最大の連結領域のみ残す（島ノイズ除去・断片化解消）。"},
            },
        },
    },
    {
        "name": "generate_costmap",
        "description": "LAS 点群レイヤからコストマップ(cost/DSM/表示RGB)を生成する。las_layer_id 省略時は最新の LAS を使う。コストマップ→走行可能領域→経路の最初の一歩。",
        "input_schema": {
            "type": "object",
            "properties": {"las_layer_id": {"type": "string", "description": "省略時は最新の LAS"}},
        },
    },
    {
        "name": "set_spotting",
        "description": (
            "寄り付き(spotting)シミュレータの設定を更新する。start/target は {x,y,heading_deg}（6677m・度）。"
            "max_switchbacks=0で前進のみ/1で1切り返し、require_switchback=trueで後進差し込みを必須化。"
            "method で候補生成アルゴリズムを選択（auto=全手法をコスト比較[既定] / dubins / reeds_shepp[可変半径近似] / "
            "hybrid_astar[要走行可能領域orコストマップ]）。"
            "weights でコスト関数（距離/時間/後進/切り返し/コストマップ）の重みを調整。指定項目だけ更新。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "start": {
                    "type": "object",
                    "properties": {"x": {"type": "number"}, "y": {"type": "number"}, "heading_deg": {"type": "number"}},
                },
                "target": {
                    "type": "object",
                    "properties": {"x": {"type": "number"}, "y": {"type": "number"}, "heading_deg": {"type": "number"}},
                },
                "max_switchbacks": {"type": "integer", "enum": [0, 1]},
                "require_switchback": {"type": "boolean"},
                "method": {"type": "string", "enum": ["auto", "dubins", "reeds_shepp", "hybrid_astar"],
                           "description": "候補生成アルゴリズム。auto=全手法比較(既定)。hybrid_astarは走行可能領域/コストマップが必要。"},
                "smooth_path": {"type": "boolean", "description": "経路平滑化のON/OFF（既定ON）。曲率不連続による速度低下/蛇行を抑える。"},
                "road_width_m": {"type": "number", "description": "道幅[m]。>0 で要求クリアランス=道幅/2（車体幅ぶんの余裕を確保）。0で車両footprintを使用。"},
                "weights": {
                    "type": "object",
                    "properties": {
                        "w_distance": {"type": "number"},
                        "w_time": {"type": "number"},
                        "w_reverse": {"type": "number"},
                        "w_switchback": {"type": "number"},
                        "w_costmap": {"type": "number"},
                    },
                },
            },
        },
    },
    {
        "name": "simulate_spotting",
        "description": "現在の start/target/重み/車両で寄り付き経路を計画する。結果は地図に表示され、所要時間・前進/後進距離・切り返し回数・クリアランス・寄り付き誤差を返す。start と target が必要。",
        "input_schema": {"type": "object", "properties": {}},
    },
]


# ---------------------------------------------------------------------------
# ツール実行
# ---------------------------------------------------------------------------
def _exec_tool(name: str, args: dict, state: dict, actions: list[dict]) -> dict:
    """ツールを実行し、Claude へ返す結果(dict)を返す。副作用は state 更新 + actions 追加。"""
    if name == "get_app_state":
        return _state_snapshot(state)

    if name == "list_vehicles":
        return {"vehicles": vehicles_router.list_vehicles()}

    if name == "set_vehicle":
        vid = args.get("vehicle_id")
        if vid not in BUILTIN_IDS:
            return {"error": f"unknown vehicle: {vid}"}
        state["vehicle_id"] = vid
        actions.append({"type": "SET_VEHICLE", "vehicle_id": vid})
        return {"ok": True, "vehicle_id": vid}

    if name == "set_plan_options":
        p = state["plan"]
        changed = {}
        for k in ("mode", "algorithm", "spacing_m", "road_width_m", "min_turn_radius_m", "enforce_footprint", "enforce_min_radius", "allow_reverse", "refine_elastic_band"):
            if k in args and args[k] is not None:
                p[k] = args[k]
                changed[k] = args[k]
        actions.append({"type": "SET_PLAN_OPTIONS", **changed})
        return {"ok": True, "plan_options": p}

    if name == "set_waypoints":
        wps = args.get("waypoints") or []
        norm = []
        for w in wps:
            norm.append(
                {
                    "x": float(w["x"]),
                    "y": float(w["y"]),
                    "role": w.get("role", "via"),
                    "heading_deg": w.get("heading_deg"),
                }
            )
        state["waypoints"] = norm
        actions.append({"type": "SET_WAYPOINTS", "waypoints": norm})
        return {"ok": True, "n": len(norm)}

    if name == "plan_route":
        return _do_plan_route(state, actions)

    if name in ("generate_drivable", "generate_costmap"):
        # 生成系（ディスク書込）は1リクエスト内の実行回数を上限で打ち切る（安全）。
        used = int(state.get("_gen_calls", 0))
        if used >= MAX_GENERATE_CALLS:
            return {"error": f"生成系ツールの実行回数上限({MAX_GENERATE_CALLS}回)に達したため打ち切りました。"}
        state["_gen_calls"] = used + 1
        return _do_generate_drivable(args, actions) if name == "generate_drivable" else _do_generate_costmap(args, actions)

    if name == "set_spotting":
        s = state["spot"]
        for k in ("start", "target", "max_switchbacks", "require_switchback", "road_width_m", "method", "smooth_path", "weights"):
            if k in args and args[k] is not None:
                s[k] = args[k]
        actions.append({"type": "SET_SPOTTING", **{k: s[k] for k in s}})
        return {"ok": True, "spotting": s}

    if name == "simulate_spotting":
        return _do_simulate_spotting(state, actions)

    return {"error": f"unknown tool: {name}"}


def _do_plan_route(state: dict, actions: list[dict]) -> dict:
    wps = state["waypoints"]
    if len(wps) < 2:
        return {"error": "ウェイポイントが不足（start と goal が必要）。set_waypoints で配置してください。"}
    p = state["plan"]
    cost = _latest("cost")
    drivable = _latest("drivable")
    try:
        req = planning_router.PlanRequest(
            waypoints=[
                planning_router.XYIn(x=w["x"], y=w["y"], heading_deg=w.get("heading_deg")) for w in wps
            ],
            mode=p["mode"],
            algorithm=p["algorithm"],
            spacing_m=p.get("spacing_m") or 2.0,
            corridor_width_m=(p["road_width_m"] if (p.get("road_width_m") or 0) > 0 else None),
            enforce_footprint=bool(p.get("enforce_footprint", True)),
            enforce_min_radius=bool(p.get("enforce_min_radius", True)),
            allow_reverse=bool(p.get("allow_reverse", False)),
            refine_elastic_band=bool(p.get("refine_elastic_band", False)),
            min_turn_radius_m=p.get("min_turn_radius_m"),
            vehicle_id=state["vehicle_id"],
            costmap_layer_id=cost["id"] if cost else None,
            drivable_layer_id=drivable["id"] if drivable else None,
            no_go_polygons=(state.get("areas") or None),
        )
        res = planning_router.plan(req)
    except HTTPException as e:
        return {"error": str(e.detail)}
    actions.append({
        "type": "SET_ROUTE", "trajectory": res["trajectory"], "analysis": res["analysis"],
        "safety": res.get("safety"),
    })
    a = res["analysis"]
    sf = res.get("safety") or {}
    return {
        "ok": True,
        "length_m": round(res["trajectory"]["length_m"], 1),
        "measured_min_radius_m": res.get("measured_min_radius_m"),
        "feasible": a["feasible"],
        "safety_passed": sf.get("passed"),
        "safety_reasons": sf.get("reasons", []),
        "max_grade_pct": a.get("max_grade_pct"),
        "min_clearance_m": res.get("min_clearance_m"),
        "warning": res.get("warning"),
    }


def _do_generate_drivable(args: dict, actions: list[dict]) -> dict:
    cost = _latest("cost")
    if not cost:
        return {"error": "cost レイヤがありません。先に generate_costmap を実行してください。"}
    _keys = {"threshold", "close_m", "open_m", "min_area_m2", "clearance_m", "smooth_m", "max_hole_m2", "keep_largest"}
    params = drivable_router.GenParams(**{k: v for k, v in args.items() if k in _keys and v is not None})
    try:
        meta = drivable_router.create(drivable_router.GenRequest(cost_layer_id=cost["id"], params=params))
    except HTTPException as e:
        return {"error": str(e.detail)}
    actions.append({"type": "REFRESH_LAYERS", "select_drivable_id": meta["id"]})
    return {"ok": True, "drivable_layer_id": meta["id"], "stats": meta.get("stats")}


def _do_generate_costmap(args: dict, actions: list[dict]) -> dict:
    las_id = args.get("las_layer_id")
    if not las_id:
        las = _latest("las")
        if not las:
            return {"error": "LAS レイヤがありません。先に LAS をアップロードしてください。"}
        las_id = las["id"]
    try:
        meta = costmap_router.generate_costmap(costmap_router.CostmapRequest(las_layer_id=las_id))
    except HTTPException as e:
        return {"error": str(e.detail)}
    actions.append({"type": "REFRESH_LAYERS", "select_cost_id": meta["id"]})
    return {"ok": True, "cost_layer_id": meta["id"], "warning": meta.get("warning")}


def _do_simulate_spotting(state: dict, actions: list[dict]) -> dict:
    s = state["spot"]
    if not s.get("start") or not s.get("target"):
        return {"error": "start と target の姿勢が必要。set_spotting で設定してください。"}
    drivable = _latest("drivable")
    cost = _latest("cost")
    try:
        req = simulate_router.SpottingRequest(
            start=simulate_router.PoseIn(**s["start"]),
            target=simulate_router.PoseIn(**s["target"]),
            max_switchbacks=s.get("max_switchbacks", 1),
            require_switchback=bool(s.get("require_switchback", False)),
            method=s.get("method", "auto"),
            smooth_path=bool(s.get("smooth_path", True)),
            road_width_m=(s["road_width_m"] if (s.get("road_width_m") or 0) > 0 else None),
            vehicle_id=state["vehicle_id"],
            drivable_layer_id=drivable["id"] if drivable else None,
            costmap_layer_id=cost["id"] if cost else None,
            no_go_polygons=(state.get("areas") or None),
            weights=simulate_router.WeightsIn(**(s.get("weights") or {})),
        )
        res = simulate_router.spotting(req)
    except HTTPException as e:
        return {"error": str(e.detail)}
    actions.append({"type": "SET_SPOT_RESULT", "result": res})
    return {"ok": True, "feasible": res["feasible"], "status": res["status"], "metrics": res["metrics"]}


# ---------------------------------------------------------------------------
# システムプロンプト
# ---------------------------------------------------------------------------
def _system_prompt(state: dict) -> str:
    snap = _state_snapshot(state)
    return (
        "あなたは FMS Route Studio の経路計画コパイロットです。鉱山/建設現場の自律走行車両のための"
        "地図・経路を作る作業を支援します。回答は必ず日本語で、簡潔・具体的に。\n\n"
        "ワークフロー: LAS点群 → コストマップ(generate_costmap) → 走行可能領域(generate_drivable) → "
        "ウェイポイント/方位(set_waypoints) → 経路生成(plan_route) → 寄り付きシミュレーション(simulate_spotting)。\n\n"
        "原則:\n"
        "- 座標は EPSG:6677 のメートル系。x,y は東/北方向のメートル。方位 heading_deg は度。\n"
        "- ユーザーが操作を依頼したら、説明だけで終わらせず必ず対応するツールを呼んで実行し、結果を要約する。\n"
        "- 座標が分からないのに勝手に作らない。start/goal や姿勢の座標が必要なら、ユーザーに尋ねるか地図クリック配置を促す。\n"
        "- auto/A*（hybrid_astar・grid_astar）には cost レイヤが必要。無ければ generate_costmap を先に提案/実行する。\n"
        "- 解析(曲率/勾配/feasibility)や違反があれば、原因（半径不足・道幅・領域非連結など）と次の一手を示す。\n"
        "- 破壊的でない操作（経路生成・解析・寄り付き）は確認なしで実行してよい。レイヤ生成も実行してよい。\n\n"
        "現在の状態:\n" + json.dumps(snap, ensure_ascii=False, indent=2)
    )


# ---------------------------------------------------------------------------
# プロバイダ解決（Claude / OpenAI互換: Groq・OpenRouter・Ollama・OpenAI 等）
# ---------------------------------------------------------------------------
# OpenAI 互換は1実装で複数の無料/ローカルバックエンドを賄える（base_url を差し替えるだけ）。
# 鍵を貼らずに使う例: GROQ_API_KEY(無料) / OPENROUTER_API_KEY(無料モデル有) /
#   FRS_AI_BASE_URL=http://localhost:11434/v1 (Ollama・鍵不要・完全ローカル)。
_OPENAI_PRESETS = {
    "groq": ("https://api.groq.com/openai/v1", "GROQ_API_KEY", "llama-3.3-70b-versatile", "Groq"),
    "openrouter": ("https://openrouter.ai/api/v1", "OPENROUTER_API_KEY", "meta-llama/llama-3.3-70b-instruct", "OpenRouter"),
    "openai": ("https://api.openai.com/v1", "OPENAI_API_KEY", "gpt-4o-mini", "OpenAI"),
}


def _resolve_provider() -> dict:
    """環境変数から使用プロバイダを決める。明示(FRS_AI_PROVIDER)→自動検出の順。"""
    env = os.environ
    forced = (env.get("FRS_AI_PROVIDER") or "").strip().lower()
    model_override = env.get("FRS_AI_MODEL")

    def openai_cfg(preset_key: str | None) -> dict:
        # preset があれば既定を使い、無ければ FRS_AI_BASE_URL/FRS_AI_API_KEY の汎用(Ollama等)。
        if preset_key and preset_key in _OPENAI_PRESETS:
            base, key_env, default_model, label = _OPENAI_PRESETS[preset_key]
            base = env.get("FRS_AI_BASE_URL", base)
            api_key = env.get(key_env) or env.get("FRS_AI_API_KEY")
        else:
            base = env.get("FRS_AI_BASE_URL")
            api_key = env.get("FRS_AI_API_KEY") or env.get("OPENAI_API_KEY") or "not-needed"
            default_model, label = None, "OpenAI互換"
        return {
            "kind": "openai", "label": label, "base_url": base,
            "api_key": api_key, "model": model_override or default_model,
        }

    # 明示指定
    if forced == "anthropic":
        return {"kind": "anthropic", "label": "Claude", "model": model_override or MODEL}
    if forced in _OPENAI_PRESETS:
        return openai_cfg(forced)
    if forced in ("openai_compat", "ollama", "local", "custom"):
        return openai_cfg(None)

    # 自動検出
    if env.get("ANTHROPIC_API_KEY") or env.get("ANTHROPIC_AUTH_TOKEN"):
        return {"kind": "anthropic", "label": "Claude", "model": model_override or MODEL}
    if env.get("GROQ_API_KEY"):
        return openai_cfg("groq")
    if env.get("OPENROUTER_API_KEY"):
        return openai_cfg("openrouter")
    if env.get("FRS_AI_BASE_URL"):  # Ollama 等ローカル（鍵不要）
        return openai_cfg(None)
    if env.get("OPENAI_API_KEY"):
        return openai_cfg("openai")
    return {"kind": "none", "label": None, "model": None}


def _provider_ready(cfg: dict) -> tuple[bool, str | None]:
    if cfg["kind"] == "none":
        return False, None
    if cfg["kind"] == "openai":
        if not cfg.get("base_url"):
            return False, "FRS_AI_BASE_URL（OpenAI互換エンドポイント）を設定してください。"
        if not cfg.get("model"):
            return False, "FRS_AI_MODEL（使用モデル名）を設定してください。"
        if not cfg.get("api_key"):
            return False, "API キー（FRS_AI_API_KEY 等）を設定してください。"
    return True, None


def _openai_tools() -> list[dict]:
    return [
        {"type": "function", "function": {"name": t["name"], "description": t["description"],
                                          "parameters": t.get("input_schema") or {"type": "object", "properties": {}}}}
        for t in TOOLS
    ]


def _oai_create(openai_mod, client, model, messages, tools, *, max_tries: int = 3):
    """OpenAI互換 chat.completions.create を一時エラー(429/5xx/接続)で指数バックオフ再試行。

    BadRequestError(400, tool_use_failed 等)は再試行せず呼び出し側へ（本文形式ツール復元のため）。
    """
    import time

    delay = 1.0
    for attempt in range(max_tries):
        try:
            return client.chat.completions.create(
                model=model, messages=messages, tools=tools, tool_choice="auto", max_tokens=2048
            )
        except openai_mod.BadRequestError:
            raise
        except openai_mod.RateLimitError:
            if attempt == max_tries - 1:
                raise
        except openai_mod.APIStatusError as e:
            if getattr(e, "status_code", 0) < 500 or attempt == max_tries - 1:
                raise
        except openai_mod.APIConnectionError:
            if attempt == max_tries - 1:
                raise
        time.sleep(delay)
        delay *= 2


def _parse_textual_toolcalls(text: str) -> list[tuple[str, dict]]:
    """一部モデル(Groq Llama 等)は tool 呼び出しを本文に `<function=NAME{JSON}</function>`
    形式で吐き、API が 400 tool_use_failed を返すことがある。その文字列から (name, args) を復元する。"""
    out: list[tuple[str, dict]] = []
    if not text:
        return out
    for m in re.finditer(r"<function=([A-Za-z0-9_]+)", text):
        name = m.group(1)
        i = text.find("{", m.end())
        if i < 0:
            out.append((name, {}))
            continue
        depth, end = 0, -1
        for j in range(i, len(text)):
            if text[j] == "{":
                depth += 1
            elif text[j] == "}":
                depth -= 1
                if depth == 0:
                    end = j
                    break
        try:
            args = json.loads(text[i:end + 1]) if end > 0 else {}
        except Exception:
            args = {}
        out.append((name, args if isinstance(args, dict) else {}))
    return out


def _extract_failed_generation(err) -> str:
    """openai BadRequestError から Groq の failed_generation 文字列を取り出す。"""
    cands = []
    body = getattr(err, "body", None)
    if isinstance(body, dict):
        cands += [body, body.get("error") if isinstance(body.get("error"), dict) else {}]
    try:
        j = err.response.json()
        if isinstance(j, dict):
            cands += [j, j.get("error") if isinstance(j.get("error"), dict) else {}]
    except Exception:
        pass
    for c in cands:
        fg = c.get("failed_generation") if isinstance(c, dict) else None
        if isinstance(fg, str) and fg:
            return fg
    return ""


# ---------------------------------------------------------------------------
# エンドポイント
# ---------------------------------------------------------------------------
@router.get("/status")
def status() -> dict:
    """AI が利用可能か（プロバイダ/鍵が設定済みか）を返す。"""
    cfg = _resolve_provider()
    ready, why = _provider_ready(cfg)
    return {"available": ready, "provider": cfg["label"], "model": cfg.get("model"), "detail": why}


def _run_anthropic(cfg, system, history, state, actions, steps) -> str:
    import anthropic

    client = anthropic.Anthropic()
    model = cfg["model"]

    def create(messages):
        base = dict(model=model, max_tokens=4096, system=system, tools=TOOLS, messages=messages)
        try:
            return client.messages.create(**base, thinking={"type": "adaptive"}, output_config={"effort": "medium"})
        except anthropic.BadRequestError:
            return client.messages.create(**base)

    messages = list(history)
    resp = None
    try:
        for _ in range(MAX_TOOL_ITERS):
            resp = create(messages)
            if resp.stop_reason != "tool_use":
                break
            messages.append({"role": "assistant", "content": resp.content})  # thinking ブロック保持
            results = []
            for block in resp.content:
                if block.type != "tool_use":
                    continue
                args = block.input if isinstance(block.input, dict) else {}
                out = _exec_tool(block.name, args, state, actions)
                steps.append({"tool": block.name, "input": args, "ok": "error" not in out})
                results.append({"type": "tool_result", "tool_use_id": block.id,
                                "content": json.dumps(out, ensure_ascii=False), "is_error": "error" in out})
            messages.append({"role": "user", "content": results})
    except anthropic.AuthenticationError:
        raise HTTPException(503, "Anthropic の資格情報が無効です。")
    except anthropic.APIStatusError as e:
        raise HTTPException(502, f"Claude API エラー: {e.status_code} {getattr(e, 'message', '')}")
    return "".join(b.text for b in (resp.content if resp else []) if getattr(b, "type", None) == "text").strip()


def _run_openai(cfg, system, history, state, actions, steps) -> str:
    import openai

    client = openai.OpenAI(api_key=cfg.get("api_key") or "not-needed", base_url=cfg.get("base_url"))
    model = cfg["model"]
    tools = _openai_tools()
    messages: list[dict] = [{"role": "system", "content": system}] + list(history)
    reply = ""
    try:
        for _ in range(MAX_TOOL_ITERS):
            try:
                resp = _oai_create(openai, client, model, messages, tools)
            except openai.BadRequestError as e:
                # Groq/Llama が本文形式でツールを吐き 400 tool_use_failed → 失敗文字列から復元して継続。
                calls = _parse_textual_toolcalls(_extract_failed_generation(e))
                if not calls:
                    raise
                tcs = [
                    {"id": f"fc_{k}", "type": "function",
                     "function": {"name": nm, "arguments": json.dumps(a, ensure_ascii=False)}}
                    for k, (nm, a) in enumerate(calls)
                ]
                messages.append({"role": "assistant", "content": None, "tool_calls": tcs})
                for tc, (nm, a) in zip(tcs, calls):
                    out = _exec_tool(nm, a, state, actions)
                    steps.append({"tool": nm, "input": a, "ok": "error" not in out})
                    messages.append({"role": "tool", "tool_call_id": tc["id"],
                                     "content": json.dumps(out, ensure_ascii=False)})
                continue
            msg = resp.choices[0].message
            if not getattr(msg, "tool_calls", None):
                reply = msg.content or ""
                break
            messages.append({
                "role": "assistant",
                "content": msg.content or None,
                "tool_calls": [
                    {"id": tc.id, "type": "function",
                     "function": {"name": tc.function.name, "arguments": tc.function.arguments}}
                    for tc in msg.tool_calls
                ],
            })
            for tc in msg.tool_calls:
                try:
                    args = json.loads(tc.function.arguments or "{}")
                except Exception:
                    args = {}
                if not isinstance(args, dict):  # フリーモデルが null/配列を返すことがある
                    args = {}
                out = _exec_tool(tc.function.name, args, state, actions)
                steps.append({"tool": tc.function.name, "input": args, "ok": "error" not in out})
                messages.append({"role": "tool", "tool_call_id": tc.id,
                                 "content": json.dumps(out, ensure_ascii=False)})
        else:
            reply = "(ツール呼び出しが上限に達しました)"
    except openai.AuthenticationError:
        raise HTTPException(503, "API キーが無効です。")
    except openai.APIStatusError as e:
        raise HTTPException(502, f"{cfg['label']} API エラー: {getattr(e, 'status_code', '?')} {getattr(e, 'message', '')}")
    except openai.APIConnectionError:
        raise HTTPException(502, f"{cfg['label']} に接続できません（base_url/サーバ起動を確認）。")
    return (reply or "").strip()


@router.post("/chat")
def chat(req: ChatRequest) -> dict:
    cfg = _resolve_provider()
    ready, why = _provider_ready(cfg)
    if not ready:
        raise HTTPException(
            503,
            why or "AI プロバイダが未設定です。無料の例: GROQ_API_KEY を設定 / "
            "OPENROUTER_API_KEY を設定 / FRS_AI_BASE_URL=http://localhost:11434/v1 (Ollama・鍵不要) / "
            "ANTHROPIC_API_KEY を設定。設定後 API サーバを再起動してください。",
        )

    state = _init_state(req.context)
    actions: list[dict] = []
    steps: list[dict] = []
    system = _system_prompt(state)
    history = [{"role": m.role, "content": m.content} for m in req.messages if m.content.strip()]
    if not history:
        raise HTTPException(400, "messages が空です")

    if cfg["kind"] == "anthropic":
        reply = _run_anthropic(cfg, system, history, state, actions, steps)
    else:
        reply = _run_openai(cfg, system, history, state, actions, steps)

    return {"reply": reply or "(応答なし)", "actions": actions, "steps": steps, "provider": cfg["label"]}
