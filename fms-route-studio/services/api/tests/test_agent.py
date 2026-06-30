"""AI アシスタント(agent) ルーターのテスト。

Claude API は呼ばず、鍵未設定時のガードとツールディスパッチャの純粋部分を検証する。
"""
import os

from fastapi.testclient import TestClient

from app.main import app
from app.routers import agent

client = TestClient(app)


_PROVIDER_ENVS = [
    "ANTHROPIC_API_KEY", "ANTHROPIC_AUTH_TOKEN", "GROQ_API_KEY", "OPENROUTER_API_KEY",
    "OPENAI_API_KEY", "FRS_AI_BASE_URL", "FRS_AI_API_KEY", "FRS_AI_PROVIDER", "FRS_AI_MODEL",
]


def _no_key():
    for k in _PROVIDER_ENVS:
        os.environ.pop(k, None)


def test_status_no_provider():
    _no_key()
    r = client.get("/api/agent/status")
    assert r.status_code == 200
    body = r.json()
    assert body["available"] is False
    assert body["model"] is None and body["provider"] is None


def test_chat_without_provider_returns_503():
    _no_key()
    r = client.post("/api/agent/chat", json={"messages": [{"role": "user", "content": "hi"}], "context": {}})
    assert r.status_code == 503


def test_status_detects_groq():
    _no_key()
    os.environ["GROQ_API_KEY"] = "gsk_test"
    try:
        body = client.get("/api/agent/status").json()
        assert body["available"] is True
        assert body["provider"] == "Groq"
        assert body["model"]  # 既定 or FRS_AI_MODEL
    finally:
        _no_key()


def test_status_ollama_needs_model():
    _no_key()
    os.environ["FRS_AI_BASE_URL"] = "http://localhost:11434/v1"
    try:
        body = client.get("/api/agent/status").json()
        # base はあるがモデル未指定 → not ready（モデル設定を促す）
        assert body["available"] is False
        assert "FRS_AI_MODEL" in (body["detail"] or "")
    finally:
        _no_key()


def test_openai_tools_format():
    tools = agent._openai_tools()
    assert all(t["type"] == "function" and "name" in t["function"] and "parameters" in t["function"] for t in tools)
    assert {t["function"]["name"] for t in tools} >= {"plan_route", "generate_costmap", "set_waypoints"}


def test_parse_textual_toolcalls_groq_fallback():
    # Groq/Llama が 400 tool_use_failed 時に返す failed_generation 形式を復元
    fg = '<function=set_vehicle{"vehicle_id": "HM400"}</function>'
    calls = agent._parse_textual_toolcalls(fg)
    assert calls == [("set_vehicle", {"vehicle_id": "HM400"})]
    # ネストした JSON（weights）も balanced 抽出
    fg2 = '<function=set_spotting{"weights": {"w_reverse": 2, "w_switchback": 5}}</function>'
    calls2 = agent._parse_textual_toolcalls(fg2)
    assert calls2[0][0] == "set_spotting" and calls2[0][1]["weights"]["w_reverse"] == 2
    # 複数
    fg3 = '<function=set_vehicle{"vehicle_id":"HD605"}</function> and <function=plan_route{}</function>'
    names = [c[0] for c in agent._parse_textual_toolcalls(fg3)]
    assert names == ["set_vehicle", "plan_route"]


def test_extract_failed_generation():
    class E:
        body = {"error": {"code": "tool_use_failed", "failed_generation": "<function=plan_route{}</function>"}}
    assert "plan_route" in agent._extract_failed_generation(E())


def test_run_openai_recovers_from_tool_use_failed(monkeypatch):
    """Groq の tool_use_failed(400) → failed_generation から復元してツール実行できる。"""
    import openai

    class Boom(openai.BadRequestError):
        def __init__(self):
            self.body = {"error": {"code": "tool_use_failed",
                                   "failed_generation": '<function=set_vehicle{"vehicle_id":"HM400"}</function>'}}

    class FakeMsg:
        def __init__(self, content=None, tool_calls=None):
            self.content = content
            self.tool_calls = tool_calls

    class FakeResp:
        def __init__(self, msg):
            self.choices = [type("C", (), {"message": msg})()]

    class FakeCompletions:
        def __init__(self):
            self.n = 0

        def create(self, **kw):
            self.n += 1
            if self.n == 1:
                raise Boom()
            return FakeResp(FakeMsg(content="車両をHM400にしました。"))

    class FakeClient:
        def __init__(self, **kw):
            self.chat = type("Ch", (), {"completions": FakeCompletions()})()

    monkeypatch.setattr(openai, "OpenAI", FakeClient)
    cfg = {"kind": "openai", "label": "Groq", "base_url": "http://x/v1", "api_key": "k", "model": "m"}
    actions, steps = [], []
    reply = agent._run_openai(cfg, "sys", [{"role": "user", "content": "車両をHM400に"}],
                              agent._init_state({}), actions, steps)
    assert "HM400" in reply
    assert any(a["type"] == "SET_VEHICLE" and a["vehicle_id"] == "HM400" for a in actions)
    assert steps and steps[0]["tool"] == "set_vehicle"


def test_run_openai_tool_loop(monkeypatch):
    """OpenAI 互換パスの tool-use ループ配線をモックで検証（鍵不要）。"""
    import openai

    class FakeFn:
        def __init__(self, name, args):
            self.name = name
            self.arguments = args

    class FakeTC:
        def __init__(self, id, name, args):
            self.id = id
            self.function = FakeFn(name, args)

    class FakeMsg:
        def __init__(self, content=None, tool_calls=None):
            self.content = content
            self.tool_calls = tool_calls

    class FakeResp:
        def __init__(self, msg):
            self.choices = [type("C", (), {"message": msg})()]

    class FakeCompletions:
        def __init__(self):
            self.n = 0

        def create(self, **kw):
            self.n += 1
            if self.n == 1:
                return FakeResp(FakeMsg(tool_calls=[
                    FakeTC("c1", "set_waypoints",
                           '{"waypoints":[{"x":0,"y":0,"role":"start"},{"x":20,"y":0,"role":"goal"}]}'),
                ]))
            return FakeResp(FakeMsg(content="waypointを設定しました。"))

    class FakeChat:
        def __init__(self):
            self.completions = FakeCompletions()

    class FakeClient:
        def __init__(self, **kw):
            self.chat = FakeChat()

    monkeypatch.setattr(openai, "OpenAI", FakeClient)
    cfg = {"kind": "openai", "label": "Groq", "base_url": "http://x/v1", "api_key": "k", "model": "m"}
    actions, steps = [], []
    reply = agent._run_openai(cfg, "sys", [{"role": "user", "content": "経路準備して"}], agent._init_state({}), actions, steps)
    assert "waypoint" in reply
    assert any(a["type"] == "SET_WAYPOINTS" for a in actions)
    assert steps and steps[0]["tool"] == "set_waypoints"


def test_tools_registered():
    names = {t["name"] for t in agent.TOOLS}
    assert {"plan_route", "generate_costmap", "generate_drivable", "simulate_spotting", "set_waypoints"} <= names
    # 各ツールに input_schema がある
    assert all("input_schema" in t and "name" in t and "description" in t for t in agent.TOOLS)


def test_init_state_defaults():
    st = agent._init_state({})
    assert st["plan"]["mode"] == "waypoint_guided"
    assert st["plan"]["algorithm"] == "spline"
    assert st["spot"]["max_switchbacks"] == 1
    assert st["waypoints"] == []


def test_exec_tool_set_vehicle_and_options():
    st = agent._init_state({})
    actions: list[dict] = []

    out = agent._exec_tool("set_vehicle", {"vehicle_id": "HD785"}, st, actions)
    assert out["ok"] and st["vehicle_id"] == "HD785"
    assert actions[-1] == {"type": "SET_VEHICLE", "vehicle_id": "HD785"}

    out = agent._exec_tool("set_vehicle", {"vehicle_id": "NOPE"}, st, actions)
    assert "error" in out

    out = agent._exec_tool("set_plan_options", {"mode": "auto", "algorithm": "hybrid_astar"}, st, actions)
    assert st["plan"]["mode"] == "auto" and st["plan"]["algorithm"] == "hybrid_astar"
    assert actions[-1]["type"] == "SET_PLAN_OPTIONS"


def test_exec_tool_set_waypoints_emits_action():
    st = agent._init_state({})
    actions: list[dict] = []
    wps = [
        {"x": 1.0, "y": 2.0, "role": "start", "heading_deg": 90},
        {"x": 5.0, "y": 6.0, "role": "goal"},
    ]
    out = agent._exec_tool("set_waypoints", {"waypoints": wps}, st, actions)
    assert out["ok"] and out["n"] == 2
    assert st["waypoints"][0]["x"] == 1.0
    assert actions[-1]["type"] == "SET_WAYPOINTS"
    assert len(actions[-1]["waypoints"]) == 2


def test_exec_tool_get_app_state():
    st = agent._init_state({"vehicle_id": "HM400"})
    out = agent._exec_tool("get_app_state", {}, st, [])
    assert out["vehicle_id"] == "HM400"
    assert "resolved_layers" in out and "layers" in out


def test_plan_route_needs_waypoints():
    st = agent._init_state({})
    out = agent._exec_tool("plan_route", {}, st, [])
    assert "error" in out  # waypoint 不足


def test_oai_create_retries_then_succeeds(monkeypatch):
    monkeypatch.setattr("time.sleep", lambda *a: None)

    class FakeMod:
        class BadRequestError(Exception): ...
        class RateLimitError(Exception): ...
        class APIStatusError(Exception): ...
        class APIConnectionError(Exception): ...

    calls = {"n": 0}

    class Comp:
        def create(self, **kw):
            calls["n"] += 1
            if calls["n"] < 3:
                raise FakeMod.RateLimitError()
            return "OK"

    class Client:
        def __init__(self):
            self.chat = type("C", (), {"completions": Comp()})()

    out = agent._oai_create(FakeMod, Client(), "m", [], [])
    assert out == "OK" and calls["n"] == 3


def test_oai_create_badrequest_not_retried(monkeypatch):
    import pytest
    monkeypatch.setattr("time.sleep", lambda *a: None)

    class FakeMod:
        class BadRequestError(Exception): ...
        class RateLimitError(Exception): ...
        class APIStatusError(Exception): ...
        class APIConnectionError(Exception): ...

    calls = {"n": 0}

    class Comp:
        def create(self, **kw):
            calls["n"] += 1
            raise FakeMod.BadRequestError()

    class Client:
        def __init__(self):
            self.chat = type("C", (), {"completions": Comp()})()

    with pytest.raises(FakeMod.BadRequestError):
        agent._oai_create(FakeMod, Client(), "m", [], [])
    assert calls["n"] == 1
