"""複数台の簡易シミュレーション（Phase D 骨格: 区間予約Mutex＋優先度＋規定減速停止）。

各車は自分の経路を速度プロファイル（最大速度・加速・減速）で進む。経路の重なり（Phase B の競合区間）を
**共有リソース(区間)**とみなし、相手が占有中の区間へは入れない＝手前で**規定減速度＋マージン**で停止して
待ち、相手が抜けたら再発進する（鉄道のブロッキング/Mutexと同じ）。経路末端でも停止する。

優先度: 自由な区間を同時に欲した場合は priority 小（＝高優先）が先に確保。膠着（全車停止して
循環待ち）は**デッドロック検出**して報告する（解消＝待避所は Phase C）。

時間離散ステップ。座標は作業CRS(メートル)。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .conflict import detect_conflicts


@dataclass
class SimVehicle:
    points: np.ndarray          # 経路中心線 (N,2)[m]
    v_max: float = 5.0          # 最大速度[m/s]
    accel: float = 0.5          # 加速[m/s^2]
    decel: float = 1.0          # 減速[m/s^2]（走行中の制動カーブ v=√(2·decel·d) に使う通常減速度）
    reserve_decel: float | None = None  # 予約距離 v_max²/(2·decel) 用の保守側減速度（積載時等。None=decel）
    half_width: float = 1.7     # 車幅/2[m]
    half_length: float = 3.0    # 車長/2[m]（区間占有を車体長ぶん膨張＝Mutexで車体が重ならない）
    priority: int = 0           # 小さいほど高優先
    start_time: float = 0.0     # 出発時刻[s]
    name: str = ""


@dataclass
class _Zone:
    """2経路が共有する競合区間（Mutex リソース）。enter/exit は経路ごとの弧長[m]。"""
    routes: tuple[int, int]
    enter: dict[int, float]
    exit: dict[int, float]
    owner: int | None = None    # 現在占有中の車両 index


@dataclass
class SimResult:
    traces: list[list[dict]] = field(default_factory=list)  # 車両ごと [{t,s,x,y,heading_deg,v,state}]
    events: list[dict] = field(default_factory=list)        # {t,vehicle,type,zone}
    deadlock: bool = False
    deadlock_time: float | None = None
    makespan: float = 0.0
    status: str = "OK"
    min_separation_m: float | None = None  # 全時刻・全ペアの中心間最接近距離[m]
    min_sep_time: float | None = None
    min_sep_pair: tuple[int, int] | None = None  # 最接近した車両ペア
    collision: bool = False                 # 最接近 < 両車の半幅和（車体重なり＝危険/Mutex破れ）
    auto_bays: list[dict] = field(default_factory=list)  # 自動配置した待避所 [{vehicle,s_center,offset,side}]
    wait_time_s: list[float] = field(default_factory=list)  # 車両ごとの待機時間[s]（区間待ちで停止）
    total_wait_s: float = 0.0               # 全車の待機時間合計[s]（トラフィック効率の指標）
    travel_time_s: list[float] = field(default_factory=list)  # 車両ごとの所要時間[s]（出発→到達）


def _cum_s(pts: np.ndarray) -> np.ndarray:
    if len(pts) < 2:
        return np.zeros(len(pts))
    return np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])))])


def _rect_corners(x: float, y: float, heading_deg: float, hl: float, hw: float):
    """車体矩形（全長2hl×全幅2hw、中心(x,y)・向きheading）の4隅。"""
    th = math.radians(heading_deg)
    c, s = math.cos(th), math.sin(th)
    return [(x + dx * c - dy * s, y + dx * s + dy * c)
            for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def _rects_overlap(r1, r2) -> bool:
    """凸四角形どうしの重なり判定（分離軸定理）。"""
    for r in (r1, r2):
        for k in range(4):
            x1, y1 = r[k]
            x2, y2 = r[(k + 1) % 4]
            ax, ay = -(y2 - y1), (x2 - x1)  # 辺法線（正規化不要=射影の大小比較のみ）
            p1 = [px * ax + py * ay for px, py in r1]
            p2 = [px * ax + py * ay for px, py in r2]
            if max(p1) < min(p2) or max(p2) < min(p1):
                return False  # 分離軸あり=非交差
    return True


def _bodies_overlap(fi: dict, fj: dict, vi: "SimVehicle", vj: "SimVehicle") -> bool:
    """トレースフレーム2つの車体（向き付き矩形）が重なるか。

    旧実装の「中心間距離 < 半幅和」は車長を無視し、縦方向（進行方向）の車体重なりを
    見逃していた（例: 10m級車体が斜め交差ですれ違うと中心間6mでも車体は接触）。
    """
    d = math.hypot(fi["x"] - fj["x"], fi["y"] - fj["y"])
    reach = math.hypot(vi.half_length, vi.half_width) + math.hypot(vj.half_length, vj.half_width)
    if d > reach:
        return False  # 外接円が離れている=重なり得ない（SAT省略の高速パス）
    return _rects_overlap(
        _rect_corners(fi["x"], fi["y"], fi["heading_deg"], vi.half_length, vi.half_width),
        _rect_corners(fj["x"], fj["y"], fj["heading_deg"], vj.half_length, vj.half_width),
    )


def _pose_at(pts: np.ndarray, s_arr: np.ndarray, s: float):
    """弧長 s の (x,y,heading[deg])。線形補間。"""
    n = len(pts)
    if n == 0:
        return 0.0, 0.0, 0.0
    if s <= 0:
        i = 0
    elif s >= s_arr[-1]:
        i = n - 2 if n >= 2 else 0
    else:
        i = int(np.searchsorted(s_arr, s) - 1)
        i = max(0, min(i, n - 2))
    if n < 2:
        return float(pts[0, 0]), float(pts[0, 1]), 0.0
    seg = max(s_arr[i + 1] - s_arr[i], 1e-9)
    f = (s - s_arr[i]) / seg
    x = pts[i, 0] + f * (pts[i + 1, 0] - pts[i, 0])
    y = pts[i, 1] + f * (pts[i + 1, 1] - pts[i, 1])
    head = np.degrees(np.arctan2(pts[i + 1, 1] - pts[i, 1], pts[i + 1, 0] - pts[i, 0]))
    return float(x), float(y), float(head)


def simulate_fleet(vehicles: list[SimVehicle], *, dt: float = 0.2, gap_m: float = 2.0,
                   max_time: float = 600.0, cell: float = 0.5, clearance_m: float = 0.0) -> SimResult:
    """複数台の簡易シミュレーション。返値 SimResult（各車の時系列 trace ＋ イベント ＋ デッドロック）。

    gap_m: 占有区間手前で止まる際の停止マージン[m]（規定減速度に上乗せの安全余裕）。
    """
    nv = len(vehicles)
    if nv == 0:
        return SimResult(status="NO_VEHICLES")
    pts = [np.asarray(v.points, float) for v in vehicles]
    s_arr = [_cum_s(p) for p in pts]
    length = [float(s_arr[i][-1]) if len(s_arr[i]) else 0.0 for i in range(nv)]

    # 競合区間 → Mutex リソース（経路ごとの enter/exit 弧長）。
    # **同方向の重なり(追従/合流)は Mutex を作らない**（排他にすると追従と矛盾し膠着する）。
    # 前方車追従(car-following)が車間を保つ。Mutex は交差・対向(反対向き)にのみ適用する。
    routes = [{"points": pts[i], "half_width": vehicles[i].half_width} for i in range(nv)]
    zones: list[_Zone] = []
    for c in detect_conflicts(routes, cell=cell, clearance_m=clearance_m):
        a_en = min((iv.s_start for iv in c.a_intervals), default=0.0)
        a_ex = max((iv.s_end for iv in c.a_intervals), default=0.0)
        b_en = min((iv.s_start for iv in c.b_intervals), default=0.0)
        b_ex = max((iv.s_end for iv in c.b_intervals), default=0.0)
        # 同方向判定: 共有区間**全体**を等分サンプリングし、対応点どうしの進行方向の内積の
        # 平均で判定する。同方向の重なり（追従/併走）は区間の同じ割合 f が物理的にほぼ同じ場所を
        # 指すため全サンプルで cos≈1、対向は cos≈-1、交差は交差角の cos になる。
        # 閾値 0.85（±約30°）: それ以下（60°交差等）は Mutex を作る。
        # 旧: 中点1点の cos>0.3（±72°）→ 60°交差でも「同方向」扱いになり車体接触を許していた。
        n_smp = 7
        cs = []
        for k_ in range(n_smp):
            f = k_ / (n_smp - 1)
            ta = math.radians(_pose_at(pts[c.a], s_arr[c.a], a_en + f * (a_ex - a_en))[2])
            tb = math.radians(_pose_at(pts[c.b], s_arr[c.b], b_en + f * (b_ex - b_en))[2])
            cs.append(math.cos(ta - tb))
        same_dir = (sum(cs) / len(cs)) > 0.85
        if same_dir:
            continue  # 同方向は car-following が処理（区間排他にしない）
        zones.append(_Zone(routes=(c.a, c.b), enter={c.a: a_en, c.b: b_en}, exit={c.a: a_ex, c.b: b_ex}))

    def zones_of(vi: int):
        return [z for z in zones if vi in z.routes]

    s = [0.0] * nv
    v = [0.0] * nv
    done = [length[i] <= 1e-9 for i in range(nv)]
    traces: list[list[dict]] = [[] for _ in range(nv)]
    events: list[dict] = []
    order = sorted(range(nv), key=lambda i: (vehicles[i].priority, i))  # 高優先（小）から処理

    def record(t):
        for i in range(nv):
            x, y, hd = _pose_at(pts[i], s_arr[i], s[i])
            if vehicles[i].points is not None and not (s[i] <= 0 and t < vehicles[i].start_time):
                # 後進ではないので heading は進行方向。停止/未発進も姿勢は記録。
                pass
            state = "done" if done[i] else ("wait" if v[i] < 0.05 and t >= vehicles[i].start_time else "run")
            traces[i].append({"t": round(t, 2), "s": round(s[i], 3), "x": x, "y": y,
                              "heading_deg": round(hd, 1), "v": round(v[i], 3), "state": state})

    record(0.0)
    t = 0.0
    stall_steps = 0
    deadlock = False
    deadlock_time = None
    n_steps = int(max_time / dt)
    for _ in range(n_steps):
        t += dt
        moved_total = 0.0
        active_blocked = 0
        n_active = 0
        # ステップ開始時の各車の現在姿勢（前方車追従 car-following の判定用）。
        cur_pose = [_pose_at(pts[k], s_arr[k], s[k]) for k in range(nv)]  # (x, y, heading_deg)
        cur_xy = [(p[0], p[1]) for p in cur_pose]
        for i in order:
            if done[i]:
                continue
            if t < vehicles[i].start_time:
                continue
            n_active += 1
            veh = vehicles[i]
            # 予約距離は保守側（積載時）減速度で計算＝実制動が伸びても Mutex ゾーン内で
            # 停止しきれない事態を防ぐ。走行中の制動カーブ(v_stop)は通常減速度 decel を使う
            # （積載値で統一すると停止点の数百m手前から徐行が始まり非現実的になる）。
            r_dec = veh.reserve_decel if veh.reserve_decel else veh.decel
            reserve_dist = veh.v_max * veh.v_max / (2.0 * max(r_dec, 1e-6)) + gap_m + 1.0
            hl = veh.half_length
            # まだ通過し終えていない競合区間について、**接近時(制動距離内)に予約**を試みる。
            # 占有区間は車体長ぶん膨張(enter-hl 〜 exit+hl)＝車体が区間にかかる間ずっと占有とみなし、
            # Mutex で相手と車体が重ならないようにする。空けば確保、相手占有中なら手前(enter-hl-gap)で停止。
            targets = [length[i]]
            blocked = False
            for z in zones_of(i):
                en, ex = z.enter[i] - hl, z.exit[i] + hl   # 車体長ぶん膨張した占有区間
                if s[i] > ex - 1e-6:
                    continue  # この区間は通過済み（もう要らない）
                if z.owner == i:
                    continue  # 予約済み＝通れる
                if z.owner is None:
                    if s[i] >= en - reserve_dist:  # 制動距離内に入った＝先に予約
                        z.owner = i
                        events.append({"t": round(t, 2), "vehicle": i, "type": "reserve", "zone": z.routes})
                        continue
                    # まだ遠い: 制約なし（後で予約）
                    continue
                # 相手が占有中 → 入れない
                if s[i] < en - 1e-6:
                    targets.append(max(0.0, en - gap_m))   # 占有区間手前で停止して待つ
                    blocked = True
                else:
                    targets.append(s[i])                   # 既に区間内で未確保＝その場停止（膠着）
                    blocked = True
            # --- 前方車追従(car-following): 自分の進路上・前方(車線内)に他車がいれば、その手前で止まる。
            #     区間Mutexは「区間の排他」だけなので、同一車線の追従/追越し・停止車への追突はこれで防ぐ。
            #     浅い合流（同方向扱い=Mutexなし）では互いを「前方車」と認識して相互停止＝偽デッドロック
            #     になり得るため、**相互認識時は高優先が進み低優先だけが譲る**（通常の追従=前後関係が
            #     明確な場合は相互にならないので従来どおり）。---
            pi = pts[i]
            ix, iy = cur_xy[i]
            for j in range(nv):
                if j == i or done[j] or t < vehicles[j].start_time:
                    continue
                jx, jy = cur_xy[j]
                dd = np.hypot(pi[:, 0] - jx, pi[:, 1] - jy)
                m = int(np.argmin(dd))
                lateral = float(dd[m])
                s_proj = float(s_arr[i][m])
                if lateral <= veh.half_width + vehicles[j].half_width + gap_m and s_proj > s[i] + 1e-6:
                    # 相互認識か（j から見ても自分が前方か）を判定。相互かつ**ほぼ同方向**（浅い合流）
                    # かつ自分が高優先なら譲らない（低優先 j だけが譲る＝偽デッドロック回避）。
                    # 対向（ヘッドオン）に適用すると高優先が突っ込むため、進行方向 cos>0.5 に限定
                    # （対向は従来どおり両者停止→DEADLOCK 報告→待避所で解消）。
                    ddj = np.hypot(pts[j][:, 0] - ix, pts[j][:, 1] - iy)
                    mj = int(np.argmin(ddj))
                    mutual = (float(ddj[mj]) <= veh.half_width + vehicles[j].half_width + gap_m
                              and float(s_arr[j][mj]) > s[j] + 1e-6)
                    same_heading = math.cos(math.radians(cur_pose[i][2] - cur_pose[j][2])) > 0.5
                    if mutual and same_heading and (veh.priority, i) < (vehicles[j].priority, j):
                        continue  # 高優先側: 低優先 j が譲る（j 側の追従制約は残る）
                    lead_gap = s_proj - s[i] - (hl + vehicles[j].half_length) - gap_m  # 前方車体後端まで
                    targets.append(s[i] + max(0.0, lead_gap))
                    if lead_gap < 0.5:
                        blocked = True
            target = min(targets)
            d = target - s[i]
            # 規定減速で target に止まれる速度上限 v=sqrt(2*decel*d)
            v_stop = float(np.sqrt(max(0.0, 2.0 * veh.decel * max(0.0, d))))
            v[i] = max(0.0, min(v[i] + veh.accel * dt, veh.v_max, v_stop))
            s_new = min(s[i] + v[i] * dt, length[i])
            # 通過し終えた区間を解放（車体後端が膨張区間を抜けたら）
            for z in zones_of(i):
                if z.owner == i and s_new > z.exit[i] + hl - 1e-6:
                    z.owner = None
                    events.append({"t": round(t, 2), "vehicle": i, "type": "exit", "zone": z.routes})
            moved_total += s_new - s[i]
            if blocked and v[i] < 0.05:
                active_blocked += 1
                events.append({"t": round(t, 2), "vehicle": i, "type": "wait", "zone": None})
            s[i] = s_new
            if s[i] >= length[i] - 1e-6:
                done[i] = True
                for z in zones_of(i):  # 完了時は保持区間を解放
                    if z.owner == i:
                        z.owner = None
                events.append({"t": round(t, 2), "vehicle": i, "type": "done", "zone": None})
        record(t)
        if all(done):  # 全車到達で終了（未発進車がいる間は終了しない）
            break
        # デッドロック検出: 動きがほぼ無く、全アクティブ車が待ち（占有区間で循環待ち）
        if n_active > 0 and moved_total < 1e-4 and active_blocked >= 1 and active_blocked == n_active:
            stall_steps += 1
        else:
            stall_steps = 0
        if stall_steps >= int(2.0 / dt):  # 2秒間まったく進まない＝膠着
            deadlock = True
            deadlock_time = round(t, 2)
            break

    # 安全検証: 全時刻・全ペアの中心間最接近距離 ＋ **車体（向き付き矩形）の重なり判定**。
    # 旧: 中心間距離 < 半幅和 のみ＝車長を無視し縦方向の車体重なりを見逃していた。
    min_sep = float("inf")
    min_sep_t: float | None = None
    min_sep_pair: tuple[int, int] | None = None
    collision = False
    nf = len(traces[0]) if traces and traces[0] else 0
    for k in range(nf):
        for i in range(nv):
            fi = traces[i][k]
            # 走行中(出発済かつ未到達)の車両どうしのみ評価。到達済(=配送/退場)や未発進は除外
            # （積込/排土点を共有する経路で端点が重なる誤検出を避ける）。
            if fi["t"] < vehicles[i].start_time or fi["state"] == "done":
                continue
            for j in range(i + 1, nv):
                fj = traces[j][k]
                if fj["t"] < vehicles[j].start_time or fj["state"] == "done":
                    continue
                d = float(np.hypot(fi["x"] - fj["x"], fi["y"] - fj["y"]))
                if d < min_sep:
                    min_sep = d
                    min_sep_t = fi["t"]
                    min_sep_pair = (i, j)
                if not collision and _bodies_overlap(fi, fj, vehicles[i], vehicles[j]):
                    collision = True

    # 待機時間: 「相手占有区間に阻まれて停止」した wait イベント数×dt（起動時の静止や末端減速は含めない）。
    wait_steps = [0] * nv
    for ev in events:
        if ev["type"] == "wait":
            wait_steps[ev["vehicle"]] += 1
    wait_time = [round(w * dt, 2) for w in wait_steps]
    # 所要時間: 出発(start_time)から到達(done フレーム)まで。未到達は makespan まで。
    finish = {ev["vehicle"]: ev["t"] for ev in events if ev["type"] == "done"}
    travel_time = [round(float(finish.get(i, t)) - vehicles[i].start_time, 2) for i in range(nv)]

    res = SimResult(traces=traces, events=events, deadlock=deadlock, deadlock_time=deadlock_time,
                    makespan=round(t, 2),
                    min_separation_m=(round(min_sep, 2) if min_sep != float("inf") else None),
                    min_sep_time=min_sep_t, min_sep_pair=min_sep_pair, collision=collision,
                    wait_time_s=wait_time, total_wait_s=round(sum(wait_time), 2),
                    travel_time_s=travel_time)
    res.status = "DEADLOCK" if deadlock else ("COLLISION" if collision else ("TIMEOUT" if not all(done) else "OK"))
    return res


def _yielder(vehicles, i, j):
    """ペア(i,j)で譲る側＝低優先(priority 大、同値なら index 大)を返す。"""
    pi, pj = vehicles[i].priority, vehicles[j].priority
    if pi != pj:
        return i if pi > pj else j
    return max(i, j)


def simulate_fleet_auto(vehicles: list[SimVehicle], *, dt: float = 0.2, gap_m: float = 2.0,
                        max_time: float = 600.0, cell: float = 0.5, clearance_m: float = 0.0,
                        max_bays: int = 6) -> SimResult:
    """`simulate_fleet` を実行し、衝突(接触)が出たら**低優先側へ自動で待避所(横退避)**を入れて再試行する。

    衝突した最接近ペアの共有競合区間の中点に、譲る側(低優先)の経路を横退避させる待避所を配置→再シム。
    解消するか max_bays 回まで繰り返す。配置した待避所は SimResult.auto_bays に記録する。
    """
    import numpy as np

    from .conflict import detect_conflicts
    from .passing import lateral_detour

    work = [SimVehicle(points=np.asarray(v.points, float).copy(), v_max=v.v_max, accel=v.accel,
                       decel=v.decel, reserve_decel=v.reserve_decel,
                       half_width=v.half_width, half_length=v.half_length,
                       priority=v.priority, start_time=v.start_time, name=v.name) for v in vehicles]
    auto_bays: list[dict] = []
    tried: set = set()  # (vehicle, side) 既試行
    res = simulate_fleet(work, dt=dt, gap_m=gap_m, max_time=max_time, cell=cell, clearance_m=clearance_m)
    for _ in range(max_bays):
        # 接触(collision) または 対向膠着(deadlock=正面で停止)を、待避所で解消する。
        if not (res.collision or res.deadlock) or res.min_sep_pair is None:
            break
        i, j = res.min_sep_pair
        y = _yielder(work, i, j)
        other = j if y == i else i
        if (y, 1) in tried and (y, -1) in tried:
            break  # 譲る側の両サイドを試して改善なし → 反復を打ち切る（非収束防止）
        # ペア(y,other)の共有競合区間を特定し、譲る側 y の経路上の中点へ待避所を置く
        routes = [{"points": work[k].points, "half_width": work[k].half_width} for k in range(len(work))]
        cs = [c for c in detect_conflicts(routes, cell=cell, clearance_m=clearance_m)
              if {c.a, c.b} == {y, other}]
        if not cs:
            break
        c = max(cs, key=lambda c: c.overlap_area_m2)
        ivs = c.a_intervals if c.a == y else c.b_intervals
        if not ivs:
            break
        s_center = 0.5 * (ivs[0].s_start + ivs[0].s_end)
        side = 1 if (y, 1) not in tried else -1
        tried.add((y, side))
        offset = work[y].half_width + work[other].half_width + gap_m + 2.0
        hold = 2.0 * max(work[y].half_length, work[other].half_length) + gap_m + 4.0
        prev_pts = work[y].points
        prev_sep = res.min_separation_m
        work[y].points = lateral_detour(work[y].points, s_center=s_center, offset=offset,
                                        side=side, ramp=max(6.0, hold * 0.5), hold=hold)
        trial = simulate_fleet(work, dt=dt, gap_m=gap_m, max_time=max_time, cell=cell, clearance_m=clearance_m)
        # 採用条件: 解消した、または min_separation が改善した場合のみ。
        # 悪化する待避所は巻き戻す（悪化を積み重ねて max_bays まで暴走するのを防ぐ）。
        resolved = not (trial.collision or trial.deadlock)
        improved = resolved or (
            trial.min_separation_m is not None
            and (prev_sep is None or trial.min_separation_m > prev_sep + 1e-6)
        )
        if improved:
            auto_bays.append({"vehicle": y, "s_center": round(s_center, 2), "offset": round(offset, 2), "side": side})
            res = trial
        else:
            work[y].points = prev_pts  # 巻き戻し。res は据え置き＝次周で同ペアの逆サイドを試す
    res.auto_bays = auto_bays
    return res


def simulate_fleet_sequential(vehicles: list[SimVehicle], *, loops: int = 1, dt: float = 0.2,
                              max_time: float = 600.0) -> SimResult:
    """**逐次ローテーション**ディスパッチ: 1台ずつ順に走行し、Goal到達で次の車をStart。全車終わると
    最初へ戻り、各車 `loops` 周まで繰り返す。基本同時に走るのは1台（積込/排土点を1台ずつ使うイメージ）。

    各車は出発時に s=0 へ戻って自経路を走り、規定減速でGoalに停止。アイドル中は現在位置で待機。
    返値 SimResult（全車・全時刻の trace、車両ごと所要時間、makespan）。
    """
    nv = len(vehicles)
    if nv == 0:
        return SimResult(status="NO_VEHICLES")
    loops = max(1, int(loops))
    pts = [np.asarray(v.points, float) for v in vehicles]
    s_arr = [_cum_s(p) for p in pts]
    length = [float(s_arr[i][-1]) if len(s_arr[i]) else 0.0 for i in range(nv)]
    s = [0.0] * nv
    v = [0.0] * nv
    laps = [0] * nv          # 各車の完了周回数
    active_time = [0.0] * nv  # 各車の走行時間合計[s]
    traces: list[list[dict]] = [[] for _ in range(nv)]
    events: list[dict] = []
    t = 0.0

    def record(active: int):
        for k in range(nv):
            x, y, hd = _pose_at(pts[k], s_arr[k], s[k])
            st = "run" if k == active else ("done" if laps[k] >= loops else "wait")
            traces[k].append({"t": round(t, 2), "s": round(s[k], 3), "x": x, "y": y,
                              "heading_deg": round(hd, 1), "v": round(v[k], 3), "state": st})

    record(-1)
    order = list(range(nv))                       # 発進順＝入力（ライブラリ）順
    schedule = [i for _ in range(loops) for i in order]
    n_steps = int(max_time / dt)
    steps = 0
    for i in schedule:
        if length[i] <= 1e-9:
            laps[i] += 1
            continue
        s[i] = 0.0  # この周回の起点へ（前回Goalからはリセット）
        v[i] = 0.0
        events.append({"t": round(t, 2), "vehicle": i, "type": "dispatch", "zone": None})
        while s[i] < length[i] - 1e-6 and steps < n_steps:
            t += dt
            steps += 1
            d = length[i] - s[i]
            v_stop = float(np.sqrt(max(0.0, 2.0 * vehicles[i].decel * max(0.0, d))))
            v[i] = max(0.0, min(v[i] + vehicles[i].accel * dt, vehicles[i].v_max, v_stop))
            s[i] = min(s[i] + v[i] * dt, length[i])
            active_time[i] += dt
            record(i)
        v[i] = 0.0
        laps[i] += 1
        events.append({"t": round(t, 2), "vehicle": i,
                       "type": "done" if laps[i] >= loops else "lap", "zone": None})
        if steps >= n_steps:
            break

    res = SimResult(traces=traces, events=events, makespan=round(t, 2),
                    travel_time_s=[round(active_time[i], 2) for i in range(nv)],
                    wait_time_s=[0.0] * nv, total_wait_s=0.0,
                    min_separation_m=None, collision=False, deadlock=False)
    res.status = "OK" if all(laps[i] >= loops for i in range(nv)) else "TIMEOUT"
    return res
