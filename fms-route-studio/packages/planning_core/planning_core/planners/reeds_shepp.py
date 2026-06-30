"""Reeds-Shepp 曲線（前進＋後進の最小旋回半径つき最短曲線, 設計 §9.4）。

2姿勢を「R_min を守る前進/後進の円弧＋直線」で接続する。Dubins と違い後進を許すため、
曲がりながらの後進（角度のついた切り返し）を生成できる。hybrid A* の解析接続やスポッティングの
全体接続候補に使うと、「目標正面で直進バック」になりがちな切り返しを、角度付き cusp に多様化できる。

実装は **Reeds & Shepp の完全な48経路集合**（CSC / CCC / CCCC / CCSC / CCSCC）を OMPL の
ReedsSheppStateSpace と同じ閉形式式で列挙する。狭隘地で効く cusp 付き語（CCC/CCSC/CCSCC）まで
含むため、急な切り返しや差し込みが必要なスポッティングで真価を発揮する。**各候補は実際にサンプル
して終点一致を検証してから採用**するため、式の不備があっても誤った経路を返さない（候補が減るだけ）。

`reeds_shepp_paths` は検証済み候補を弧長の短い順に**複数**返す。最短が衝突しても次善を試せるよう、
スポッティング/解析接続側で全候補を衝突・コスト評価できる。
"""
from __future__ import annotations

import math

ZERO = 1e-9
PI = math.pi
HALFPI = 0.5 * math.pi


def _polar(x: float, y: float) -> tuple[float, float]:
    return math.hypot(x, y), math.atan2(y, x)


def _mod2pi(t: float) -> float:
    """角度を (-pi, pi] へ。"""
    v = math.fmod(t, 2.0 * PI)
    if v < -PI:
        v += 2.0 * PI
    elif v > PI:
        v -= 2.0 * PI
    return v


# 後方互換エイリアス（旧名）。
_M = _mod2pi


def _tau_omega(u, v, xi, eta, phi):
    delta = _mod2pi(u - v)
    A = math.sin(u) - math.sin(delta)
    B = math.cos(u) - math.cos(delta) - 1.0
    t1 = math.atan2(eta * A - xi * B, xi * A + eta * B)
    t2 = 2.0 * (math.cos(delta) - math.cos(v) - math.cos(u)) + 3.0
    tau = _mod2pi(t1 + PI) if t2 < 0 else _mod2pi(t1)
    omega = _mod2pi(tau - u + v - phi)
    return tau, omega


# --- 基本語の閉形式式（OMPL ReedsSheppStateSpace と同一。戻り値 (ok, t, u, v)）---

def _LpSpLp(x, y, phi):  # formula 8.1 (CSC, 同方向)
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if t >= -ZERO:
        v = _mod2pi(phi - t)
        if v >= -ZERO:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpSpRp(x, y, phi):  # formula 8.2 (CSC, 逆方向)
    u1, t1 = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u1sq = u1 * u1
    if u1sq >= 4.0:
        u = math.sqrt(u1sq - 4.0)
        theta = math.atan2(2.0, u)
        t = _mod2pi(t1 + theta)
        v = _mod2pi(t - phi)
        if t >= -ZERO and v >= -ZERO:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpRmL(x, y, phi):  # formula 8.3/8.4 (CCC)
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    u1, theta = _polar(xi, eta)
    if u1 <= 4.0:
        u = -2.0 * math.asin(0.25 * u1)
        t = _mod2pi(theta + 0.5 * u + PI)
        v = _mod2pi(phi - t + u)
        if t >= -ZERO and u <= ZERO:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpRupLumRm(x, y, phi):  # formula 8.7 (CCCC)
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho = 0.25 * (2.0 + math.hypot(xi, eta))
    if rho <= 1.0:
        u = math.acos(rho)
        t, v = _tau_omega(u, -u, xi, eta, phi)
        if t >= -ZERO and v <= ZERO:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpRumLumRp(x, y, phi):  # formula 8.8 (CCCC)
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho = (20.0 - xi * xi - eta * eta) / 16.0
    if 0.0 <= rho <= 1.0:
        u = -math.acos(rho)
        if u >= -HALFPI:
            t, v = _tau_omega(u, u, xi, eta, phi)
            if t >= -ZERO and v >= -ZERO:
                return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpRmSmLm(x, y, phi):  # formula 8.9 (CCSC) — 中央Cは固定 -pi/2
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho >= 2.0:
        r = math.sqrt(rho * rho - 4.0)
        u = 2.0 - r
        t = _mod2pi(theta + math.atan2(r, -2.0))
        v = _mod2pi(phi - HALFPI - t)
        if t >= -ZERO and u <= ZERO and v <= ZERO:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpRmSmRm(x, y, phi):  # formula 8.9 変種 (CCSC)
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho, theta = _polar(-eta, xi)
    if rho >= 2.0:
        t = theta
        u = 2.0 - rho
        v = _mod2pi(t + HALFPI - phi)
        if t >= -ZERO and u <= ZERO and v <= ZERO:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpRmSLmRp(x, y, phi):  # formula 8.11 (CCSCC) — 両端Cの内側Cは固定 -pi/2
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho, _theta = _polar(xi, eta)
    if rho >= 2.0:
        u = 4.0 - math.sqrt(rho * rho - 4.0)
        if u <= ZERO:
            t = _mod2pi(math.atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta))
            v = _mod2pi(t - phi)
            if t >= -ZERO and v >= -ZERO:
                return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _reflect(types):
    return ["R" if c == "L" else "L" if c == "R" else "S" for c in types]


def _emit4(out, base_types, build, x, y, phi, fn):
    """基本語 fn の4対称変換（恒等/時間反転/反射/両方）を列挙して out へ追加。

    build(t,u,v) は基本語のセグメント長タプル（CCSC/CCSCC の固定 -pi/2 もここで挿入）。
    時間反転=長さ符号反転（後進化）、反射=L<->R 入替。
    """
    rt = _reflect(base_types)
    ok, t, u, v = fn(x, y, phi)
    if ok:
        out.append(list(zip(build(t, u, v), base_types)))
    ok, t, u, v = fn(-x, y, -phi)
    if ok:
        out.append(list(zip(tuple(-a for a in build(t, u, v)), base_types)))
    ok, t, u, v = fn(x, -y, -phi)
    if ok:
        out.append(list(zip(build(t, u, v), rt)))
    ok, t, u, v = fn(-x, -y, phi)
    if ok:
        out.append(list(zip(tuple(-a for a in build(t, u, v)), rt)))


def _emit4_rev(out, base_types, build, x, y, phi, fn):
    """CCSC の「逆順」変種（C-S-C-C → C-C-S-C）を列挙。入力を経路反転座標へ移し、
    セグメント順を反転する（reverse(['L','R','S','L'])=['L','S','R','L'] が逆順語）。"""
    xb = x * math.cos(phi) + y * math.sin(phi)
    yb = x * math.sin(phi) - y * math.cos(phi)
    rev_types = list(reversed(base_types))
    rrev = _reflect(rev_types)
    ok, t, u, v = fn(xb, yb, phi)
    if ok:
        out.append(list(zip(tuple(reversed(build(t, u, v))), rev_types)))
    ok, t, u, v = fn(-xb, yb, -phi)
    if ok:
        out.append(list(zip(tuple(-a for a in reversed(build(t, u, v))), rev_types)))
    ok, t, u, v = fn(xb, -yb, -phi)
    if ok:
        out.append(list(zip(tuple(reversed(build(t, u, v))), rrev)))
    ok, t, u, v = fn(-xb, -yb, phi)
    if ok:
        out.append(list(zip(tuple(-a for a in reversed(build(t, u, v))), rrev)))


def _word_candidates(x, y, phi):
    """正規化座標(x,y,phi)に対する全RS語（48経路）の segs=[(signed_len, type), ...] を列挙。"""
    out: list = []
    _b3 = lambda t, u, v: (t, u, v)  # noqa: E731
    # CSC
    _emit4(out, ["L", "S", "L"], _b3, x, y, phi, _LpSpLp)
    _emit4(out, ["L", "S", "R"], _b3, x, y, phi, _LpSpRp)
    # CCC
    _emit4(out, ["L", "R", "L"], _b3, x, y, phi, _LpRmL)
    # CCCC（中央2弧は ±u）
    _emit4(out, ["L", "R", "L", "R"], lambda t, u, v: (t, u, -u, v), x, y, phi, _LpRupLumRm)
    _emit4(out, ["L", "R", "L", "R"], lambda t, u, v: (t, u, u, v), x, y, phi, _LpRumLumRp)
    # CCSC（中央Cは固定 -pi/2）＋その逆順
    _csc_build = lambda t, u, v: (t, -HALFPI, u, v)  # noqa: E731
    _emit4(out, ["L", "R", "S", "L"], _csc_build, x, y, phi, _LpRmSmLm)
    _emit4_rev(out, ["L", "R", "S", "L"], _csc_build, x, y, phi, _LpRmSmLm)
    _emit4(out, ["L", "R", "S", "R"], _csc_build, x, y, phi, _LpRmSmRm)
    _emit4_rev(out, ["L", "R", "S", "R"], _csc_build, x, y, phi, _LpRmSmRm)
    # CCSCC（内側2弧は固定 -pi/2）
    _emit4(out, ["L", "R", "S", "L", "R"], lambda t, u, v: (t, -HALFPI, u, -HALFPI, v), x, y, phi, _LpRmSLmRp)
    return out


def _step(x, y, yaw, curv, direction, ds):
    sds = direction * ds
    if abs(curv) < 1e-9:
        return x + sds * math.cos(yaw), y + sds * math.sin(yaw), yaw
    r = 1.0 / curv
    nyaw = (yaw + sds * curv + math.pi) % (2.0 * math.pi) - math.pi
    cx = x - r * math.sin(yaw)
    cy = y + r * math.cos(yaw)
    return cx + r * math.sin(nyaw), cy - r * math.cos(nyaw), nyaw


def _sample(start, segs, rho, step):
    x, y, yaw = start
    pts = [(x, y, yaw, "F")]
    for length, ctype in segs:
        if abs(length) < 1e-9:
            continue
        gear = 1 if length >= 0 else -1
        arc = abs(length) * rho          # 'S'=距離, 'L'/'R'=角度×rho=弧長
        curv = 0.0 if ctype == "S" else (1.0 / rho if ctype == "L" else -1.0 / rho)
        n = max(1, int(arc / step))
        ds = arc / n
        for _ in range(n):
            x, y, yaw = _step(x, y, yaw, curv, gear, ds)
            pts.append((x, y, yaw, "F" if gear > 0 else "R"))
    return pts


def reeds_shepp_paths(start, goal, rho: float, step: float = 0.3,
                      *, pos_tol: float = 0.05, yaw_tol_deg: float = 1.5):
    """start/goal=(x,y,yaw[rad])。終点一致を検証した候補を弧長の短い順に返す（list of points list）。

    pos_tol[m] / yaw_tol_deg[deg]: 終点許容。寄り付き精度のため既定は厳しめ（5cm / 1.5°）。
    許容内の候補のみ採用し、サンプリング離散化による微小残差だけを終点スナップで吸収する
    （0.5m級の粗いスナップはしない＝最終点に折れを作らない）。
    """
    sx, sy, syaw = start
    gx, gy, gyaw = goal
    c, s = math.cos(syaw), math.sin(syaw)
    dx, dy = gx - sx, gy - sy
    x = (dx * c + dy * s) / rho
    y = (-dx * s + dy * c) / rho
    phi = _mod2pi(gyaw - syaw)

    cands = _word_candidates(x, y, phi)
    cands.sort(key=lambda segs: sum(abs(l) for l, _ in segs))
    yaw_tol = math.radians(yaw_tol_deg)
    valid = []
    for segs in cands:
        pts = _sample(start, segs, rho, step)
        ex, ey, eyaw = pts[-1][0], pts[-1][1], pts[-1][2]
        if math.hypot(ex - gx, ey - gy) <= pos_tol and abs(_mod2pi(eyaw - gyaw)) <= yaw_tol:
            pts[-1] = (gx, gy, gyaw, pts[-1][3])  # 微小残差を終点スナップで吸収
            valid.append(pts)
    return valid


def sample_reeds_shepp(start, goal, rho: float, step: float = 0.3,
                       *, pos_tol: float = 0.05, yaw_tol_deg: float = 1.5):
    """最短の Reeds-Shepp 経路を (x,y,yaw,gear) 点列で返す。無ければ None。"""
    v = reeds_shepp_paths(start, goal, rho, step, pos_tol=pos_tol, yaw_tol_deg=yaw_tol_deg)
    return v[0] if v else None
