import { useMemo } from "react";

import { ProfileChart } from "@/components/ProfileChart";
import type { ProfileBand } from "@/components/ProfileChart";
import { useStore } from "@/store/useStore";
import type { TrajPoint, Violation } from "@/types/api";

// violation.kind → どのチャートに帯を出すか
function bandsFor(kinds: string[], violations: Violation[]): ProfileBand[] {
  return violations
    .filter((v) => kinds.includes(v.kind))
    .map((v) => ({ s0: v.s_start, s1: v.s_end }));
}
function limitFor(kinds: string[], violations: Violation[]): number | null {
  const v = violations.find((x) => kinds.includes(x.kind));
  return v ? v.limit : null;
}

export function AnalysisPanel() {
  const route = useStore((s) => s.route);
  const spotResult = useStore((s) => s.spotResult);
  const activeFeature = useStore((s) => s.activeFeature);
  const hoverIndex = useStore((s) => s.hoverPointIndex);
  const setHoverPoint = useStore((s) => s.setHoverPoint);

  // 寄り付き工程で寄り付き解析があればそれを、無ければ経路解析を表示。
  const spotView =
    activeFeature === "spotting" && spotResult?.analysis && spotResult?.trajectory
      ? { trajectory: spotResult.trajectory, analysis: spotResult.analysis, safety: spotResult.safety }
      : null;
  const view = spotView ?? route;
  const sourceLabel = spotView ? "寄り付き" : "経路";

  const series = useMemo(() => {
    if (!view) return null;
    const pts: TrajPoint[] = view.trajectory.points;
    const xs = pts.map((p) => p.s);
    return {
      xs,
      kappa: pts.map((p) => Math.abs(p.curvature)),
      dk: pts.map((p) => p.curvature_rate),
      steer: pts.map((p) => (p.steer_deg == null ? NaN : p.steer_deg)),
      grade: pts.map((p) => (p.grade_pct == null ? NaN : p.grade_pct)),
      speed: pts.map((p) => (p.speed_mps == null ? NaN : p.speed_mps)),
      elev: pts.map((p) => (p.z == null ? NaN : p.z)),
      hasSteer: pts.some((p) => p.steer_deg != null),
      hasGrade: pts.some((p) => p.grade_pct != null),
      hasSpeed: pts.some((p) => p.speed_mps != null),
      hasElev: pts.some((p) => p.z != null),
    };
  }, [view]);

  if (!view || !series) {
    return (
      <section className="card">
        <h3>解析</h3>
        <p className="hint">経路を生成、または寄り付きをシミュレーションすると、曲率 / dκ/ds / 速度 / 可否を表示します。</p>
      </section>
    );
  }

  const { trajectory: t, analysis: a } = view;
  const v = a.violations ?? [];
  // 曲率の制限線: violation(min_radius) の limit(=κ上限) を優先、無ければ実測 min radius を参考線に。
  const kappaLimit = limitFor(["min_radius"], v) ?? (t.min_radius_m ? 1 / t.min_radius_m : null);

  return (
    <section className="card">
      <h3>解析 · {sourceLabel}</h3>
      <ul className="metrics">
        <li>
          <span>総延長</span>
          <b>{t.length_m.toFixed(1)} m</b>
        </li>
        <li>
          <span>最小旋回半径</span>
          <b>{t.min_radius_m === null ? "∞" : `${t.min_radius_m.toFixed(2)} m`}</b>
        </li>
        <li>
          <span>最大曲率 |κ|</span>
          <b>{a.max_curvature.toFixed(4)} 1/m</b>
        </li>
        <li>
          <span>最大 dκ/ds</span>
          <b>{a.max_curvature_rate.toExponential(2)} 1/m²</b>
        </li>
        <li>
          <span>曲率ソース</span>
          <b>{t.curvature_source === "analytic" ? "解析解" : "数値"}</b>
        </li>
        {a.max_speed_mps != null && (
          <li>
            <span>最高速度</span>
            <b>{a.max_speed_mps.toFixed(1)} m/s</b>
          </li>
        )}
        {a.time_total_s != null && (
          <li>
            <span>所要時間</span>
            <b>{a.time_total_s.toFixed(1)} s</b>
          </li>
        )}
        {a.stopping_distance_m != null && (
          <li>
            <span>停止可能距離</span>
            <b>{a.stopping_distance_m.toFixed(1)} m</b>
          </li>
        )}
        <li>
          <span>実現可能性</span>
          <b className={a.feasible ? "ok" : "bad"}>{a.feasible ? "OK" : "NG"}</b>
        </li>
      </ul>
      {a.not_applicable.length > 0 && <p className="hint">N/A: {a.not_applicable.join(", ")}</p>}
      {v.length > 0 && (
        <p className="hint" style={{ color: "#fca5a5" }}>
          violations: {v.map((x) => `${x.kind}@${x.s_start.toFixed(0)}–${x.s_end.toFixed(0)}m`).join(", ")}
        </p>
      )}

      {view.safety && (
        <div className="safety">
          <div className="safety-head">
            安全検証（配信可否）:
            <b className={view.safety.passed ? "ok" : "bad"}>{view.safety.passed ? " OK" : " NG"}</b>
          </div>
          <ul className="safety-checks">
            {view.safety.checks.map((c) => (
              <li key={c.name} className={!c.applicable ? "na" : c.ok ? "ok" : "bad"}>
                {!c.applicable ? "—" : c.ok ? "✓" : "✗"} {c.label}
                {c.applicable && c.measured != null && c.limit != null && (
                  <span className="sval"> ({c.measured.toFixed(2)} / 限界 {c.limit.toFixed(2)})</span>
                )}
              </li>
            ))}
          </ul>
          {view.safety.reasons.length > 0 && (
            <p className="hint" style={{ color: "#fca5a5" }}>不可理由: {view.safety.reasons.join(" / ")}</p>
          )}
        </div>
      )}

      <div className="charts">
        <ProfileChart
          title="曲率 |κ|"
          unit="1/m"
          color="#38bdf8"
          xs={series.xs}
          ys={series.kappa}
          limit={kappaLimit}
          limitLabel={t.min_radius_m ? `min R=${t.min_radius_m.toFixed(1)}m` : undefined}
          bands={bandsFor(["min_radius"], v)}
          hoverIndex={hoverIndex}
          onHover={setHoverPoint}
        />
        <ProfileChart
          title="曲率変化率 dκ/ds"
          unit="1/m²"
          color="#f59e0b"
          xs={series.xs}
          ys={series.dk}
          limit={limitFor(["kappa_rate"], v)}
          bands={bandsFor(["kappa_rate"], v)}
          fmt={(x) => (Math.abs(x) < 1e-3 && x !== 0 ? x.toExponential(1) : x.toFixed(4))}
          hoverIndex={hoverIndex}
          onHover={setHoverPoint}
        />
        {series.hasSteer && (
          <ProfileChart
            title="操舵角 steer"
            unit="deg"
            color="#a78bfa"
            xs={series.xs}
            ys={series.steer}
            limit={limitFor(["steer_rate"], v)}
            bands={bandsFor(["steer_rate"], v)}
            hoverIndex={hoverIndex}
            onHover={setHoverPoint}
          />
        )}
        {series.hasGrade && (
          <ProfileChart
            title="勾配 grade"
            unit="%"
            color="#34d399"
            xs={series.xs}
            ys={series.grade}
            limit={limitFor(["grade"], v)}
            bands={bandsFor(["grade"], v)}
            hoverIndex={hoverIndex}
            onHover={setHoverPoint}
          />
        )}
        {series.hasElev && (
          <ProfileChart
            title="標高 z"
            unit="m"
            color="#b45309"
            xs={series.xs}
            ys={series.elev}
            fmt={(x) => x.toFixed(1)}
            hoverIndex={hoverIndex}
            onHover={setHoverPoint}
          />
        )}
        {series.hasSpeed && (
          <ProfileChart
            title="速度 v"
            unit="m/s"
            color="#7c3aed"
            xs={series.xs}
            ys={series.speed}
            hoverIndex={hoverIndex}
            onHover={setHoverPoint}
          />
        )}
      </div>
      <p className="hint" style={{ marginTop: 4 }}>
        グラフ上をホバーで s 位置の値を表示＋地図上の経路点をハイライト。赤帯= violation / 赤破線= 制限。
      </p>
    </section>
  );
}
