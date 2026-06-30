import { useEffect, useState } from "react";

import { api } from "@/api/client";
import { useStore } from "@/store/useStore";

// 編集フィールド定義（適用される kinematic_type を types に列挙）。
type Kin = "rigid_bicycle" | "articulated" | "tracked_skid";
interface FieldDef {
  key: string;
  label: string;
  unit?: string;
  types: Kin[];
  step?: number;
}
const ALL: Kin[] = ["rigid_bicycle", "articulated", "tracked_skid"];
const FIELDS: { group: string; items: FieldDef[] }[] = [
  {
    group: "寸法・フットプリント",
    items: [
      { key: "overall_length", label: "全長", unit: "m", types: ALL, step: 0.1 },
      { key: "overall_width", label: "全幅", unit: "m", types: ALL, step: 0.1 },
      { key: "overall_height", label: "全高", unit: "m", types: ALL, step: 0.1 },
      { key: "footprint_radius", label: "フットプリント半径", unit: "m", types: ALL, step: 0.1 },
      { key: "road_width", label: "必要道幅", unit: "m", types: ALL, step: 0.1 },
      { key: "track_width", label: "履帯中心間", unit: "m", types: ["tracked_skid"], step: 0.01 },
    ],
  },
  {
    group: "運動学",
    items: [
      { key: "min_turning_radius", label: "最小旋回半径 R_min", unit: "m", types: ["rigid_bicycle", "articulated"], step: 0.1 },
      { key: "wheel_base", label: "ホイールベース", unit: "m", types: ["rigid_bicycle", "articulated"], step: 0.01 },
      { key: "max_steer_angle", label: "最大操舵角", unit: "rad", types: ["rigid_bicycle", "articulated"], step: 0.01 },
      { key: "max_steer_rate", label: "最大操舵レート", unit: "rad/s", types: ["rigid_bicycle", "articulated"], step: 0.01 },
      { key: "max_articulation_angle", label: "最大中折れ角", unit: "rad", types: ["articulated"], step: 0.01 },
      { key: "kappa_max_fwd", label: "前進 κ上限", unit: "1/m", types: ["rigid_bicycle", "articulated"], step: 0.001 },
      { key: "kappa_max_rev", label: "後進 κ上限", unit: "1/m", types: ["rigid_bicycle", "articulated"], step: 0.001 },
      { key: "kappa_rate_max", label: "κ変化率上限 dκ/ds", unit: "1/m²", types: ALL, step: 0.01 },
    ],
  },
  {
    group: "速度・加速度（速度プロファイル）",
    items: [
      { key: "max_speed_fwd", label: "前進最大速度", unit: "m/s", types: ALL, step: 0.1 },
      { key: "max_speed_rev", label: "後進最大速度", unit: "m/s", types: ALL, step: 0.1 },
      { key: "max_lateral_accel", label: "横加速度上限", unit: "m/s²", types: ALL, step: 0.1 },
      { key: "max_accel", label: "加速上限", unit: "m/s²", types: ALL, step: 0.1 },
      { key: "max_decel", label: "減速上限", unit: "m/s²", types: ALL, step: 0.1 },
    ],
  },
  {
    group: "安全しきい値",
    items: [
      { key: "max_grade_pct", label: "許容縦断勾配", unit: "%", types: ALL, step: 1 },
      { key: "min_clearance_m", label: "要求最小離隔", unit: "m", types: ALL, step: 0.1 },
    ],
  },
];

function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

export function VehiclePanel() {
  const vehicleId = useStore((s) => s.vehicleId);
  const setVehicleId = useStore((s) => s.setVehicleId);
  const bumpVehiclesRev = useStore((s) => s.bumpVehiclesRev);
  const setStatus = useStore((s) => s.setStatus);

  const [ids, setIds] = useState<{ id: string; spec_status: string; overridden?: boolean }[]>([]);
  const [kin, setKin] = useState<Kin>("rigid_bicycle");
  const [def, setDef] = useState<Record<string, unknown>>({});
  const [form, setForm] = useState<Record<string, string>>({});
  const [overridden, setOverridden] = useState(false);
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    api.listVehicles().then((vs) => setIds(vs.map((v) => ({ id: v.id, spec_status: v.spec_status, overridden: v.overridden }))))
      .catch(() => undefined);
  }, []);

  const vid = vehicleId ?? "HD785";

  useEffect(() => {
    let alive = true;
    api.vehicleDetail(vid).then((d) => {
      if (!alive) return;
      setDef(d.default);
      setKin(d.effective.kinematic_type as Kin);
      setOverridden(Object.keys(d.override).length > 0);
      // フォームは有効値で初期化
      const f: Record<string, string> = {};
      for (const g of FIELDS) for (const it of g.items) {
        const v = num(d.effective[it.key]);
        if (v != null) f[it.key] = String(v);
      }
      setForm(f);
    }).catch(() => undefined);
    return () => {
      alive = false;
    };
  }, [vid]);

  async function save() {
    setBusy(true);
    try {
      // 既定と異なる値だけをオーバーライドとして送る（最小差分）。
      const fields: Record<string, number> = {};
      for (const g of FIELDS) for (const it of g.items) {
        if (!it.types.includes(kin)) continue;
        const raw = form[it.key];
        if (raw == null || raw === "") continue;
        const v = Number(raw);
        if (!Number.isFinite(v)) continue;
        const dv = num(def[it.key]);
        if (dv == null || Math.abs(v - dv) > 1e-9) fields[it.key] = v;
      }
      const res = await api.saveVehicleOverride(vid, fields);
      setOverridden(Object.keys(res.override).length > 0);
      bumpVehiclesRev();
      const lst = await api.listVehicles();
      setIds(lst.map((v) => ({ id: v.id, spec_status: v.spec_status, overridden: v.overridden })));
      setStatus(`車両 ${vid} のパラメータを保存しました（${Object.keys(fields).length}項目）`);
    } catch (e) {
      setStatus(`保存に失敗: ${String(e)}`);
    } finally {
      setBusy(false);
    }
  }

  async function reset() {
    setBusy(true);
    try {
      await api.resetVehicleOverride(vid);
      const d = await api.vehicleDetail(vid);
      setOverridden(false);
      const f: Record<string, string> = {};
      for (const g of FIELDS) for (const it of g.items) {
        const v = num(d.effective[it.key]);
        if (v != null) f[it.key] = String(v);
      }
      setForm(f);
      bumpVehiclesRev();
      const lst = await api.listVehicles();
      setIds(lst.map((v) => ({ id: v.id, spec_status: v.spec_status, overridden: v.overridden })));
      setStatus(`車両 ${vid} を既定値に戻しました`);
    } catch (e) {
      setStatus(`リセットに失敗: ${String(e)}`);
    } finally {
      setBusy(false);
    }
  }

  return (
    <section className="card">
      <h3>車両パラメータ</h3>
      <label>
        車種
        <select value={vid} onChange={(e) => setVehicleId(e.target.value)}>
          {ids.map((v) => (
            <option key={v.id} value={v.id}>
              {v.id} {v.spec_status === "estimated" ? "≈" : ""} {v.overridden ? "●調整済" : ""}
            </option>
          ))}
        </select>
      </label>

      {FIELDS.map((g) => {
        const items = g.items.filter((it) => it.types.includes(kin));
        if (items.length === 0) return null;
        return (
          <div key={g.group} style={{ marginTop: 6 }}>
            <div className="hint" style={{ margin: "2px 0" }}>{g.group}</div>
            <div className="grid2">
              {items.map((it) => {
                const dv = num(def[it.key]);
                const cur = form[it.key];
                const changed = dv != null && cur != null && cur !== "" && Math.abs(Number(cur) - dv) > 1e-9;
                return (
                  <label key={it.key} title={dv != null ? `既定 ${dv}` : ""}>
                    <span style={{ color: changed ? "#fde047" : undefined }}>
                      {it.label}{it.unit ? `（${it.unit}）` : ""}{changed ? " *" : ""}
                    </span>
                    <input
                      type="number"
                      step={it.step ?? 0.1}
                      value={cur ?? ""}
                      onChange={(e) => setForm((f) => ({ ...f, [it.key]: e.target.value }))}
                    />
                  </label>
                );
              })}
            </div>
          </div>
        );
      })}

      <div className="row" style={{ marginTop: 8 }}>
        <button className="primary" onClick={save} disabled={busy}>保存</button>
        <button onClick={reset} disabled={busy || !overridden}>既定に戻す</button>
      </div>
      <p className="hint">
        値は可逆オーバーライドとして保存（実測YAMLは保持）。経路生成・解析・寄り付き・フットプリントに即反映。
        <b>*</b>＝既定と異なる項目。全幅/全長を変えるとフットプリントも更新されます。
      </p>
    </section>
  );
}
