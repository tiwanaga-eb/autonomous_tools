import { useEffect } from "react";

import { api } from "@/api/client";
import { dispatch } from "@/commandBus";
import { MapView } from "@/map/MapView";
import { ThreeView } from "@/map/ThreeView";
import { AnalysisPanel } from "@/panels/AnalysisPanel";
import { AreaPanel } from "@/panels/AreaPanel";
import { ChatPanel } from "@/panels/ChatPanel";
import { CostmapPanel } from "@/panels/CostmapPanel";
import { DrivableAreaPanel } from "@/panels/DrivableAreaPanel";
import { FleetPanel } from "@/panels/FleetPanel";
import { IOPanel } from "@/panels/IOPanel";
import { LayerPanel } from "@/panels/LayerPanel";
import { PilePanel } from "@/panels/PilePanel";
import { ProjectPanel } from "@/panels/ProjectPanel";
import { RoutePanel } from "@/panels/RoutePanel";
import { SpottingPanel } from "@/panels/SpottingPanel";
import { VehiclePanel } from "@/panels/VehiclePanel";
import { useStore } from "@/store/useStore";
import type { FeatureId } from "@/store/useStore";
import { WORKING_EPSG, JGD2011_ZONE_LIST } from "@/map/proj";
import { BrandMark } from "@/ui/BrandMark";
import { Sidebar } from "@/ui/Sidebar";

const FEATURES: Record<FeatureId, { title: string; render: () => JSX.Element }> = {
  ai: { title: "AI アシスタント", render: () => <ChatPanel /> },
  data: { title: "データ（レイヤ）", render: () => <LayerPanel /> },
  map: {
    title: "マップ生成（コスト / 走行可能領域）",
    render: () => (
      <>
        <CostmapPanel />
        <DrivableAreaPanel />
      </>
    ),
  },
  route: { title: "経路", render: () => <RoutePanel /> },
  vehicle: { title: "車両パラメータ", render: () => <VehiclePanel /> },
  spotting: { title: "寄り付きシミュレータ", render: () => <SpottingPanel /> },
  areas: { title: "エリア", render: () => <AreaPanel /> },
  piles: { title: "排土（パイル）配置", render: () => <PilePanel /> },
  fleet: { title: "複数台（経路の競合判定）", render: () => <FleetPanel /> },
  project: {
    title: "プロジェクト / 入出力",
    render: () => (
      <>
        <ProjectPanel />
        <IOPanel />
      </>
    ),
  },
};

export function App() {
  const setLayers = useStore((s) => s.setLayers);
  const setStatus = useStore((s) => s.setStatus);
  const status = useStore((s) => s.status);
  const statusLevel = useStore((s) => s.statusLevel);
  const busy = useStore((s) => s.busy);
  const busyLabel = useStore((s) => s.busyLabel);
  const activeFeature = useStore((s) => s.activeFeature);
  const view3d = useStore((s) => s.view3d);
  const setView3d = useStore((s) => s.setView3d);
  const vExag = useStore((s) => s.vExag);
  const setVExag = useStore((s) => s.setVExag);
  const view3dMode = useStore((s) => s.view3dMode);
  const setView3dMode = useStore((s) => s.setView3dMode);
  const pointSize = useStore((s) => s.pointSize);
  const setPointSize = useStore((s) => s.setPointSize);
  const pointBudget = useStore((s) => s.pointBudget);
  const setPointBudget = useStore((s) => s.setPointBudget);

  useEffect(() => {
    api
      .listLayers()
      .then(setLayers)
      .catch(() => setStatus("バックエンドに接続できません（API を :8077 で起動してください）", "error"));
  }, [setLayers, setStatus]);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      // 入力欄での編集を妨げない
      const t = e.target as HTMLElement | null;
      if (t && ["INPUT", "TEXTAREA", "SELECT"].includes(t.tagName)) return;
      if (e.metaKey || e.ctrlKey) {
        if (e.key.toLowerCase() === "z") {
          e.preventDefault();
          dispatch(e.shiftKey ? { type: "REDO" } : { type: "UNDO" });
        }
        return;
      }
      // ポリゴン描画中のキーボード操作: Esc=取消 / Backspace=頂点を1つ戻す
      if (useStore.getState().mode === "polygon") {
        if (e.key === "Escape") {
          e.preventDefault();
          dispatch({ type: "CANCEL_POLYGON" });
        } else if (e.key === "Backspace") {
          e.preventDefault();
          dispatch({ type: "UNDO_POLY_VERTEX" });
        }
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  // 作業ゾーン(投影座標系)を実行時に切り替える。
  // ラスタ(COG)はアップロード時のゾーンに焼き込まれ実行時に再投影できないため、
  // backend の作業EPSGを更新したうえでページを再読込し、新ゾーンでクリーンに再初期化する。
  async function onChangeWorkingZone(epsg: number) {
    if (!epsg || epsg === WORKING_EPSG) return;
    const label = JGD2011_ZONE_LIST.find((z) => z.epsg === epsg)?.label ?? `EPSG:${epsg}`;
    const ok = window.confirm(
      `作業ゾーンを ${label} に変更します。\n\n` +
        "・ページを再読込します（未保存の作業は失われます。先にプロジェクトを保存してください）。\n" +
        "・以後に読み込むデータはこのゾーンへ変換されます。\n" +
        "・別ゾーンで取り込み済みのラスタ(ortho/コスト等)はそのゾーンで再生成が必要です。\n\n" +
        "続行しますか？",
    );
    if (!ok) return;
    try {
      await api.setWorkingCrs(epsg);
    } catch (e) {
      setStatus(`作業CRSの変更に失敗しました: ${e}`, "error");
      return;
    }
    window.location.reload();
  }

  const feature = FEATURES[activeFeature];

  return (
    <div className="app-shell">
      <Sidebar />

      <header className="topbar">
        <BrandMark className="tb-mark" onBlue={false} />
        <span className="tb-name">SmartConstruction</span>
        <span className="tb-sub">FMS Route Studio</span>
        <span className="tb-spacer" />
        <label className="tb-sub tb-crs" title="作業ゾーン（投影座標系）。読み込んだデータはこのゾーンへ変換されます。変更すると再読込します。">
          作業CRS:
          <select
            value={WORKING_EPSG}
            onChange={(e) => onChangeWorkingZone(Number(e.target.value))}
          >
            {JGD2011_ZONE_LIST.map((z) => (
              <option key={z.epsg} value={z.epsg}>
                {z.label}
              </option>
            ))}
          </select>
        </label>
        <span className="tb-status" data-level={busy ? "busy" : statusLevel}>
          <span className="dot" />
          {busy ? busyLabel || "処理中…" : status}
        </span>
      </header>

      <aside className="pane feature">
        <div className="feature-head">
          <span className="fh-accent" />
          <h2>{feature.title}</h2>
        </div>
        {feature.render()}
      </aside>

      <main className="pane center" style={{ position: "relative" }}>
        <div className="view-toggle">
          <button data-active={!view3d} onClick={() => setView3d(false)}>
            2D
          </button>
          <button data-active={view3d} onClick={() => setView3d(true)}>
            3D
          </button>
          {view3d && (
            <>
              <button data-active={view3dMode === "points"} onClick={() => setView3dMode("points")} title="LAS点群">
                点群
              </button>
              <button data-active={view3dMode === "mesh"} onClick={() => setView3dMode("mesh")} title="DSM地形メッシュ">
                地形
              </button>
              <label className="view-vexag" title="鉛直強調">
                z×{vExag.toFixed(1)}
                <input type="range" min={1} max={6} step={0.5} value={vExag} onChange={(e) => setVExag(+e.target.value)} />
              </label>
              {view3dMode === "points" && (
                <>
                  <label className="view-vexag" title="点サイズ">
                    pt{pointSize.toFixed(1)}
                    <input type="range" min={1} max={6} step={0.5} value={pointSize} onChange={(e) => setPointSize(+e.target.value)} />
                  </label>
                  <label className="view-vexag" title="表示する点数の上限（多いほど密だが読込・描画が重くなる）">
                    点数
                    <select value={pointBudget} onChange={(e) => setPointBudget(+e.target.value)}>
                      <option value={200_000}>20万</option>
                      <option value={500_000}>50万</option>
                      <option value={1_000_000}>100万</option>
                      <option value={2_000_000}>200万</option>
                      <option value={4_000_000}>400万</option>
                      <option value={8_000_000}>800万</option>
                      <option value={16_000_000}>1600万</option>
                    </select>
                  </label>
                </>
              )}
            </>
          )}
        </div>
        {view3d ? <ThreeView /> : <MapView />}
        {busy && (
          <div className="busy-overlay">
            <span className="busy-spinner" />
            <span>{busyLabel || "処理中…"}</span>
          </div>
        )}
      </main>

      <aside className="pane analysis">
        <AnalysisPanel />
      </aside>
    </div>
  );
}
