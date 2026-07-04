// 工程ナビ（サイドバー）。選択中の機能だけを作業パネルに出す（設計コンセプト）。
import type { ReactNode } from "react";

import { useStore } from "@/store/useStore";
import type { FeatureId } from "@/store/useStore";
import { BrandMark } from "@/ui/BrandMark";

const I = (p: { children: ReactNode }) => (
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor"
       strokeWidth={1.9} strokeLinecap="round" strokeLinejoin="round" aria-hidden>
    {p.children}
  </svg>
);

const ICONS: Record<FeatureId, ReactNode> = {
  ai: <I><circle cx="12" cy="12" r="9" /><path d="M8 12h.01M12 12h.01M16 12h.01" /></I>,
  data: <I><path d="M3 7l9-4 9 4-9 4-9-4z" /><path d="M3 12l9 4 9-4M3 17l9 4 9-4" /></I>,
  map: <I><path d="M9 4 3 6v14l6-2 6 2 6-2V4l-6 2-6-2z" /><path d="M9 4v14M15 6v14" /></I>,
  route: <I><circle cx="5" cy="19" r="2" /><circle cx="19" cy="5" r="2" /><path d="M7 18C13 16 8 8 17 6" /></I>,
  vehicle: <I><path d="M3 13h13l3 3v2h-3M3 13V8h10l3 5M3 13v5h2" /><circle cx="7.5" cy="18" r="1.6" /><circle cx="17" cy="18" r="1.6" /></I>,
  spotting: <I><path d="M12 3v6M12 21a7 7 0 1 0-4-12.7" /><path d="M9 6 12 3l3 3" /></I>,
  areas: <I><path d="M5 4h9l5 5v11H5z" /><path d="M9 9h6M9 13h6M9 17h4" /></I>,
  piles: <I><path d="M3 19h18" /><path d="M5 19l4-8 3 4 3-6 4 10" /></I>,
  fleet: <I><circle cx="6" cy="7" r="2" /><circle cx="18" cy="7" r="2" /><path d="M6 9v4l6 4 6-4V9" /><path d="M12 21v-4" /></I>,
  project: <I><path d="M4 5a2 2 0 0 1 2-2h8l6 6v10a2 2 0 0 1-2 2H6a2 2 0 0 1-2-2z" /><path d="M14 3v6h6" /></I>,
};

const NAV: { id: FeatureId; label: string }[] = [
  { id: "ai", label: "AI" },
  { id: "data", label: "データ" },
  { id: "map", label: "マップ" },
  { id: "route", label: "経路" },
  { id: "vehicle", label: "車両" },
  { id: "spotting", label: "寄り付き" },
  { id: "areas", label: "エリア" },
  { id: "piles", label: "排土" },
  { id: "fleet", label: "複数台" },
  { id: "project", label: "保存" },
];

export function Sidebar() {
  const active = useStore((s) => s.activeFeature);
  const setActive = useStore((s) => s.setActiveFeature);
  const layers = useStore((s) => s.layers);
  const route = useStore((s) => s.route);
  const spotResult = useStore((s) => s.spotResult);
  const pilePlan = useStore((s) => s.pilePlan);
  const fleetConflicts = useStore((s) => s.fleetConflicts);
  const fleetSim = useStore((s) => s.fleetSim);

  // 工程の進捗ドット（データ/マップ/経路/寄り付き/排土/複数台の成果物があるか）
  const dot: Partial<Record<FeatureId, boolean>> = {
    data: layers.some((l) => l.kind === "las" || l.kind === "ortho"),
    map: layers.some((l) => l.kind === "cost" || l.kind === "drivable"),
    route: !!route,
    spotting: !!spotResult,
    piles: !!pilePlan,
    fleet: !!fleetConflicts || !!fleetSim,
  };

  return (
    <nav className="rail">
      <BrandMark className="rail-logo" />
      <div className="rail-nav">
        {NAV.map((n) => (
          <button
            key={n.id}
            className="rail-item"
            data-active={active === n.id}
            onClick={() => setActive(n.id)}
            title={n.label}
          >
            {dot[n.id] && <span className="ri-dot" />}
            <span className="ri-icon">{ICONS[n.id]}</span>
            <span className="ri-label">{n.label}</span>
          </button>
        ))}
      </div>
    </nav>
  );
}
