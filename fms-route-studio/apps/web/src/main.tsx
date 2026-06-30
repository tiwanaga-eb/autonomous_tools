import React from "react";
import ReactDOM from "react-dom/client";

import { api } from "@/api/client";
import { App } from "@/App";
import { setWorkingEpsg } from "@/map/proj";
import "@/ui/styles.css";

// 作業ゾーン(投影座標系)を backend(/health の default_epsg)から起動時に採用する。
// 地図 View 生成前に確定させたいので、描画前に取得する（失敗時は既定6677のまま）。
async function boot() {
  try {
    const h = await api.health();
    if (h?.default_epsg) setWorkingEpsg(h.default_epsg);
  } catch {
    /* backend 未起動などは既定 6677 で続行 */
  }
  ReactDOM.createRoot(document.getElementById("root")!).render(
    <React.StrictMode>
      <App />
    </React.StrictMode>,
  );
}

void boot();
