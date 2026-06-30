// JGD2011 平面直角座標系（系I〜XIX = EPSG:6669〜6687）を OpenLayers に登録。
// ネイティブ投影で扱うことで Web Mercator 再投影を避け、クリック座標を厳密化（3cm精度要件）。
// 作業ゾーンは起動時に backend(/health の default_epsg)から採用する（FRS_DEFAULT_EPSG で設定）。
import { register } from "ol/proj/proj4";
import proj4 from "proj4";

// [EPSG, 原点緯度lat_0, 原点経度lon_0(10進)] — 国土地理院の平面直角座標系 系I〜XIX。
const JGD2011_ZONES: [number, number, string][] = [
  [6669, 33, "129.5"],
  [6670, 33, "131"],
  [6671, 36, "132.166666666667"],
  [6672, 33, "133.5"],
  [6673, 36, "134.333333333333"],
  [6674, 36, "136"],
  [6675, 36, "137.166666666667"],
  [6676, 36, "138.5"],
  [6677, 36, "139.833333333333"],
  [6678, 40, "140.833333333333"],
  [6679, 44, "140.25"],
  [6680, 44, "142.25"],
  [6681, 44, "144.25"],
  [6682, 26, "142"],
  [6683, 26, "127.5"],
  [6684, 26, "124"],
  [6685, 26, "131"],
  [6686, 20, "136"],
  [6687, 26, "154"],
];

for (const [epsg, lat0, lon0] of JGD2011_ZONES) {
  proj4.defs(
    `EPSG:${epsg}`,
    `+proj=tmerc +lat_0=${lat0} +lon_0=${lon0} +k=0.9999 +x_0=0 +y_0=0 ` +
      "+ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs +type=crs",
  );
}
register(proj4);

// 作業CRS（プロジェクトの投影座標系）。既定は系IX(6677)。起動時に setWorkingEpsg で上書きする。
// ※ export let のライブバインディングにより、setWorkingEpsg 後に参照する利用側へ更新が伝わる。
export let WORKING_EPSG = 6677;
export let WORKING_CRS = "EPSG:6677";

// UIの作業ゾーン選択用（系I〜XIX = EPSG:6669〜6687）。
const ROMAN = ["I","II","III","IV","V","VI","VII","VIII","IX","X","XI","XII","XIII","XIV","XV","XVI","XVII","XVIII","XIX"];
export const JGD2011_ZONE_LIST: { epsg: number; label: string }[] = JGD2011_ZONES.map(
  ([epsg], i) => ({ epsg, label: `系${ROMAN[i]} (EPSG:${epsg})` }),
);

/** 作業ゾーンを切り替える（登録済みの JGD2011 ゾーンのみ）。地図描画前（起動時）に呼ぶ想定。 */
export function setWorkingEpsg(epsg: number): boolean {
  if (!Number.isFinite(epsg) || !proj4.defs(`EPSG:${epsg}`)) return false;
  WORKING_EPSG = Math.trunc(epsg);
  WORKING_CRS = `EPSG:${WORKING_EPSG}`;
  return true;
}
