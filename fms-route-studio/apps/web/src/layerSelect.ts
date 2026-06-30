// レイヤ解決の共通ヘルパ。明示id（プロジェクト保存時に固定したレイヤ）が今も存在すれば
// それを優先し、無ければ kind の最新（末尾＝直近アップロード）にフォールバックする。
// これで「保存→再読込」時に作成当時のコスト/走行可能領域へ正しく紐づく（レビュー#20）。
import type { Layer } from "@/types/api";

export function pickLayer(
  layers: Layer[],
  kind: string,
  explicitId: string | null | undefined,
): Layer | undefined {
  if (explicitId) {
    const hit = layers.find((l) => l.id === explicitId && l.kind === kind);
    if (hit) return hit;
  }
  return [...layers].reverse().find((l) => l.kind === kind);
}
