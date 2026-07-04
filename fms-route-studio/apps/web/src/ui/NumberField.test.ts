// NumberField の確定/ライブ反映ロジック（純関数部）の検証。
// 「入力途中クランプで値が化ける」既知バグ（"0.5" と打つ途中の "0"、"-" → NaN）の回帰防止。
import { describe, expect, it } from "vitest";

import { clampTo, liveCommitValue, resolveCommit } from "@/ui/NumberField";

describe("resolveCommit（blur/Enter 時の確定値）", () => {
  it("通常の数値はパースしてそのまま", () => {
    expect(resolveCommit("2.35", 1, 0, 10)).toBe(2.35);
  });

  it("範囲外は min/max へクランプ", () => {
    expect(resolveCommit("15", 5, 0, 10)).toBe(10);
    expect(resolveCommit("-3", 5, 0, 10)).toBe(0);
  });

  it('解釈できない入力（"", "-", "abc"）は元の値へ戻す', () => {
    expect(resolveCommit("", 7, 0, 10)).toBe(7);
    expect(resolveCommit("-", 7, 0, 10)).toBe(7);
    expect(resolveCommit("abc", 7, 0, 10)).toBe(7);
  });

  it("min/max 未指定なら無制限", () => {
    expect(resolveCommit("-9999", 0)).toBe(-9999);
  });
});

describe("liveCommitValue（タイプ中のライブ反映可否）", () => {
  it("範囲内の完全な数値だけライブ反映する", () => {
    expect(liveCommitValue("3", 0, 10)).toBe(3);
    expect(liveCommitValue("0.5", 0, 10)).toBe(0.5);
  });

  it('途中入力（"-", ""）は反映しない（null）', () => {
    expect(liveCommitValue("-", 0, 10)).toBeNull();
    expect(liveCommitValue("", 0, 10)).toBeNull();
  });

  it("範囲外はライブ反映せず確定時に丸める（タイプ中のクランプ乗っ取り防止）", () => {
    // "25" と打つ途中の "2" は範囲内なので反映、"25" は範囲外なので保持
    expect(liveCommitValue("2", 0, 10)).toBe(2);
    expect(liveCommitValue("25", 0, 10)).toBeNull();
  });
});

describe("clampTo", () => {
  it("境界値を含む", () => {
    expect(clampTo(0, 0, 10)).toBe(0);
    expect(clampTo(10, 0, 10)).toBe(10);
    expect(clampTo(11, 0, 10)).toBe(10);
  });
});
