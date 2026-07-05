// 手入力に強い数値入力。
// 素の <input type="number" + onChange クランプ> は、入力途中（"0.5" と打つ途中の "0"、
// 空欄、"-"）が即クランプされて値が化けるため使い物にならない。本コンポーネントは
// 編集中はローカルの文字列を保持し、**確定時（blur / Enter / スピナー操作）にだけ**
// パース＋min/max クランプして親へ反映する。範囲内の値はタイプ中もライブ反映される。
import { useEffect, useState } from "react";
import type { CSSProperties, KeyboardEvent } from "react";

interface Props {
  value: number;
  onCommit: (v: number) => void;
  step?: number;
  min?: number;
  max?: number;
  title?: string;
  disabled?: boolean;
  style?: CSSProperties;
}

export function clampTo(v: number, min?: number, max?: number): number {
  return Math.min(max ?? Infinity, Math.max(min ?? -Infinity, v));
}

/** 確定時（blur/Enter）の解決値。解釈できない入力（"", "-", "1e" 等）は prev へ戻す。 */
export function resolveCommit(text: string, prev: number, min?: number, max?: number): number {
  const v = parseFloat(text);
  return Number.isFinite(v) ? clampTo(v, min, max) : prev;
}

/** タイプ中にライブ反映してよい値。範囲内の完全な数値のみ（途中入力はクランプせず保持）。 */
export function liveCommitValue(text: string, min?: number, max?: number): number | null {
  const v = parseFloat(text);
  return Number.isFinite(v) && v === clampTo(v, min, max) ? v : null;
}

export function NumberField({ value, onCommit, step, min, max, title, disabled, style }: Props) {
  const [text, setText] = useState(String(value));
  const [focused, setFocused] = useState(false);

  // 外部から値が変わったら（プロジェクト読込等）、編集中でなければ表示へ同期
  useEffect(() => {
    if (!focused) setText(String(value));
  }, [value, focused]);

  const commit = () => {
    const c = resolveCommit(text, value, min, max);
    if (c !== value) onCommit(c);
    setText(String(c));
  };

  return (
    <input
      type="number"
      value={text}
      step={step}
      min={min}
      max={max}
      title={title}
      disabled={disabled}
      style={style}
      onChange={(e) => {
        const t = e.target.value;
        setText(t);
        // 範囲内の完全な数値はライブ反映（スピナー操作・通常入力が即効く）。
        // 範囲外/途中の入力はクランプせず保持し、確定時に丸める。
        const v = liveCommitValue(t, min, max);
        if (v !== null) onCommit(v);
      }}
      onFocus={() => setFocused(true)}
      onBlur={() => {
        setFocused(false);
        commit();
      }}
      onKeyDown={(e: KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") (e.target as HTMLInputElement).blur();
      }}
    />
  );
}
