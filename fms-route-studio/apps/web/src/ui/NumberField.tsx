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

export function NumberField({ value, onCommit, step, min, max, title, disabled, style }: Props) {
  const [text, setText] = useState(String(value));
  const [focused, setFocused] = useState(false);

  // 外部から値が変わったら（プロジェクト読込等）、編集中でなければ表示へ同期
  useEffect(() => {
    if (!focused) setText(String(value));
  }, [value, focused]);

  const clamp = (v: number) => Math.min(max ?? Infinity, Math.max(min ?? -Infinity, v));

  const commit = () => {
    const v = parseFloat(text);
    if (Number.isFinite(v)) {
      const c = clamp(v);
      onCommit(c);
      setText(String(c));
    } else {
      setText(String(value)); // 解釈できない入力は元の値へ戻す
    }
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
        const v = parseFloat(t);
        // 範囲内の完全な数値はライブ反映（スピナー操作・通常入力が即効く）。
        // 範囲外/途中の入力はクランプせず保持し、確定時に丸める。
        if (Number.isFinite(v) && v === clamp(v)) onCommit(v);
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
