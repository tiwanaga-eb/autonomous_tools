// SmartConstruction ブランドマーク（六角バッジ＋S）。レール/トップバー兼用。
export function BrandMark({ className, onBlue = true }: { className?: string; onBlue?: boolean }) {
  // onBlue: 青背景に置く想定（白いバッジ）。false なら青いバッジ（白背景用）。
  const badge = onBlue ? "#ffffff" : "#1f44d6";
  const glyph = onBlue ? "#1f44d6" : "#ffffff";
  return (
    <svg className={className} viewBox="0 0 40 40" fill="none" aria-label="SmartConstruction">
      <path
        d="M20 2.5 34.4 11v18L20 37.5 5.6 29V11L20 2.5z"
        fill={badge}
        stroke={onBlue ? "#ffffff" : "#1f44d6"}
        strokeWidth="1.5"
        strokeLinejoin="round"
      />
      {/* スタイライズされた S（2ストローク） */}
      <path
        d="M25 14.5c-1.6-1.5-4-2.2-6.2-1.6-2.6.7-3.6 3.4-1.6 5 1.4 1.1 4.2 1.2 6 2.4 2.2 1.5 1.4 4.6-1.4 5.4-2.4.7-5-.1-6.8-1.8"
        stroke={glyph}
        strokeWidth="2.6"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}
