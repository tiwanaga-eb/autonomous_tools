# streamlit_app.py
import os
from datetime import date, timedelta

import pandas as pd
import plotly.express as px
import streamlit as st

# ================== CONFIG ==================
st.set_page_config(
    page_title="小松養浜工事 無人化施工実績ダッシュボード",
    page_icon="📊",
    layout="wide",
)

# =============== THEME（濃い青系） ===============
NAVY = "#0F172A"
BLUE = "#1E40AF"
LIGHT_BLUE = "#2563EB"
CYAN = "#06B6D4"
GRID = "rgba(15, 23, 42, 0.15)"

px.defaults.template = "plotly_white"
px.defaults.color_discrete_sequence = [LIGHT_BLUE, CYAN, BLUE]

# =============== TITLE & STYLE ===============
st.title("小松養浜工事 無人化施工実績ダッシュボード")

st.markdown(
    f"""
<style>
:root {{
  --navy: {NAVY};
  --blue: {BLUE};
  --light-blue: {LIGHT_BLUE};
  --cyan: {CYAN};
}}
html, body, [data-testid="stAppViewContainer"] {{
  background:
    radial-gradient(1200px 500px at 20% -10%, rgba(37,99,235,0.12), transparent),
    radial-gradient(1000px 400px at 80% 0%, rgba(6,182,212,0.10), transparent),
    #F8FAFC;
}}
.kpi-card {{
  padding: 14px 16px;
  border-radius: 14px;
  background: linear-gradient(135deg, rgba(37,99,235,0.10), rgba(15,23,42,0.05));
  border: 1px solid rgba(0,0,0,0.06);
  box-shadow: 0 8px 18px rgba(15,23,42,0.12);
}}
.kpi-title {{ font-size: .88rem; color: #334155; margin-bottom: 6px; }}
.kpi-value {{ font-size: 2.0rem; font-weight: 800; color: var(--blue); }}
.kpi-sub   {{ font-size: .78rem; color: #475569; }}
[data-testid="stDataFrame"] div[role="table"] {{ font-size: .95rem; }}
</style>
""",
    unsafe_allow_html=True,
)

# =============== DATA IO ===============
DATA_PATH = "production_data.csv"
DATE_FMT = "%Y-%m-%d"

COL_ORDER = [
    "date",
    "operating_hours",
    "cycles",
    "volume_m3",
    "cycle_time_min",
    "breakdown_stops",
]

def normalize_df_for_save(df: pd.DataFrame) -> pd.DataFrame:
    """保存前の標準化：日付は YYYY-MM-DD、数値は適切にキャスト、列順固定"""
    df = df.copy()
    if "date" in df.columns:
        df["date"] = pd.to_datetime(df["date"], errors="coerce").dt.strftime(DATE_FMT)
    for c in ["operating_hours", "volume_m3", "cycle_time_min"]:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    for c in ["cycles", "breakdown_stops"]:
        df[c] = pd.to_numeric(df[c], errors="coerce").fillna(0).astype("Int64")
    for c in COL_ORDER:
        if c not in df.columns:
            df[c] = pd.NA
    return df[COL_ORDER]

def load_records() -> list[dict]:
    if not os.path.exists(DATA_PATH):
        return []
    df = pd.read_csv(DATA_PATH)
    df = normalize_df_for_save(df)  # 混在CSVでも自己修復
    return df.to_dict("records")

def save_records(records: list[dict]):
    df = pd.DataFrame(records)
    df = normalize_df_for_save(df)
    df.to_csv(DATA_PATH, index=False)

# 初期ロード
if "records" not in st.session_state:
    st.session_state.records = load_records()
    if st.session_state.records:
        st.toast("💾 データを読み込みました（日付正規化済）", icon="✅")

# =============== SIDEBAR ===============
with st.sidebar:
    st.header("データ入力（1日1行）")
    d = st.date_input("日付", value=date.today())
    operating_hours = st.number_input("運用時間 (h)", min_value=0.0, step=0.5, format="%.2f")
    cycles = st.number_input("サイクル数", min_value=0, step=1)
    volume_m3 = st.number_input("生産量 (m³)", min_value=0.0, step=1.0, format="%.2f")
    cycle_time = st.number_input("サイクルタイム (分/サイクル)", min_value=0.0, step=0.1, format="%.1f")
    breakdown_stops = st.number_input("故障停車回数", min_value=0, step=1)

    c1, c2 = st.columns(2)
    with c1:
        add_clicked = st.button("➕ 追加 / 上書き", use_container_width=True)
    with c2:
        clear_clicked = st.button("🗑️ 当日削除", use_container_width=True)

    st.divider()
    st.header("表示期間")
    period = st.radio("範囲", ["全期間", "直近14日", "任意範囲"], index=1)
    start_date, end_date = None, None
    if period == "任意範囲":
        start_date, end_date = st.date_input(
            "期間を選択",
            value=(date.today() - timedelta(days=13), date.today())
        )
    show_ma = st.checkbox("移動平均（3日）を重ねる", value=True)

# 入力処理
if add_clicked:
    date_str = d.strftime(DATE_FMT)
    st.session_state.records = [r for r in st.session_state.records if r.get("date") != date_str]
    st.session_state.records.append({
        "date": date_str,
        "operating_hours": operating_hours,
        "cycles": cycles,
        "volume_m3": volume_m3,
        "cycle_time_min": cycle_time,
        "breakdown_stops": breakdown_stops,
    })
    save_records(st.session_state.records)
    st.success("✅ データを追加・保存しました！")

if clear_clicked:
    date_str = d.strftime(DATE_FMT)
    before = len(st.session_state.records)
    st.session_state.records = [r for r in st.session_state.records if r.get("date") != date_str]
    save_records(st.session_state.records)
    after = len(st.session_state.records)
    if before != after:
        st.warning(f"🗑️ {date_str} のデータを削除しました。")
    else:
        st.info(f"{date_str} のデータは存在しません。")

# =============== TYPING & DAILY AGGREGATION ===============
def to_typed(df: pd.DataFrame) -> pd.DataFrame:
    if df.empty:
        return df
    df = normalize_df_for_save(df)
    df["date"] = pd.to_datetime(df["date"], format=DATE_FMT, errors="coerce")
    for c in ["operating_hours", "cycles", "volume_m3", "cycle_time_min", "breakdown_stops"]:
        df[c] = pd.to_numeric(df[c], errors="coerce").fillna(0)
    return df

def aggregate_daily(df: pd.DataFrame) -> pd.DataFrame:
    """日毎集計。CTはサイクル数で加重平均（cycles=0 は通常平均にフォールバック）。
       ※ groupby は『実在する列』で行うので FutureWarning/KeyError を回避。
    """
    if df.empty:
        return df
    d = df.copy()
    d["date_day"] = d["date"].dt.floor("D")  # 実在する列を作る

    # 合計値
    sums = d.groupby("date_day")[["operating_hours", "cycles", "volume_m3", "breakdown_stops"]].sum()

    # 加重平均CT
    num = (d["cycle_time_min"] * d["cycles"]).groupby(d["date_day"]).sum()
    den = d["cycles"].groupby(d["date_day"]).sum()
    fallback_mean = d.groupby("date_day")["cycle_time_min"].mean()
    ct = (num / den.replace(0, pd.NA)).fillna(fallback_mean)

    # まとめる（date は datetime64[ns]、日付のみ）
    daily = pd.DataFrame({
        "date": sums.index,
        "operating_hours": sums["operating_hours"].values,
        "cycles": sums["cycles"].values,
        "volume_m3": sums["volume_m3"].values,
        "breakdown_stops": sums["breakdown_stops"].values,
        "cycle_time_min": ct.values,
    }).sort_values("date")
    return daily

raw_df = to_typed(pd.DataFrame(st.session_state.records))
df = aggregate_daily(raw_df)

# 期間フィルタ（集計後の“日毎DF”に対して）
df_view = df.copy()
if not df_view.empty:
    if period == "直近14日":
        end_dt = df_view["date"].max().date()
        start_dt = end_dt - timedelta(days=13)
        df_view = df_view[(df_view["date"].dt.date >= start_dt) & (df_view["date"].dt.date <= end_dt)]
    elif period == "任意範囲" and start_date and end_date:
        df_view = df_view[(df_view["date"].dt.date >= start_date) & (df_view["date"].dt.date <= end_date)]

# =============== KPI（横一列） ===============
def kpi_row(df_a: pd.DataFrame, subtitle: str):
    if df_a.empty:
        st.info(f"{subtitle} のデータがありません。")
        return
    total_cycles = int(df_a["cycles"].sum())
    total_volume = float(df_a["volume_m3"].sum())
    total_hours = float(df_a["operating_hours"].sum())
    total_breaks = int(df_a["breakdown_stops"].sum())
    avg_ct = float(df_a["cycle_time_min"].mean())

    cols = st.columns(5)
    for c, (title, value, sub) in zip(
        cols,
        [
            ("総サイクル数", f"{total_cycles:,}", subtitle),
            ("総生産量", f"{total_volume:,.1f} m³", subtitle),
            ("総運用時間", f"{total_hours:,.1f} h", subtitle),
            ("故障停車合計", f"{total_breaks:,} 回", subtitle),
            ("平均サイクルタイム（日平均）", f"{avg_ct:,.2f} 分/サイクル", subtitle),
        ],
    ):
        with c:
            st.markdown(
                f'<div class="kpi-card">'
                f'<div class="kpi-title">{sub}</div>'
                f'<div class="kpi-value">{value}</div>'
                f'<div class="kpi-sub">{title}</div>'
                f'</div>',
                unsafe_allow_html=True,
            )

# =============== CHART HELPERS ===============
def apply_xaxis(fig):
    """横軸を日付のみ（YYYY-MM-DD）で表示 + 45°傾けて見やすく"""
    fig.update_layout(
        xaxis=dict(
            tickformat="%Y-%m-%d",
            tickangle=-45,
            gridcolor=GRID,
        ),
        yaxis=dict(gridcolor=GRID),
        margin=dict(l=10, r=10, t=40, b=60),
    )
    return fig

# =============== RENDER ===============
if df.empty:
    st.info("まだデータがありません。左のフォームから追加してください。")
else:
    st.subheader("🔢 KPI")
    kpi_row(df, "全期間")
    if len(df_view) and (len(df_view) != len(df)):
        kpi_row(df_view, "選択期間")

    tabs = st.tabs(["📈 グラフ（日毎・日付表示）", "🧾 データ一覧（日毎）", "⬇️ CSVダウンロード"])

    with tabs[0]:
        plot_df = df_view if len(df_view) else df
        plot_df = plot_df.copy()
        if show_ma:
            for col in ["volume_m3", "operating_hours", "cycle_time_min"]:
                plot_df[f"{col}_ma3"] = plot_df[col].rolling(3, min_periods=1).mean()

        c1, c2 = st.columns(2)
        with c1:
            fig1 = px.line(plot_df, x="date", y="volume_m3", markers=True, title="日毎の生産量 (m³)")
            if show_ma and "volume_m3_ma3" in plot_df:
                fig1.add_scatter(x=plot_df["date"], y=plot_df["volume_m3_ma3"], mode="lines", name="MA(3)")
            st.plotly_chart(apply_xaxis(fig1), use_container_width=True)

            fig2 = px.line(plot_df, x="date", y="operating_hours", markers=True, title="日毎の運用時間 (h)")
            if show_ma and "operating_hours_ma3" in plot_df:
                fig2.add_scatter(x=plot_df["date"], y=plot_df["operating_hours_ma3"], mode="lines", name="MA(3)")
            st.plotly_chart(apply_xaxis(fig2), use_container_width=True)

        with c2:
            fig3 = px.line(plot_df, x="date", y="cycle_time_min", markers=True, title="サイクルタイム (分/サイクル)")
            if show_ma and "cycle_time_min_ma3" in plot_df:
                fig3.add_scatter(x=plot_df["date"], y=plot_df["cycle_time_min_ma3"], mode="lines", name="MA(3)")
            st.plotly_chart(apply_xaxis(fig3), use_container_width=True)

            fig4 = px.bar(plot_df, x="date", y="breakdown_stops", title="故障停車回数")
            st.plotly_chart(apply_xaxis(fig4), use_container_width=True)

    with tabs[1]:
        view_df = (df_view if len(df_view) else df).sort_values("date", ascending=False).copy()
        view_df["date"] = view_df["date"].dt.strftime(DATE_FMT)
        st.dataframe(
            view_df.rename(
                columns={
                    "date": "日付",
                    "operating_hours": "運用時間(h)",
                    "cycles": "サイクル数",
                    "volume_m3": "生産量(m³)",
                    "cycle_time_min": "サイクルタイム(分/サイクル)",
                    "breakdown_stops": "故障停車回数",
                }
            ),
            use_container_width=True,
            height=420,
        )

    with tabs[2]:
        export_df = normalize_df_for_save(pd.DataFrame(st.session_state.records)).sort_values("date")
        st.download_button(
            "📥 CSVとして保存（全期間・正規化済み YYYY-MM-DD）",
            data=export_df.to_csv(index=False).encode("utf-8"),
            file_name="production_data.csv",
            mime="text/csv",
            use_container_width=True,
        )