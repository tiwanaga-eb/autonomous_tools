"""鉱山/土木シナリオのチューニングレポートを生成。
実行: PYTHONPATH=packages/planning_core python scripts/scenario_report.py
"""
from planning_core.scenarios import write_report

if __name__ == "__main__":
    out = "docs/SCENARIO_REPORT.md"
    write_report(out)
    print(f"wrote {out}")
