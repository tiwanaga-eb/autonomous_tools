#!/usr/bin/env python3
"""pytest が無い環境でも planning_core のテストを実行する簡易ランナー。

使い方:
    python tests/run_tests.py
（pytest があるなら `pytest -q` 推奨）
"""
from __future__ import annotations

import importlib
import sys
import traceback
from pathlib import Path

TEST_MODULES = [
    "test_geometry",
    "test_curvature",
    "test_spline",
    "test_costmap",
    "test_vehicle",
]


def main() -> int:
    here = Path(__file__).resolve().parent
    pkg_root = here.parent  # packages/planning_core （import planning_core 用）
    sys.path.insert(0, str(pkg_root))
    sys.path.insert(0, str(here))

    passed = failed = 0
    failures: list[str] = []
    for modname in TEST_MODULES:
        mod = importlib.import_module(modname)
        for name in sorted(dir(mod)):
            if not name.startswith("test_"):
                continue
            fn = getattr(mod, name)
            if not callable(fn):
                continue
            try:
                fn()
                passed += 1
                print(f"PASS  {modname}.{name}")
            except Exception:  # noqa: BLE001
                failed += 1
                failures.append(f"{modname}.{name}")
                print(f"FAIL  {modname}.{name}")
                traceback.print_exc()

    print(f"\n==== {passed} passed, {failed} failed ====")
    if failures:
        print("failed:", ", ".join(failures))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
