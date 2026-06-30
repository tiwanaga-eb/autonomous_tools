import os
import sys
import tempfile
from pathlib import Path

# テスト用データディレクトリ（app import 前に設定する必要あり）
os.environ.setdefault("FRS_DATA_DIR", tempfile.mkdtemp(prefix="frs_api_test_"))

# services/api を import パスに（`import app` 用）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
