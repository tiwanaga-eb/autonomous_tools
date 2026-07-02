"""store.py（JSON レイヤレジストリ）の整合性テスト。

対象: 原子的書込（tmp→os.replace）、_LOCK による read-modify-write の直列化、
delete_layer のレジストリ/ディスク両方の削除。conftest が FRS_DATA_DIR を
テンポラリに向けるため実データには触れない。
"""
from __future__ import annotations

import json
import threading

from app import store
from app.settings import REGISTRY_PATH


def _meta(layer_id: str, kind: str = "cost") -> dict:
    return {"id": layer_id, "kind": kind, "version": 1}


def test_add_get_list_roundtrip():
    lid = store.new_layer_id("cost")
    store.add_layer(_meta(lid))
    assert store.get_layer(lid) == _meta(lid)
    assert any(l["id"] == lid for l in store.list_layers())


def test_new_layer_id_prefix_and_uniqueness():
    ids = {store.new_layer_id("drivable") for _ in range(50)}
    assert len(ids) == 50
    assert all(i.startswith("drivable_") for i in ids)


def test_save_is_atomic_no_tmp_leftover_and_valid_json():
    lid = store.new_layer_id("cost")
    store.add_layer(_meta(lid))
    # tmp ファイルが残っていない（os.replace 済み）
    assert not REGISTRY_PATH.with_suffix(".json.tmp").exists()
    # レジストリは常に valid JSON で layers キーを持つ
    reg = json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))
    assert "layers" in reg and lid in reg["layers"]


def test_delete_layer_removes_entry_and_dir():
    lid = store.new_layer_id("cost")
    store.add_layer(_meta(lid))
    d = store.layer_dir(lid)
    d.mkdir(parents=True, exist_ok=True)
    (d / "cog.tif").write_bytes(b"x")

    assert store.delete_layer(lid) is True
    assert store.get_layer(lid) is None
    assert not d.exists()


def test_delete_missing_layer_returns_false():
    assert store.delete_layer("cost_nonexistent") is False


def test_concurrent_adds_are_not_lost():
    """_LOCK が read-modify-write を直列化し、並行 add で last-writer-wins に
    ならないこと（32 スレッド全件がレジストリに残る）。"""
    ids = [store.new_layer_id("cost") for _ in range(32)]
    barrier = threading.Barrier(len(ids))

    def worker(lid: str) -> None:
        barrier.wait()  # 全スレッド同時に read-modify-write を開始させる
        store.add_layer(_meta(lid))

    threads = [threading.Thread(target=worker, args=(lid,)) for lid in ids]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    stored = {l["id"] for l in store.list_layers()}
    missing = [lid for lid in ids if lid not in stored]
    assert not missing, f"lost updates: {missing}"


def test_concurrent_add_and_delete_registry_stays_valid():
    """add と delete が競合してもレジストリが破損（不正 JSON / layers 欠落）しないこと。"""
    keep = [store.new_layer_id("cost") for _ in range(8)]
    victim = [store.new_layer_id("cost") for _ in range(8)]
    for lid in victim:
        store.add_layer(_meta(lid))

    def adder() -> None:
        for lid in keep:
            store.add_layer(_meta(lid))

    def deleter() -> None:
        for lid in victim:
            store.delete_layer(lid)

    ts = [threading.Thread(target=adder), threading.Thread(target=deleter)]
    for t in ts:
        t.start()
    for t in ts:
        t.join()

    reg = json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))
    assert "layers" in reg
    stored = set(reg["layers"])
    assert all(lid in stored for lid in keep)
    assert all(lid not in stored for lid in victim)
