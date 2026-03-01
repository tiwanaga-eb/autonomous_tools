from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass
class OpenItem:
    f: float
    g: float
    node_id: int
    key: Any


class PriorityOpenSet:
    def __init__(self):
        self._heap = []
        self._counter = 0

    def push(self, f: float, g: float, key: Any) -> None:
        self._counter += 1
        heapq.heappush(self._heap, (f, self._counter, g, key))

    def pop(self) -> Optional[OpenItem]:
        if not self._heap:
            return None
        f, node_id, g, key = heapq.heappop(self._heap)
        return OpenItem(f=f, g=g, node_id=node_id, key=key)

    def __len__(self) -> int:
        return len(self._heap)


class ClosedBestG:
    def __init__(self):
        self.best: Dict[Any, float] = {}

    def is_better(self, key: Any, g: float) -> bool:
        old = self.best.get(key)
        if old is None or g < old:
            self.best[key] = g
            return True
        return False
