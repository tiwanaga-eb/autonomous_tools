/**
 * Tiny binary min-heap keyed on a numeric priority field of the stored item.
 *
 * Generic, dependency-free, used by Hybrid A* as the OPEN set.
 */
export class MinHeap<T extends { priority: number }> {
  private data: T[] = [];

  get size(): number {
    return this.data.length;
  }

  push(item: T): void {
    this.data.push(item);
    this.bubbleUp(this.data.length - 1);
  }

  pop(): T | undefined {
    if (this.data.length === 0) return undefined;
    const top = this.data[0];
    const last = this.data.pop()!;
    if (this.data.length > 0) {
      this.data[0] = last;
      this.bubbleDown(0);
    }
    return top;
  }

  private bubbleUp(i: number): void {
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (this.data[parent].priority <= this.data[i].priority) break;
      [this.data[parent], this.data[i]] = [this.data[i], this.data[parent]];
      i = parent;
    }
  }

  private bubbleDown(i: number): void {
    const n = this.data.length;
    while (true) {
      const l = 2 * i + 1;
      const r = 2 * i + 2;
      let best = i;
      if (l < n && this.data[l].priority < this.data[best].priority) best = l;
      if (r < n && this.data[r].priority < this.data[best].priority) best = r;
      if (best === i) break;
      [this.data[best], this.data[i]] = [this.data[i], this.data[best]];
      i = best;
    }
  }
}
