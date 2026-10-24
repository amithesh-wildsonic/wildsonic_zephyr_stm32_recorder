"""Deterministic reservation/interleaving model for the PSRAM staging ring.

These host tests exercise the documented state machine, not the Zephyr C object;
the C source is also checked for the corresponding reserve/write/publish order.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / "app"


class Ring:
    def __init__(self, capacity):
        self.mem = bytearray(capacity)
        self.capacity = capacity
        self.head = self.tail = self.occupied = self.reserved = 0
        self.inflight = False
        self.drops = self.accepted = self.flushed = 0

    @property
    def free(self):
        return self.capacity - self.occupied - self.reserved

    def reserve(self, data):
        if self.inflight:
            return -16  # -EBUSY
        if len(data) > self.free:
            return -105  # -ENOBUFS
        self.inflight = True
        self.reserved = len(data)
        return self.head

    def finish(self, start, data, fail_after=None):
        first = min(len(data), self.capacity - start)
        self.mem[start:start + first] = data[:first]
        if fail_after == 1:
            self.reserved = 0
            self.inflight = False
            self.drops += 1
            return -5
        self.mem[:len(data) - first] = data[first:]
        self.head = (start + len(data)) % self.capacity
        self.occupied += len(data)
        self.accepted += len(data)
        self.reserved = 0
        self.inflight = False
        return 0

    def peek(self, n):
        n = min(n, self.occupied)
        first = min(n, self.capacity - self.tail)
        return bytes(self.mem[self.tail:self.tail + first] + self.mem[:n - first])

    def commit(self, n):
        if n > self.occupied:
            return -22
        self.tail = (self.tail + n) % self.capacity
        self.occupied -= n
        self.flushed += n
        return 0


def test_reservation_is_invisible_until_complete_write():
    ring = Ring(8)
    pos = ring.reserve(b"abcd")
    assert ring.inflight and ring.reserved == 4 and ring.occupied == 0
    assert ring.peek(8) == b""
    assert ring.finish(pos, b"abcd") == 0
    assert ring.peek(8) == b"abcd"


def test_failed_wrapped_write_keeps_committed_fifo_and_reclaims_reservation():
    ring = Ring(8)
    pos = ring.reserve(b"abcdef")
    assert ring.finish(pos, b"abcdef") == 0
    ring.commit(5)
    pos = ring.reserve(b"WXYZ")  # starts at offset 6 and wraps
    before = ring.peek(8)
    assert ring.finish(pos, b"WXYZ", fail_after=1) == -5
    assert ring.peek(8) == before
    assert ring.free == 7 and ring.drops == 1 and ring.accepted == 6


def test_consumer_can_commit_old_bytes_while_new_write_is_reserved():
    ring = Ring(8)
    pos = ring.reserve(b"1234")
    ring.finish(pos, b"1234")
    pos = ring.reserve(b"5678")
    assert ring.free == 0
    assert ring.commit(2) == 0
    assert ring.peek(8) == b"34"
    assert ring.free == 2  # four unread/reserved bytes remain accounted
    assert ring.finish(pos, b"5678") == 0
    assert ring.peek(8) == b"345678"


def test_source_publishes_only_after_psram_write_and_accounts_reservation():
    src = (ROOT / "src" / "psram_staging.c").read_text()
    push = src.split("int psram_staging_push", 1)[1].split("int psram_staging_peek", 1)[0]
    assert "reserved_bytes = bytes" in push
    assert "count_occupied += bytes" in push
    assert push.index("psram_write(") < push.index("count_occupied += bytes")
    assert "count_occupied + reserved_bytes" in src


if __name__ == "__main__":
    test_reservation_is_invisible_until_complete_write()
    test_failed_wrapped_write_keeps_committed_fifo_and_reclaims_reservation()
    test_consumer_can_commit_old_bytes_while_new_write_is_reserved()
    test_source_publishes_only_after_psram_write_and_accounts_reservation()
    print("ALL test_psram_staging_concurrency TESTS PASSED!")

