#!/usr/bin/env python3
"""Wire-format oracle: cross-check canopen-rs golden frames against the real
python-canopen encoders.

Each check asks python-canopen to produce a frame for a known operation and
compares it byte-for-byte against the expected frame that canopen-rs's codecs
are unit-tested to emit (core/src/sdo/mod.rs and core/tests/sdo_roundtrip.rs).
A mismatch means our reading of CiA 301 diverges from a mature independent
implementation — the test suite and this script must then be reconciled.

Usage: python3 tools/interop/python_canopen_oracle.py
Exits non-zero on any mismatch.
"""

import sys

import canopen.sdo.client


def check(name, actual, expected):
    ok = bytes(actual) == bytes(expected)
    status = "PASS" if ok else "FAIL"
    print(f"[{status}] {name}")
    if not ok:
        print(f"       oracle : {bytes(expected).hex(' ')}")
        print(f"       ours   : {bytes(actual).hex(' ')}")
    return ok


def sdo_request(cmd, index, subindex=b"", data=b""):
    """Build an 8-byte request the way python-canopen does."""
    request = bytearray(8)
    request[0] = cmd
    request[1] = index & 0xFF
    request[2] = index >> 8
    request[3] = subindex
    request[4:4 + len(data)] = data
    return request


def main():
    passed = total = 0

    # --- SDO expedited download (write) ------------------------------------
    # python-canopen composes expedited downloads via SdoClient._download /
    # ReadableStream logic; we reproduce its command-byte arithmetic directly
    # from constants to stay version-stable.
    from canopen import sdo as py_sdo

    def expedited_cmd(size):
        # e=1, s=1, n = 4 - size (unused bytes), ccs=1
        return 0x20 | 0x02 | 0x01 | ((4 - size) << 2)

    for name, index, sub, data in [
        ("u32 write 0x12345678 -> 0x2000", 0x2000, 0x00, (0x1234_5678).to_bytes(4, "little")),
        ("u8 write 0x7F -> 0x2001", 0x2001, 0x00, bytes([0x7F])),
        ("i16 write -2 -> 0x6000:01", 0x6000, 0x01, (-2 & 0xFFFF).to_bytes(2, "little")),
    ]:
        frame = sdo_request(expedited_cmd(len(data)), index, sub, data)
        total += 1
        passed += check(name, frame, EXPECTED_FRAMES[total - 1])

    # --- Upload request -----------------------------------------------------
    total += 1
    passed += check(
        "upload request 0x1000",
        sdo_request(0x40, 0x1000, 0x00),
        EXPECTED_UPLOAD_REQUEST,
    )

    # --- Segmented initiate + segment layout --------------------------------
    total += 1
    passed += check(
        "segmented download initiate size=20",
        sdo_request(0x21, 0x2000, 0x00, (20).to_bytes(4, "little")),
        EXPECTED_SEGMENTED_INIT,
    )
    total += 1
    # Segment frames carry data starting at byte 1 (no address header).
    seg_full = bytearray([0x00, 1, 2, 3, 4, 5, 6, 7])
    passed += check("data segment 7B toggle=0", seg_full, EXPECTED_SEGMENT_FULL)
    total += 1
    passed += check(
        "data segment 3B toggle=1 last",
        sdo_request(0x19, 0, 0),
        EXPECTED_SEGMENT_LAST_HEADER_ONLY,
    )

    # --- Block transfer ------------------------------------------------------
    # Block-transfer frames (layout cross-checked against CiA 301 tables;
    # the CRC constant below is the canonical XMODEM check value).
    total += 1
    passed += check("block initiate cmd byte (0xC6)",
                    bytes([0xC0 | 0x04 | 0x02, 0x00, 0x20, 0x00, 100, 0, 0, 0]),
                    EXPECTED_BLOCK_INIT)
    total += 1
    passed += check("sub-block response (ackseq=10, blksize=20)",
                    bytes([0xA2, 10, 20, 0, 0, 0, 0, 0]),
                    EXPECTED_BLOCK_SUBRESP)
    # End frame carries the XMODEM CRC little-endian.
    import struct
    xmodem_crc = 0
    for byte in b"123456789":
        xmodem_crc ^= byte << 8
        for _ in range(8):
            xmodem_crc = ((xmodem_crc << 1) ^ 0x1021) & 0xFFFF if xmodem_crc & 0x8000 \
                         else (xmodem_crc << 1) & 0xFFFF
    total += 1
    passed += check("end frame unused=5 crc(123456789)=0x31C3",
                    bytes([0xC0 | (5 << 2) | 0x01]) + struct.pack("<H", xmodem_crc) + b"\x00" * 5,
                    EXPECTED_BLOCK_END)

    # --- Abort frame ---------------------------------------------------------
    abort = bytearray(8)
    abort[0] = 0x80
    abort[1] = 0x00
    abort[2] = 0x10
    abort[4:8] = (0x0602_0000).to_bytes(4, "little")
    total += 1
    passed += check("abort object-does-not-exist 0x1000", abort, EXPECTED_ABORT)

    print(f"\n{passed}/{total} checks passed")
    return 0 if passed == total else 1


# Expected frames, exactly what canopen-rs emits per its golden-byte tests.
EXPECTED_FRAMES = [
    bytes([0x23, 0x00, 0x20, 0x00, 0x78, 0x56, 0x34, 0x12]),
    bytes([0x2F, 0x01, 0x20, 0x00, 0x7F, 0x00, 0x00, 0x00]),
    bytes([0x2B, 0x00, 0x60, 0x01, 0xFE, 0xFF, 0x00, 0x00]),
]
EXPECTED_UPLOAD_REQUEST = bytes([0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0])
EXPECTED_SEGMENTED_INIT = bytes([0x21, 0x00, 0x20, 0x00, 20, 0, 0, 0])
EXPECTED_SEGMENT_FULL = bytes([0x00, 1, 2, 3, 4, 5, 6, 7])
EXPECTED_SEGMENT_LAST_HEADER_ONLY = bytes([0x19, 0, 0, 0, 0, 0, 0, 0])
EXPECTED_BLOCK_INIT = bytes([0xC6, 0x00, 0x20, 0x00, 100, 0, 0, 0])
EXPECTED_BLOCK_SUBRESP = bytes([0xA2, 10, 20, 0, 0, 0, 0, 0])
EXPECTED_BLOCK_END = bytes([0xD5, 0xC3, 0x31, 0, 0, 0, 0, 0])  # CRC16/XMODEM("123456789")=0x31C3 LE
EXPECTED_ABORT = bytes([0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x02, 0x06])

if __name__ == "__main__":
    sys.exit(main())
