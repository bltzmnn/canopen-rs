#!/usr/bin/env python3
"""Wire-format oracle: cross-check canopen-rs frames against the real
python-canopen encoders (canopen.sdo.client, v2.x) — BOTH SIDES LIVE.

Each check drives a *real* ``canopen.sdo.client.SdoClient`` whose network
send path is captured in memory, records the request frames python-canopen
actually produces for a known operation, and compares them byte-for-byte
against the frames canopen-rs's codecs emit **right now**, captured by
running ``cargo run -p canopen-core --example oracle_frames`` (see
``core/examples/oracle_frames.rs``). A mismatch means our reading of CiA 301
diverges from a mature independent implementation — the test suite and this
script must then be reconciled.

Because the canopen-rs side is produced by the real codecs at oracle time
(rather than hand-transcribed constants), encoder drift cannot be silently
hand-synced into ``EXPECTED_FRAMES`` anymore (``docs/DESIGN-DECISIONS.md`` TH1). The
CRC-16/XMODEM is additionally cross-checked over several payloads against
``binascii.crc_hqx`` — the same primitive python-canopen's block transfer
uses (``docs/DESIGN-DECISIONS.md`` TH2).

Coverage: SDO expedited downloads (u32/u8/i16), segmented download
(initiate + full segment stream driven through python-canopen's state machine
with canned acknowledgements), block-download initiate, upload request and
the abort layout. NMT, EMCY, SYNC, PDO mapping and LSS frames are covered by
golden-byte tests and the pyport suites instead; python-canopen exposes no
offline encoders for those (docs/IMPLEMENTATION.md §14).

Usage: python3 tools/interop/python_canopen_oracle.py
Exits non-zero on any mismatch.
"""

import binascii
import logging
import struct
import subprocess
import sys
from pathlib import Path

from canopen.sdo import client as py_client

# python-canopen logs aborts via the root logger; keep the report clean.
logging.disable(logging.ERROR)

SDO_STRUCT = py_client.SDO_STRUCT

REPO_ROOT = Path(__file__).resolve().parents[2]


class CapturedSdoClient(py_client.SdoClient):
    """A real SdoClient whose sends are captured instead of going to a bus.

    ``read_response`` feeds back canned acknowledgements so python-canopen's
    genuine state machine streams multi-frame transfers; when the canned
    queue runs dry it raises and the captured frames up to that point are
    still valid for comparison.
    """

    def __init__(self):
        od = __import__("canopen").objectdictionary.ObjectDictionary()
        super().__init__(0x605, 0x585, od)
        self.sent = []
        self.canned = []
        # Fail fast: we only ever want the request bytes.
        self.RESPONSE_TIMEOUT = 0.01

    def send_request(self, request):
        self.sent.append(bytes(request))

    def read_response(self):
        if not self.canned:
            raise py_client.SdoCommunicationError("no canned response")
        return self.canned.pop(0)


def dl_response(index, sub):
    """Download-initiate acknowledgement (SCCS=3) echoing the address."""
    return bytes([0x60]) + SDO_STRUCT.pack(0x60, index, sub)[1:] + b"\x00" * 4


def seg_response(toggle):
    """Download-segment acknowledgement (SCCS=1) carrying the toggle bit."""
    return bytes([0x20 | (0x10 if toggle else 0x00), 0, 0, 0, 0, 0, 0, 0])


def blk_dl_init_response(index, sub, blksize=127, crc=True):
    """Block-download initiate acknowledgement from the server side
    (response command specifier 101 → 0xA0 range; canopen-rs emits 0xA4)."""
    cmd = py_client.RESPONSE_BLOCK_DOWNLOAD | py_client.INITIATE_BLOCK_TRANSFER
    if crc:
        cmd |= py_client.CRC_SUPPORTED
    frame = bytearray(8)
    SDO_STRUCT.pack_into(frame, 0, cmd, index, sub)
    frame[4] = blksize
    return bytes(frame)


def frames(value):
    """Normalize a frame or sequence of frames to a list of bytes."""
    if isinstance(value, (bytes, bytearray)):
        return [bytes(value)]
    return [bytes(f) for f in value]


def check(name, actual, expected):
    a, e = frames(actual), frames(expected)
    ok = a == e
    status = "PASS" if ok else "FAIL"
    print(f"[{status}] {name}")
    if not ok:
        for label, side in (("python", e), ("ours  ", a)):
            print(f"       {label}: {'  '.join(f.hex(' ') for f in side)}")
    return ok


def run_canopen_rs():
    """Runs the canopen-rs live frame dump and parses its output.

    Returns ``(frames, crcs)`` where ``frames`` maps an operation name to a
    list of frame bytes and ``crcs`` maps a payload label to ``(crc, hex)``.
    """
    proc = subprocess.run(
        ["cargo", "run", "-q", "-p", "canopen-core", "--example", "oracle_frames"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=600,
        check=False,
    )
    if proc.returncode != 0:
        print("canopen-rs oracle_frames example failed:", file=sys.stderr)
        print(proc.stderr, file=sys.stderr)
        raise SystemExit(2)
    frames = {}
    crcs = {}
    for line in proc.stdout.splitlines():
        parts = line.split()
        if not parts:
            continue
        name = parts[0]
        if name == "crc":
            # Empty payloads print no hex tail (`crc empty 0000 ` → 3 parts).
            crcs[parts[1]] = (int(parts[2], 16), parts[3] if len(parts) > 3 else "")
        else:
            # Every SDO frame is exactly 8 bytes on the wire; the example
            # prints all frames of an operation back-to-back on one line, so
            # regroup the byte stream into 8-byte frames.
            blob = bytes.fromhex("".join(parts[1:]))
            frames[name] = [blob[i:i + 8] for i in range(0, len(blob), 8)]
    return frames, crcs


def main():
    rs_frames, rs_crcs = run_canopen_rs()
    passed = total = 0

    # --- Expedited downloads (real WritableStream, real command-byte math) --
    for name, index, sub, data, rs_key in [
        ("u32 write 0x12345678 -> 0x2000", 0x2000, 0x00, (0x1234_5678).to_bytes(4, "little"), "u32_write"),
        ("u8 write 0x7F -> 0x2001", 0x2001, 0x00, bytes([0x7F]), "u8_write"),
        ("i16 write -2 -> 0x6000:01", 0x6000, 0x01, (-2 & 0xFFFF).to_bytes(2, "little"), "i16_write"),
    ]:
        c = CapturedSdoClient()
        c.canned.append(dl_response(index, sub))
        stream = py_client.WritableStream(c, index, sub, size=len(data))
        stream.write(data)
        total += 1
        passed += check(name, c.sent[0], rs_frames[rs_key][0])

    # --- Upload request ------------------------------------------------------
    c = CapturedSdoClient()
    try:
        py_client.ReadableStream(c, 0x1000, 0x00).read(4)
    except Exception:  # noqa: BLE001 - no canned response; the request is what matters
        pass
    total += 1
    passed += check("upload request 0x1000", c.sent[0], rs_frames["upload_request"][0])

    # --- Segmented download: initiate + every segment ------------------------
    c = CapturedSdoClient()
    payload = b"A" * 20
    c.canned.append(dl_response(0x2000, 0x00))
    stream = py_client.WritableStream(c, 0x2000, 0x00, size=len(payload))
    i = 0
    while not stream._done:  # noqa: SLF001 - driving the real state machine
        c.canned.append(seg_response(stream._toggle))  # noqa: SLF001
        i += stream.write(payload[i:i + 7])
    total += 1
    passed += check(
        f"segmented download 20B ({len(c.sent)} frames)",
        c.sent,
        rs_frames["segmented_download"],
    )

    # --- Block download initiate ---------------------------------------------
    c = CapturedSdoClient()
    c.canned.append(blk_dl_init_response(0x2000, 0x00))
    py_client.BlockDownloadStream(c, 0x2000, 0x00, size=100)
    total += 1
    passed += check("block initiate cmd byte (0xC6)", c.sent[0], rs_frames["block_initiate"][0])

    # --- Abort frame ----------------------------------------------------------
    # python-canopen's abort() leaves index/subindex zeroed (its own TODO);
    # canopen-rs echoes the address like CANopenNode does. Both are CiA-legal,
    # so compare the command byte and the 32-bit abort code only.
    c = CapturedSdoClient()
    c.abort(0x0602_0000)
    rs_abort = rs_frames["abort"][0]
    total += 1
    passed += check(
        "abort object-does-not-exist (cmd+code, address is impl-defined)",
        [c.sent[0][0]] + list(c.sent[0][4:8]),
        [rs_abort[0]] + list(rs_abort[4:8]),
    )

    # --- CRC-16/XMODEM: live canopen-rs values vs binascii.crc_hqx -----------
    for label, (crc_value, payload_hex) in sorted(rs_crcs.items()):
        total += 1
        ours = binascii.crc_hqx(bytes.fromhex(payload_hex), 0)
        ok = ours == crc_value
        print(
            f"[{'PASS' if ok else 'FAIL'}] crc16/xmodem({label!r}) "
            f"canopen-rs={crc_value:#06x} binascii={ours:#06x}"
        )
        passed += 1 if ok else 0

    print(f"\n{passed}/{total} checks passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
