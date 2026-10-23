# canopen-rs

English | [简体中文](README.md)

A production-grade Rust CANopen (CiA 301/305/306) stack for Linux (SocketCAN).

```
┌──────────────────────────────────────────────────────┐
│  cli (canopen binary)                                 │
│    eds dump/gen · read/write · nmt · monitor         │
├──────────────────────────────────────────────────────┤
│  canopen-host (std)                                   │
│    SocketCan (blocking) │ AsyncSocketCan (tokio)      │
│    Network/RemoteNode │ EDS parser │ OD codegen       │
│    Storage (FileStorage) │ Gateway (CiA 309)          │
├──────────────────────────────────────────────────────┤
│  canopen-core (sans-I/O protocol core)                │
│    node │ sdo {server,client,block} │ pdo │ nmt       │
│    lss │ emcy │ leds │ gateway │ storage              │
│    sync_time │ object_dictionary │ standard           │
└──────────────────────────────────────────────────────┘
```

## Features

- **SDO**: expedited / segmented / block transfer (CRC-16/XMODEM, ACK rewind, download-side blksize negotiation up to 127), server-side permission checks at initiate, complete abort-code table.
- **PDO**: sub-byte bit-level mapping and packing, slots assembled from the OD (0x1400–0x1BFF), SYNC acyclic/cyclic plus sync-start-value phase locking, inhibit time / event timer, RTR triggering.
- **NMT**: slave state machine, boot-up/heartbeat producer, node guarding; master heartbeat consumer with a four-state machine (Started/Lost/RemoteReset events carrying the concrete NodeId).
- **EMCY**: idempotent raise/clear, error-register recomputation, bounded history ring shared with 0x1003; consumer with multiple callbacks, panic isolation and active-table tracking.
- **LSS**: selective switch, configure/store/inquire, Fastscan discovery on both sides, AllNodesScanner multi-node enumeration, bit-rate configure/activate; an unconfigured node serves only LSS.
- **EDS**: CiA 306 parser (lazy `$NODEID` resolution) to a runtime OD or build-time code generation; CompactSubObj expansion; EDS retrieval from a device via SDO 0x1021.
- **Storage**: 0x1010/0x1011 parameter persistence (`ParameterStorage` trait plus the `FileStorage` backend).
- **LED**: CiA 303-3 indicator LED state machine (flicker/blink/flash patterns).
- **Gateway**: minimal CiA 309-3 ASCII gateway slice (read/write/NMT/set node).
- **CiA 402**: out of scope (the former `canopen-402` crate was removed by decision).
- **Host**: blocking and tokio async transports, `Network` subscription dispatch (callback isolation plus RX health checks), `RemoteNode` (store/restore, minimal-width `write_int`, explicit-width `write_int_as`, start/stop), EDS parsing and code generation, and CLI tooling.

## Quick start

```bash
cargo build --workspace
sudo tools/vcan_setup.sh          # virtual CAN interface for integration tests

# Master: read the device type, write a parameter, start a node
canopen read vcan0 5 0x1000
canopen write vcan0 5 0x1017 --type u16 500
canopen nmt vcan0 start all
canopen monitor vcan0 -d 30

# EDS -> runtime dictionary or compile-time code generation
canopen eds dump device.eds
canopen eds gen device.eds -o od.rs --fn-name build_od
```

Library usage (master):

```rust
use canopen_core::datatypes::DataType;
use canopen_core::types::{Address, NodeId};
use canopen_host::client::Network;

let net = Network::open("vcan0")?;
let node = net.node(5);
let dt = node.sdo_read(Address::new(0x1000, 0), DataType::Unsigned32)?;
node.write_int(Address::new(0x1017, 0), 500)?;
node.start()?;
net.monitor_heartbeat(NodeId::new(5).expect("node id 5 is valid"), 1500);
for ev in net.heartbeat_events() { /* Started/Lost/... */ }
let eds = net.read_eds_from_device(5)?; // fetch the device EDS via 0x1021
```

Library usage (slave; see the `core/src/node.rs` documentation):

```rust
let mut node = Node::new(node_id, standard_od)?;
if let Some(boot) = node.boot() { bus.send(boot.cob_id, boot.payload())?; }
loop {
    for tx in node.poll(now_ms()) { bus.send(tx.cob_id, tx.payload())?; }
    if let Some(rx) = bus.recv(10ms)? {
        for tx in node.on_frame(rx.cob_id, rx.payload()) { bus.send(..)?; }
    }
}
```

## Testing and quality

```bash
cargo test --workspace                              # unit + integration + pyport regressions (556 tests + 5 doctests; no bus required; 2 vcan tests ignored)
python3 tools/interop/python_canopen_oracle.py      # wire-format oracle against python-canopen (11/11)
cargo test -p canopen-host --test vcan -- --ignored # real vcan0 flow (run sudo tools/vcan_setup.sh first)
cargo clippy --workspace --all-targets -- -D warnings
```

These gates are run locally and in the CI-equivalent environment (the repository currently does not host `.github/` workflows).
The wire format is checked against [python-canopen](https://github.com/christiansandberg/canopen) as an independent oracle.

The hardware cross-validation plan (four suites: our master/slave against CANopenLinux/python-canopen) is in `tests/interop/TESTPLAN.md`; the 2026-10-23 dual-board results are in `tests/interop/RESULTS-2026-10-23.md` and `tests/interop/RESULTS-2026-10-23-fixes.md`.

## Reference implementations

- Architecture survey and references: `docs/REFERENCES.md`
- Implementation spec and status: `docs/IMPLEMENTATION.md`
- Roadmap and extensions: `docs/EXTENSIONS.md`
- Design decisions and excluded findings: `docs/DESIGN-DECISIONS.md`

License: Apache-2.0 OR MIT. See [LICENSE-MIT](LICENSE-MIT) and [LICENSE-APACHE](LICENSE-APACHE).
