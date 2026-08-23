#!/usr/bin/env bash
# Bring up a virtual CAN interface for integration tests.
set -euo pipefail
sudo modprobe vcan
sudo ip link add dev "${1:-vcan0}" type vcan 2>/dev/null || true
sudo ip link set up "${1:-vcan0}"
echo "vcan interface '${1:-vcan0}' is up"
