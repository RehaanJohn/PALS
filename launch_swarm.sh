#!/usr/bin/env bash
set -euo pipefail
PALS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$PALS_ROOT/scripts/swarm.py" "$@"
