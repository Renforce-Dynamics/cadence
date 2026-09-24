#!/usr/bin/env bash
# End-to-end MuJoCo sim verification for the SONIC states.
#
# Stage 1 (process level): start the real runtime on the sim entry headless,
# wait for the fixedpos entry gate, drive sonic_clip -> PLAYING -> damping via
# the operator protocol, assert no safety halt. Stage 2 (in-process driver):
# physics-quality metrics for the stand and walk clips (pelvis z band,
# tracking RMSE, no halt).
#
# Requires the sim + inference extras:
#   uv pip install --python .venv/bin/python "mujoco>=3,<4" "onnxruntime>=1.16,<1.24"
#
# Usage: ./scripts/verify_sonic_sim.sh
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${CADENCE_VENV:-$ROOT/.venv}"
PYTHON="$VENV/bin/python"
ENTRY="$ROOT/configs/entry/a3/mock/entry_a3_sonic_sim.yaml"
RUN_DIR="$(mktemp -d "${TMPDIR:-/tmp}/sonic-sim-run.XXXXXX")"
HOST=127.0.0.1
PORT=50560

RUNTIME_PID=""
PASSED=0
FAILED=0
FAILED_STAGES=""

cleanup() {
    [ -n "$RUNTIME_PID" ] && kill "$RUNTIME_PID" 2>/dev/null
    [ -n "$RUNTIME_PID" ] && wait "$RUNTIME_PID" 2>/dev/null
    rm -rf "$RUN_DIR"
}
trap cleanup EXIT INT TERM

stage_pass() { PASSED=$((PASSED + 1)); echo "PASS  $1"; }
stage_fail() { FAILED=$((FAILED + 1)); FAILED_STAGES="$FAILED_STAGES $1"; echo "FAIL  $1"; }

wait_status() {
    "$PYTHON" - "$1" "$2" "$HOST" "$PORT" <<'PYEOF'
import sys, time
from cadence_protocol.client import OperatorClient
expr, timeout, host, port = sys.argv[1], float(sys.argv[2]), sys.argv[3], int(sys.argv[4])
deadline = time.monotonic() + timeout
last = "no reply"
while time.monotonic() < deadline:
    try:
        with OperatorClient(host, port, timeout_s=0.5) as client:
            s = client.status()
        last = {k: s.get(k) for k in ("mode", "substate", "safety_halted", "entry_gate_ready")}
        if s.get("safety_halted"):
            print("safety_halted=True:", s, flush=True)
            sys.exit(3)
        if eval(expr, {"s": s}):
            print("reached:", last, flush=True)
            sys.exit(0)
    except Exception as error:
        last = repr(error)
    time.sleep(0.2)
print("timeout; last status:", last, flush=True)
sys.exit(1)
PYEOF
}

echo "== sonic sim verification =="
if [ ! -x "$PYTHON" ]; then
    echo "environment missing: $VENV; run ./scripts/setup.sh first" >&2
    exit 2
fi
if ! "$PYTHON" -c "import mujoco" 2>/dev/null; then
    echo "mujoco missing: uv pip install --python $PYTHON 'mujoco>=3,<4'" >&2
    exit 2
fi

# --- Stage 1: real runtime process, operator-driven ------------------------
"$PYTHON" -m cadence run --config "$ENTRY" --output "$RUN_DIR" \
    > "$RUN_DIR/stdout.log" 2>&1 &
RUNTIME_PID=$!
sleep 1
if ! kill -0 "$RUNTIME_PID" 2>/dev/null; then
    echo "runtime exited immediately:"; cat "$RUN_DIR/stdout.log"
    stage_fail "sim startup"
    echo "== summary: $PASSED passed, $FAILED failed:$FAILED_STAGES =="
    exit 1
fi
# The sim entry starts in fixedpos (damping falls before requests arrive).
if wait_status "s['mode'] == 'FIXEDPOS' and s.get('entry_gate_ready') is True" 90; then
    stage_pass "sim startup + fixedpos entry gate"
else
    stage_fail "sim startup + fixedpos entry gate"
    tail -20 "$RUN_DIR/stdout.log"
    echo "== summary: $PASSED passed, $FAILED failed:$FAILED_STAGES =="
    exit 1
fi

if "$PYTHON" "$ROOT/scripts/request_state.py" 6 --expect-mode SONIC_CLIP \
    && wait_status "s['mode'] == 'SONIC_CLIP' and s.get('substate') == 'PLAYING'" 20; then
    sleep 10   # let it walk
    if wait_status "s['mode'] == 'SONIC_CLIP' and s.get('substate') == 'PLAYING'" 5; then
        stage_pass "sim sonic_clip RAMP->PLAYING (walk, 10 s)"
    else
        stage_fail "sim sonic_clip RAMP->PLAYING (walk, 10 s)"
    fi
else
    stage_fail "sim sonic_clip RAMP->PLAYING (walk, 10 s)"
fi

if "$PYTHON" "$ROOT/scripts/request_state.py" 1 --expect-mode DAMPING \
    && wait_status "s['mode'] == 'DAMPING' and s['safety_halted'] is False" 15; then
    stage_pass "sim damping, no safety halt"
else
    stage_fail "sim damping, no safety halt"
fi

# --- Stage 2: in-process metrics driver ------------------------------------
if "$PYTHON" "$ROOT/scripts/verify_sonic_sim.py" --config "$ENTRY"; then
    stage_pass "sim physics metrics (stand + walk)"
else
    stage_fail "sim physics metrics (stand + walk)"
fi

echo "== summary: $PASSED passed, $FAILED failed${FAILED_STAGES:+ (failed:$FAILED_STAGES)} =="
[ "$FAILED" = 0 ]
