#!/usr/bin/env bash
# End-to-end mock-backend verification of the SONIC states.
#
# Stages: runtime startup -> operator readiness -> fixedpos entry gate ->
# sonic_clip RAMP/PLAYING/DONE -> sonic_stream WAITING/TRACKING/LOST/WAITING ->
# damping. safety_halted must stay false throughout. Exits nonzero if any
# stage fails. Compatible with macOS bash 3.2 and Linux.
#
# Usage: ./scripts/verify_sonic_mock.sh
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${CADENCE_VENV:-$ROOT/.venv}"
PYTHON="$VENV/bin/python"
ENTRY="$ROOT/configs/entry/a3/mock/entry_a3_operator_sonic.yaml"
CLIP_NPZ="$ROOT/data/motions/BMD_0319_stand.npz"
RUN_DIR="$(mktemp -d "${TMPDIR:-/tmp}/sonic-mock-run.XXXXXX")"
HOST=127.0.0.1
PORT=50560

RUNTIME_PID=""
PRODUCER_PID=""
PASSED=0
FAILED=0
FAILED_STAGES=""

cleanup() {
    [ -n "$PRODUCER_PID" ] && kill "$PRODUCER_PID" 2>/dev/null
    [ -n "$RUNTIME_PID" ] && kill "$RUNTIME_PID" 2>/dev/null
    [ -n "$PRODUCER_PID" ] && wait "$PRODUCER_PID" 2>/dev/null
    [ -n "$RUNTIME_PID" ] && wait "$RUNTIME_PID" 2>/dev/null
    rm -rf "$RUN_DIR"
}
trap cleanup EXIT INT TERM

stage_pass() { PASSED=$((PASSED + 1)); echo "PASS  $1"; }
stage_fail() { FAILED=$((FAILED + 1)); FAILED_STAGES="$FAILED_STAGES $1"; echo "FAIL  $1"; }

# wait_status <python predicate on status dict s> <timeout_s>
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

echo "== sonic mock verification =="
echo "runtime entry: $ENTRY"

if [ ! -x "$PYTHON" ]; then
    echo "environment missing: $VENV; run ./scripts/setup.sh first" >&2
    exit 2
fi

# --- Stage 1: start the runtime -------------------------------------------
"$PYTHON" -m cadence run --config "$ENTRY" --output "$RUN_DIR" \
    > "$RUN_DIR/stdout.log" 2>&1 &
RUNTIME_PID=$!
sleep 1
if ! kill -0 "$RUNTIME_PID" 2>/dev/null; then
    echo "runtime exited immediately:"; cat "$RUN_DIR/stdout.log"
    stage_fail "startup"
    echo "== summary: $PASSED passed, $FAILED failed:$FAILED_STAGES =="
    exit 1
fi
if wait_status "True" 90; then
    stage_pass "startup + operator readiness"
else
    stage_fail "startup + operator readiness"
    tail -20 "$RUN_DIR/stdout.log"
    echo "== summary: $PASSED passed, $FAILED failed:$FAILED_STAGES =="
    exit 1
fi

# --- Stage 2: fixedpos and entry gate --------------------------------------
if "$PYTHON" "$ROOT/scripts/request_state.py" 2 --expect-mode FIXEDPOS \
    && wait_status "s['mode'] == 'FIXEDPOS' and s.get('entry_gate_ready') is True" 30; then
    stage_pass "fixedpos entry gate"
else
    stage_fail "fixedpos entry gate"
fi

# --- Stage 3: sonic_clip RAMP -> PLAYING -> DONE ---------------------------
CLIP_OK=1
if ! "$PYTHON" "$ROOT/scripts/request_state.py" 6 --expect-mode SONIC_CLIP; then
    CLIP_OK=0
elif ! wait_status "s['mode'] == 'SONIC_CLIP' and s.get('substate') == 'PLAYING'" 20; then
    CLIP_OK=0
elif ! wait_status "s['mode'] == 'SONIC_CLIP' and s.get('substate') == 'DONE'" 75; then
    CLIP_OK=0
fi
if [ "$CLIP_OK" = 1 ]; then stage_pass "sonic_clip RAMP->PLAYING->DONE"; else stage_fail "sonic_clip RAMP->PLAYING->DONE"; fi

# --- Stage 4: sonic_stream WAITING -> TRACKING -> LOST -> WAITING ----------
STREAM_OK=1
if ! "$PYTHON" "$ROOT/scripts/request_state.py" 7 --expect-mode SONIC_STREAM; then
    STREAM_OK=0
elif ! wait_status "s['mode'] == 'SONIC_STREAM' and s.get('substate') == 'WAITING'" 15; then
    STREAM_OK=0
else
    "$PYTHON" "$ROOT/scripts/send-motion-ref.py" "$CLIP_NPZ" --loop > "$RUN_DIR/producer.log" 2>&1 &
    PRODUCER_PID=$!
    if ! wait_status "s['mode'] == 'SONIC_STREAM' and s.get('substate') == 'TRACKING'" 20; then
        STREAM_OK=0
    else
        kill "$PRODUCER_PID" 2>/dev/null
        wait "$PRODUCER_PID" 2>/dev/null
        PRODUCER_PID=""
        if ! wait_status "s.get('substate') == 'LOST'" 15; then
            STREAM_OK=0
        elif ! wait_status "s.get('substate') == 'WAITING'" 15; then
            STREAM_OK=0
        fi
    fi
fi
if [ -n "$PRODUCER_PID" ]; then kill "$PRODUCER_PID" 2>/dev/null; wait "$PRODUCER_PID" 2>/dev/null; PRODUCER_PID=""; fi
if [ "$STREAM_OK" = 1 ]; then stage_pass "sonic_stream WAITING->TRACKING->LOST->WAITING"; else stage_fail "sonic_stream WAITING->TRACKING->LOST->WAITING"; tail -5 "$RUN_DIR/producer.log" 2>/dev/null; fi

# --- Stage 5: damping and clean stop ---------------------------------------
if "$PYTHON" "$ROOT/scripts/request_state.py" 1 --expect-mode DAMPING \
    && wait_status "s['mode'] == 'DAMPING'" 15; then
    stage_pass "damping"
else
    stage_fail "damping"
fi

if wait_status "s['safety_halted'] is False" 2; then
    stage_pass "no safety halt"
else
    stage_fail "no safety halt"
fi

echo "== summary: $PASSED passed, $FAILED failed${FAILED_STAGES:+ (failed:$FAILED_STAGES)} =="
tail -3 "$RUN_DIR/stdout.log"
[ "$FAILED" = 0 ]
