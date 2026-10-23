#!/usr/bin/env bash
# End-to-end test of the Ethernet path without the kit (Linux, root for TAP):
# kit_sim (bridge firmware on a TAP device + host/fpga_emu.py as the fabric)
# <- ping, h264_send.py -p udp://192.168.77.2 with packet loss in both directions.
#
#   test_udp.sh <stream.264> [loss ...]      default losses: 0 0.01 0.05
set -uo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
SRC=${1:?stream.264}
shift
LOSSES=("$@")
[ ${#LOSSES[@]} -eq 0 ] && LOSSES=(0 0.01 0.05)
TMP=$(mktemp -d)
trap 'pkill -TERM -x kit_sim 2>/dev/null; rm -rf "$TMP"' EXIT

make -s -C "$HERE" || exit 1

ping_kit() {
    python3 - "$1" <<'EOF'
import os, socket, struct, sys
def csum(b):
    b += b"\0" * (len(b) & 1)
    s = sum(struct.unpack("!%dH" % (len(b) // 2), b))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return ~s & 0xFFFF
s = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
s.settimeout(1)
ok = 0
for seq in range(3):
    pay = os.urandom(56)
    hdr = struct.pack("!BBHHH", 8, 0, 0, 0x4864, seq)
    s.sendto(struct.pack("!BBHHH", 8, 0, csum(hdr + pay), 0x4864, seq) + pay, (sys.argv[1], 0))
    try:
        while True:
            d, _ = s.recvfrom(2048)
            ic = d[(d[0] & 15) * 4:]
            if ic[0] == 0 and struct.unpack("!HH", ic[4:8]) == (0x4864, seq):
                ok += ic[8:] == pay and csum(ic) == 0
                break
    except socket.timeout:
        pass
print("ping %d/3" % ok)
sys.exit(0 if ok == 3 else 1)
EOF
}

fail=0
for loss in "${LOSSES[@]}"; do
    rm -f "$TMP/rx.264"
    KIT_LOSS=$loss KIT_EMU_ARGS="--out $TMP/rx.264" "$HERE/kit_sim" > "$TMP/kit.log" 2>&1 &
    kit=$!
    sleep 0.7
    res="OK"
    if [ "$loss" = 0 ]; then ping_kit 192.168.77.2 > "$TMP/ping.txt" || res="PING FAILED"; fi
    python3 "$ROOT/host/h264_send.py" -p udp://192.168.77.2 "$SRC" > "$TMP/send.txt" 2>&1 || res="SEND FAILED"
    kill -TERM $kit; wait $kit 2>/dev/null
    cmp -s "$TMP/rx.264" "$SRC" || res="${res/OK/}DATA MISMATCH"
    [ "$res" = OK ] || fail=$((fail + 1))
    echo "loss $loss: $res | $(tr '\r' '\n' < "$TMP/send.txt" | grep -o '[0-9.]* [KMG]*B in .*timeouts [0-9]*') $(cat "$TMP/ping.txt" 2>/dev/null)"
    rm -f "$TMP/ping.txt"
done
[ "$fail" -eq 0 ] && echo "all passed"
[ "$fail" -eq 0 ]
