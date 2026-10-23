#!/usr/bin/env bash
# Whole chain co-simulation (Linux, root for the TAP device):
#
#   PC program (host/h264_send.py) --UDP--> kit firmware (kit_cosim: the real
#   network / bridge / fabric driver code) --APB--> h264_dec_hdmi_top in GHDL
#   (tb/tb_cosim.vhd: link, decoder, DDR model, display, TMDS) --> frames
#   decoded back from the TMDS lanes --> scripts/disp_check.py vs reference
#
#   cosim.sh <stream_dir> <name>        e.g. cosim.sh streams qcif_ip
#
# <name>.264 / .ref.yuv / .info come from scripts/gen_streams.sh.  The RTL
# simulation is thousands of times slower than the hardware, so the PC
# program runs with long time-outs (H264_RTO, WAIT_S for the decoder after
# the last byte, ~100 s per CIF picture here) and the display at 60 fps.
set -uo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
SDIR=$(cd "${1:?stream directory}" && pwd)
NAME=${2:?stream name}
WORK=${WORK:-$ROOT/sim/cwork}
OUT=${OUT:-$ROOT/sim/cosim}
KIT=192.168.77.2
mkdir -p "$WORK" "$OUT"
read -r w h frames < "$SDIR/$NAME.info"
cw=$(( (w + 15) / 16 * 16 )); ch=$(( (h + 15) / 16 * 16 ))

make -s -C "$HERE" cosim || exit 1
GF="--std=08 -frelaxed --workdir=$WORK"
for f in h264_pkg h264_tables_pkg h264_ram h264_nal h264_bitreader h264_hdr h264_mbparse h264_mc \
         h264_recon h264_deblock h264_dec_top h264_uart h264_link h264_apb_bridge h264_dec_mss_top \
         h264_tmds h264_axi_rd_arb h264_disp h264_dec_hdmi_top; do
    ghdl -a $GF "$ROOT/rtl/$f.vhd" || exit 1
done
ghdl -a $GF "$ROOT/tb/tb_util_pkg.vhd" "$ROOT/tb/tb_cosim.vhd" && ghdl -e $GF tb_cosim || exit 1

rm -f "$OUT/$NAME.rgb"
export LD_LIBRARY_PATH="$HERE${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export KIT_COSIM_CMD="ghdl -r $GF tb_cosim --ieee-asserts=disable -gG_OUT=$OUT/$NAME.rgb -gG_MAXW=$cw -gG_MAXH=$ch"
"$HERE/kit_cosim" > "$OUT/$NAME.kit.log" 2>&1 &
kit=$!
trap 'kill -TERM $kit 2>/dev/null' EXIT
sleep 1
# the RTL simulation answers in seconds, not microseconds: without a long
# time-out the PC program floods it with retransmissions
export H264_RTO=${H264_RTO:-120}
SEND="python3 $ROOT/host/h264_send.py --ip $KIT"

echo "== kit firmware up, fabric (RTL) behind APB:"
for i in $(seq 1 30); do
    $SEND --info > "$OUT/$NAME.info.txt" 2>&1 && grep -q "fabric.*ok" "$OUT/$NAME.info.txt" && break
    sleep 2
done
grep -E "^(fw|ip|fabric|link) " "$OUT/$NAME.info.txt"
echo "== sending $NAME.264 ($w x $h, $frames frames)"
t0=$(date +%s)
$SEND --fps 60 --wait-timeout ${WAIT_S:-7200} "$SDIR/$NAME.264" 2>&1 | tr '\r' '\n' | grep -v "^$" | grep -v "sent " | tail -4
echo "== kit status after the stream ($(( $(date +%s) - t0 )) s):"
$SEND --info 2>&1 | grep -E "^(udp_in|udp_out|udp_cut|eth_drop|pictures|width|height|dec_error|dec_done|disp_frames|disp_late_lines) "
sleep ${TAIL_S:-60}             # let the display show the last frame
kill -TERM $kit; wait $kit 2>/dev/null
trap - EXIT
grep -h "co-simulation:" "$OUT/$NAME.kit.log"
echo "== kit console (last lines):"
grep "^\[" "$OUT/$NAME.kit.log" | tail -6
python3 "$ROOT/scripts/disp_check.py" "$OUT/$NAME.rgb" "$SDIR/$NAME.ref.yuv" "$w" "$h" 256 192
