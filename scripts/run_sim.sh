#!/usr/bin/env bash
# Runs the RTL testbenches in GHDL and checks the output bit-exactly.
#
#   run_sim.sh top   <stream_dir> [name ...]   full decoder (tb_top) vs <name>.ref.yuv
#   run_sim.sh parse <stream_dir> [name ...]   parser only (tb_parse) vs C model trace
#   run_sim.sh model <stream_dir> [name ...]   C reference model vs <name>.ref.yuv
#   run_sim.sh link  <stream_dir> [name ...]   PC link (tb_link): frames from host/h264link.py,
#                                              errors, out of order, repeats, credit waits
#   run_sim.sh top-uart <stream_dir> [name ...]  tb_top with the stream sent over the PC link
#   run_sim.sh apb   <stream_dir> [name ...]   MSS path (tb_apb): link frames written through
#                                              h264_apb_bridge as the firmware does
#
# Streams come from gen_streams.sh (<name>.264, <name>.ref.yuv, <name>.info).
# Without names every stream in <stream_dir> is run.
# Environment: STALL=0|1 (random AXI back-pressure, default 1), TIMEOUT_US,
#              WORK (GHDL work directory, default ./sim/work),
#              IEEE_ASSERTS=disable|enable (numeric_std warnings, default disable)
set -uo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
MODE=${1:?mode: top | parse}
SDIR=$(cd "${2:?stream directory}" && pwd)
shift 2
WORK=${WORK:-$ROOT/sim/work}
OUT=${OUT:-$ROOT/sim/out}
mkdir -p "$WORK" "$OUT"
GHDL="ghdl"
# numeric_std warnings come from pipeline registers that are evaluated
# before they carry valid data; IEEE_ASSERTS=enable shows them
IEEE_ASSERTS=${IEEE_ASSERTS:-disable}
GFLAGS="--std=08 -frelaxed --workdir=$WORK"

MODEL="$WORK/h264_model"
gcc -O2 -o "$MODEL" "$ROOT/model/h264_model.c" || exit 1

RTL="h264_pkg h264_tables_pkg h264_ram h264_nal h264_bitreader h264_hdr h264_mbparse h264_mc h264_recon h264_deblock h264_dec_top h264_uart h264_link h264_dec_uart_top h264_apb_bridge h264_dec_mss_top"
for f in $RTL; do $GHDL -a $GFLAGS "$ROOT/rtl/$f.vhd" || exit 1; done
for f in tb_util_pkg tb_parse tb_top tb_link tb_apb; do $GHDL -a $GFLAGS "$ROOT/tb/$f.vhd" || exit 1; done

case "$MODE" in
    parse) $GHDL -e $GFLAGS tb_parse || exit 1 ;;
    top|top-uart) $GHDL -e $GFLAGS tb_top || exit 1 ;;
    link) $GHDL -e $GFLAGS tb_link || exit 1 ;;
    apb) $GHDL -e $GFLAGS tb_apb || exit 1 ;;
esac
VEC="python3 $ROOT/host/make_vectors.py"

names=("$@")
if [ ${#names[@]} -eq 0 ]; then
    for f in "$SDIR"/*.264; do names+=("$(basename "$f" .264)"); done
fi

pass=0; fail=0
for n in "${names[@]}"; do
    read -r w h frames < "$SDIR/$n.info"
    t0=$(date +%s)
    if [ "$MODE" = model ]; then
        "$MODEL" "$SDIR/$n.264" "$OUT/$n.model.yuv" > "$OUT/$n.log" 2>&1
        res=$(python3 "$HERE/yuvcmp.py" "$OUT/$n.model.yuv" "$SDIR/$n.ref.yuv" "$w" "$h")
    elif [ "$MODE" = parse ]; then
        "$MODEL" "$SDIR/$n.264" "$OUT/$n.model.yuv" -p "$OUT/$n.model_trace.txt" 2>/dev/null
        $GHDL -r $GFLAGS tb_parse --ieee-asserts=$IEEE_ASSERTS -gG_IN="$SDIR/$n.264" -gG_TRACE="$OUT/$n.rtl_trace.txt" \
            -gG_TIMEOUT_US="${TIMEOUT_US:-5000000}" > "$OUT/$n.log" 2>&1
        if cmp -s "$OUT/$n.model_trace.txt" "$OUT/$n.rtl_trace.txt"; then res="OK"; else res="TRACE MISMATCH"; fi
    elif [ "$MODE" = link ] || [ "$MODE" = apb ]; then
        $VEC "$SDIR/$n.264" "$OUT/$n.frames.txt" > /dev/null
        $GHDL -r $GFLAGS tb_$MODE --ieee-asserts=$IEEE_ASSERTS -gG_IN="$SDIR/$n.264" -gG_FRAMES="$OUT/$n.frames.txt" \
            -gG_RESP="$OUT/$n.resp.bin" > "$OUT/$n.log" 2>&1
        if grep -q "test passed" "$OUT/$n.log" && $VEC --check "$OUT/$n.resp.bin" > "$OUT/$n.resp.txt"; then
            res="OK $(grep -o '[a-zA-Z ]*test passed.*' "$OUT/$n.log")"
        else
            res="FAIL (see $OUT/$n.log)"
        fi
    else
        extra=()
        if [ "$MODE" = top-uart ]; then
            $VEC "$SDIR/$n.264" "$OUT/$n.frames.txt" > /dev/null
            extra=(-gG_UART=1 -gG_FRAMES="$OUT/$n.frames.txt" -gG_PERF=0)
        fi
        cw=$(( (w + 15) / 16 * 16 )); ch=$(( (h + 15) / 16 * 16 ))
        $GHDL -r $GFLAGS tb_top --ieee-asserts=$IEEE_ASSERTS -gG_IN="$SDIR/$n.264" -gG_OUT="$OUT/$n.rtl.yuv" -gG_MAXW=$cw -gG_MAXH=$ch \
            -gG_STALL="${STALL:-1}" -gG_TIMEOUT_US="${TIMEOUT_US:-5000000}" "${extra[@]}" > "$OUT/$n.log" 2>&1
        res=$(python3 "$HERE/yuvcmp.py" "$OUT/$n.rtl.yuv" "$SDIR/$n.ref.yuv" "$w" "$h")
    fi
    t1=$(date +%s)
    case "$res" in OK*) pass=$((pass + 1));; *) fail=$((fail + 1));; esac
    echo "$n: $res [$((t1 - t0)) s] $(grep -o 'stream done.*' "$OUT/$n.log")"
done
echo "PASS $pass FAIL $fail"
[ "$fail" -eq 0 ]
