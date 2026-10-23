#!/usr/bin/env bash
# Open-source resource / logic depth estimate for PolarFire (no Libero needed).
#
#   synth_estimate.sh [module ...]     default: all decoder modules
#
# GHDL (--synth) converts every module to Verilog, yosys `synth_microchip`
# maps it to PolarFire cells (CFG1..4 LUTs, ARI1 carry, SLE flip-flops,
# LSRAM / uSRAM) and pf_depth.py reports the longest register-to-register
# path in LUT levels (carry chain counted as 0.1 level per bit).
# Results go to ./sim/synth/<module>.{stat,depth}.  Paths that cross module
# boundaries are not seen; the final numbers come from Libero (Synplify Pro
# + SmartTime).
#
# Needs ghdl >= 4 with the synthesis feature and yosys >= 0.40
# (YOSYS=yowasp-yosys works too).
set -uo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
OUT=${OUT:-$ROOT/sim/synth}
YOSYS=${YOSYS:-yosys}
mkdir -p "$OUT"
cd "$OUT" || exit 1

RTL="h264_pkg h264_tables_pkg h264_ram h264_nal h264_bitreader h264_hdr h264_mbparse h264_mc h264_recon h264_deblock h264_dec_top h264_uart h264_link h264_dec_uart_top"
rm -f work-obj08.cf
for f in $RTL; do ghdl -a --std=08 -frelaxed "$ROOT/rtl/$f.vhd" || exit 1; done

mods=("$@")
if [ ${#mods[@]} -eq 0 ]; then
    mods=(h264_nal h264_bitreader h264_hdr h264_mbparse h264_mc h264_recon h264_deblock h264_coefbuf h264_predbuf)
fi

for m in "${mods[@]}"; do
    ghdl --synth --std=08 -frelaxed --out=verilog "$m" > "$m.v" 2> "$m.ghdl.log" || { echo "$m: ghdl --synth failed"; continue; }
    top=$(grep -o "^module [a-zA-Z0-9_]*" "$m.v" | tail -1 | cut -d' ' -f2)
    cat > "$m.ys" <<EOT
read_verilog $m.v
proc
setattr -unset init
synth_microchip -top $top -family polarfire -noiopad -noclkbuf
tee -o $m.stat stat
write_json $m.json
EOT
    "$YOSYS" -q "$m.ys" > "$m.yosys.log" 2>&1 || { echo "$m: yosys failed (see $OUT/$m.yosys.log)"; continue; }
    python3 "$HERE/pf_depth.py" "$m.json" --paths 5 > "$m.depth"
    cells=$(grep -E "^\s+[0-9]+\s+(CFG[1-4]|ARI1|SLE|RAM1K20|RAM64x12|MACC_PA)$" "$m.stat" |
            awk '{printf "%s=%s ", $2, $1}')
    echo "$m: $cells| $(head -1 "$m.depth" | sed 's/.*max depth //')"
done
