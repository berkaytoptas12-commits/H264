#!/usr/bin/env bash
# Generates Constrained Baseline test streams with x264 (through ffmpeg) and
# the matching ffmpeg-decoded reference YUV files.
#
#   gen_streams.sh <out_dir> [quick]
#
# Each stream <name>.264 gets <name>.ref.yuv (cropped, decode order == output
# order for these streams) and <name>.info (width height frames).
set -euo pipefail

OUT=${1:?output directory}
MODE=${2:-full}
mkdir -p "$OUT"

FF="ffmpeg -hide_banner -loglevel error -y"
BASE="profile=baseline:bframes=0:cabac=0:8x8dct=0:weightp=0"

# name | size | frames | source filter | extra x264 params
gen() {
    local name=$1 size=$2 frames=$3 src=$4 params=$5
    $FF -f lavfi -i "$src" -frames:v "$frames" -s "$size" -pix_fmt yuv420p \
        -c:v libx264 -profile:v baseline -x264-params "$BASE:$params" \
        -f h264 "$OUT/$name.264"
    $FF -i "$OUT/$name.264" -f rawvideo -pix_fmt yuv420p "$OUT/$name.ref.yuv"
    local w=${size%x*} h=${size#*x}
    echo "$w $h $frames" > "$OUT/$name.info"
}

MOV="testsrc2=size=640x480:rate=25,crop=352:288:'40+30*sin(t*3)':'40+20*cos(t*2)'"
MOVQ="testsrc2=size=320x240:rate=25,crop=176:144:'30+25*sin(t*4)':'20+20*cos(t*3)'"
NOISE="testsrc2=size=176x144:rate=25,noise=alls=40:allf=t"

gen qcif_intra   176x144  3  "$MOVQ"  "keyint=1:qp=26"
gen qcif_ip      176x144  8  "$MOVQ"  "keyint=30:ref=1"
gen qcif_ref3    176x144  10 "$MOVQ"  "keyint=30:ref=3:partitions=all:subme=7"
gen qcif_noise   176x144  6  "$NOISE" "keyint=30:ref=2:partitions=all:qp=20"
gen qcif_slices  176x144  6  "$MOVQ"  "keyint=30:slices=4:ref=2"
gen qcif_nodb    176x144  6  "$MOVQ"  "keyint=30:no-deblock=1"
gen qcif_dbofs   176x144  6  "$MOVQ"  "keyint=30:deblock=3,-2"
gen qcif_dbneg   176x144  6  "$MOVQ"  "keyint=30:deblock=-4,4"
gen qcif_cip     176x144  8  "$MOVQ"  "keyint=30:constrained-intra=1:intra-refresh=1:ref=1"
gen qcif_qp1     176x144  3  "$MOVQ"  "keyint=30:qp=1"
gen qcif_qp51    176x144  4  "$MOVQ"  "keyint=30:qp=51"
gen qcif_qp10    176x144  4  "$NOISE" "keyint=30:qp=10:partitions=all"
gen qcif_cqp     176x144  6  "$MOVQ"  "keyint=30:chroma-qp-offset=7:aq-mode=2"
gen qcif_cqpn    176x144  6  "$MOVQ"  "keyint=30:chroma-qp-offset=-9:aq-mode=1:aq-strength=2"
gen odd_200x120  200x120  6  "$MOVQ"  "keyint=30:ref=2"
gen odd_100x60   100x60   6  "$MOVQ"  "keyint=30:ref=2:partitions=all"
gen qcif_slmax   176x144  6  "$NOISE" "keyint=30:slice-max-size=300:ref=2"
if [ "$MODE" != quick ]; then
gen cif_ref4     352x288  10 "$MOV"   "keyint=30:ref=4:partitions=all:me=umh:merange=32"
gen cif_bigmv    352x288  8  "testsrc2=size=1280x720:rate=25,crop=352:288:'200+180*sin(t*5)':'100+90*cos(t*5)'" "keyint=60:ref=2:me=esa:merange=64"
gen cif_gop      352x288  20 "$MOV"   "keyint=5:min-keyint=5:ref=3:scenecut=0"
gen hd720        1280x720 3  "testsrc2=size=1280x720:rate=25" "keyint=30:ref=2"
gen hd1080       1920x1080 2 "testsrc2=size=1920x1080:rate=25" "keyint=30:ref=1"
fi
ls "$OUT"/*.264 | wc -l
