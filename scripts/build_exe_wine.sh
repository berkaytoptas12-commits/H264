#!/usr/bin/env bash
# Windows executables of the PC programs, built on Linux without Windows:
# Wine + a win-64 Python from conda-forge (micromamba) + PyInstaller.
#
#   scripts/build_exe_wine.sh [out_dir]     -> h264_gui.exe (window), h264_send.exe (console)
#
# Needs wine64 (apt install wine64), xvfb-run, curl and access to
# conda.anaconda.org.  On Windows itself use host/build_exe.bat instead.
# The executables need 64-bit Windows 10 or 11 (the C runtime ships with it).
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
OUT=$(mkdir -p "${1:-$ROOT/host/dist}" && cd "${1:-$ROOT/host/dist}" && pwd)
B=${BUILD_DIR:-$HOME/.cache/h264_exe}
MM=$B/micromamba
ENV=$B/winenv
export MAMBA_ROOT_PREFIX=$B/mamba WINEPREFIX=$B/wine WINEDEBUG=-all
export WINEDLLOVERRIDES="mscoree,mshtml=;winedbg.exe=d"
mkdir -p "$B"

if [ ! -x "$MM" ]; then
    curl -sSL https://conda.anaconda.org/conda-forge/linux-64/micromamba-2.0.5-0.tar.bz2 \
        | tar xjf - -O bin/micromamba > "$MM"
    chmod +x "$MM"
fi
if [ ! -x "$ENV/python.exe" ]; then
    "$MM" create -y -q -p "$ENV" --platform win-64 -c conda-forge \
        python=3.11 tk=8.6 pyinstaller=6.22 pyserial=3.5
    # installing for another platform leaves three things to fix by hand:
    # 1. noarch packages land in the Linux layout
    cp -r "$ENV"/lib/python3.11/site-packages/. "$ENV/Lib/site-packages/"
    # 2. files with an embedded build prefix get the (longer) Linux path
    #    patched in, which breaks binaries (_tkinter.pyd): take them unpatched
    for hp in "$MAMBA_ROOT_PREFIX"/pkgs/*/info/has_prefix; do
        pkg=$(dirname "$(dirname "$hp")")
        awk '$(NF-1) == "binary" { f = $NF; gsub(/["\r]/, "", f); print f }' "$hp" | while read -r f; do
            if [ -f "$pkg/$f" ] && [ -f "$ENV/$f" ]; then cp "$pkg/$f" "$ENV/$f"; fi
        done
    done
    # 3. conda's copy of the Windows C runtime breaks Wine; Wine has its own
    rm -f "$ENV"/api-ms-win-*.dll "$ENV"/ucrtbase.dll "$ENV"/Library/bin/api-ms-win-*.dll "$ENV"/Library/bin/ucrtbase.dll
fi
[ -d "$WINEPREFIX" ] || xvfb-run -a wineboot -i > /dev/null 2>&1
WENV=$(winepath -w "$ENV")
export WINEPATH="$WENV\\Library\\bin;$WENV"

build() {   # name, --windowed / --console
    # Wine cannot hand Python a regular file as stdout: keep stdin and stdout pipes
    (cd "$B" && xvfb-run -a wine "$ENV/python.exe" -m PyInstaller --noconfirm --clean --onefile "$2" \
        --name "$1" --distpath "$OUT" --workpath work --specpath spec "$ROOT/host/$1.py" < /dev/null 2>&1 \
        | tee "$B/$1.log" | grep -E "WARNING|ERROR|completed successfully" | grep -v "list_ports_osx" || true)
    [ -f "$OUT/$1.exe" ] || { echo "$1.exe not built" >&2; exit 1; }
    if grep -q "tkinter installation is broken" "$B/$1.log"; then
        echo "$1.exe built without Tkinter: the win-64 Python's tkinter does not load under Wine" >&2
        exit 1
    fi
}
rm -f "$OUT/h264_gui.exe" "$OUT/h264_send.exe"
build h264_gui --windowed
build h264_send --console
ls -l "$OUT"/h264_gui.exe "$OUT"/h264_send.exe
