#!/usr/bin/env bash
#
# build.sh — compile the HealthyPi 5 NEXT Arduino examples with arduino-cli.
# ============================================================================
# Compiles against the in-repo HealthyPi5 library (repo root: src/ + library.properties) plus whatever
# sensor libraries are installed in your global Arduino libraries folder.
#
#   ./extras/scripts/build.sh                 # build all examples
#   ./extras/scripts/build.sh next            # only HealthyPi5_NEXT  (FreeRTOS spine)
#   ./extras/scripts/build.sh raw             # only RawProcessing  (DSP in loop())
#   ./extras/scripts/build.sh openview        # only 08_OpenView_Stream (single-core tutorials)
#   ./extras/scripts/build.sh display         # only 12_Display_Vitals (ILI9488; needs Arduino_GFX)
#   ./extras/scripts/build.sh tutorials        # all Tutorials/ Serial-Plotter sketches
#   ./extras/scripts/build.sh ecg|resp|ppg|spo2|hr|temp|vitals|wireless  # one Tutorials sketch
#   ./extras/scripts/build.sh next --upload   # build + upload (set PORT=/dev/cu.usbmodemXXXX)
#   CLEAN=1 ./extras/scripts/build.sh         # wipe build cache first
#
# Env overrides:
#   FQBN_BASE   board FQBN without options   (default rp2040:rp2040:rpipico)
#   FQBN_EXTRA  extra FQBN menu options appended as ",<opts>" (e.g.
#               uploadmethod=picoprobe_cmsis_dap) — used by upload.sh
#   PORT        serial port for --upload
#   BUILD_DIR   where compiled artifacts go  (default build/)
# ============================================================================
set -euo pipefail

# --- locate the repo root (this script lives in <root>/extras/scripts) -------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT"

FQBN_BASE="${FQBN_BASE:-rp2040:rp2040:rpipico}"
BUILD_DIR="${BUILD_DIR:-$ROOT/build}"
# The repo root IS the HealthyPi5 library (1.5 format: library.properties + src/
# at the root), so we point arduino-cli straight at it with --library.
LIBRARY="$ROOT"

# --- preflight --------------------------------------------------------------
if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "ERROR: arduino-cli not found in PATH." >&2
  echo "       Install it, then run extras/scripts/install-core.sh (or see the README)." >&2
  exit 1
fi

if ! arduino-cli core list 2>/dev/null | grep -qi 'rp2040:rp2040'; then
  echo "ERROR: the arduino-pico core (rp2040:rp2040) is not installed." >&2
  echo "       arduino-cli core install rp2040:rp2040" >&2
  echo "       (add the board URL first — see extras/scripts/install-core.sh)." >&2
  exit 1
fi

# --- args -------------------------------------------------------------------
TARGET="all"
UPLOAD=0
for arg in "$@"; do
  case "$arg" in
    next|raw|openview|datalog|display|all) TARGET="$arg" ;;
    ecg|ppg|spo2|resp|hr|temp|vitals|wireless|tutorials) TARGET="$arg" ;;
    --upload|-u)                              UPLOAD=1 ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done

if [[ "${CLEAN:-0}" == "1" ]]; then
  echo ">> CLEAN: removing $BUILD_DIR"
  rm -rf "$BUILD_DIR"
fi

# --- one example = one (sketch dir, FQBN) pair ------------------------------
# HealthyPi5_NEXT needs the FreeRTOS-SMP variant (os=freertos); the single-core
# tutorials sketch builds against the stock variant.
build_one() {
  local name="$1" sketch="$2" fqbn="$3" cppflags="${4:-}"
  local out="$BUILD_DIR/$name"
  # Append any caller-supplied FQBN menu options (e.g. upload.sh adds
  # uploadmethod=picoprobe_cmsis_dap so build + upload use one identical FQBN).
  fqbn="${fqbn}${FQBN_EXTRA:+,$FQBN_EXTRA}"
  local prop=()
  [[ -n "$cppflags" ]] && prop=(--build-property "compiler.cpp.extra_flags=$cppflags")
  echo
  echo "============================================================"
  echo ">> Building $name"
  echo "   sketch : $sketch"
  echo "   fqbn   : $fqbn"
  echo "============================================================"
  arduino-cli compile \
    --fqbn "$fqbn" \
    --library "$LIBRARY" \
    --build-path "$out" \
    --warnings default \
    ${prop[@]+"${prop[@]}"} \
    "$sketch"

  if [[ "$UPLOAD" == "1" ]]; then
    if [[ -z "${PORT:-}" ]]; then
      echo "ERROR: --upload given but PORT is not set (e.g. PORT=/dev/cu.usbmodem1101)." >&2
      exit 1
    fi
    echo ">> Uploading $name to $PORT"
    arduino-cli upload --fqbn "$fqbn" --port "$PORT" --input-dir "$out" "$sketch"
  fi
}

case "$TARGET" in
  next)
    build_one HealthyPi5_NEXT  "$ROOT/examples/Applications/HealthyPi5_NEXT"  "$FQBN_BASE:os=freertos"
    ;;
  raw)
    build_one RawProcessing  "$ROOT/examples/Tutorials/09_RawProcessing"  "$FQBN_BASE:os=freertos"
    ;;
  openview)
    build_one OpenView_Stream "$ROOT/examples/Tutorials/08_OpenView_Stream" "$FQBN_BASE"
    ;;
  datalog)
    build_one SD_Datalog     "$ROOT/examples/Tutorials/11_SD_Datalog"      "$FQBN_BASE:os=freertos"
    ;;
  display) # additionally needs the "GFX Library for Arduino" (Arduino_GFX)
    build_one Display_Vitals "$ROOT/examples/Tutorials/12_Display_Vitals"  "$FQBN_BASE:os=freertos"
    ;;
  ecg)    build_one ECG_Plotter  "$ROOT/examples/Tutorials/01_ECG_Plotter"          "$FQBN_BASE" ;;
  resp)   build_one Respiration  "$ROOT/examples/Tutorials/02_Respiration_Plotter"  "$FQBN_BASE" ;;
  ppg)    build_one PPG_Plotter  "$ROOT/examples/Tutorials/03_PPG_Plotter"          "$FQBN_BASE" ;;
  spo2)   build_one SpO2         "$ROOT/examples/Tutorials/04_SpO2"                 "$FQBN_BASE" ;;
  hr)     build_one HeartRate    "$ROOT/examples/Tutorials/05_HeartRate"            "$FQBN_BASE" ;;
  temp)   build_one Temperature  "$ROOT/examples/Tutorials/06_Temperature"          "$FQBN_BASE" ;;
  vitals) build_one Vitals       "$ROOT/examples/Tutorials/07_Vitals_Serial"        "$FQBN_BASE" ;;
  wireless) build_one Wireless_Bridge "$ROOT/examples/Tutorials/10_Wireless_Bridge" "$FQBN_BASE" ;;
  tutorials)   # all single-core tutorials sketches (Serial Plotter / Monitor)
    build_one ECG_Plotter  "$ROOT/examples/Tutorials/01_ECG_Plotter"          "$FQBN_BASE"
    build_one Respiration  "$ROOT/examples/Tutorials/02_Respiration_Plotter"  "$FQBN_BASE"
    build_one PPG_Plotter  "$ROOT/examples/Tutorials/03_PPG_Plotter"          "$FQBN_BASE"
    build_one SpO2         "$ROOT/examples/Tutorials/04_SpO2"                 "$FQBN_BASE"
    build_one HeartRate    "$ROOT/examples/Tutorials/05_HeartRate"            "$FQBN_BASE"
    build_one Temperature  "$ROOT/examples/Tutorials/06_Temperature"          "$FQBN_BASE"
    build_one Vitals       "$ROOT/examples/Tutorials/07_Vitals_Serial"        "$FQBN_BASE"
    build_one Wireless_Bridge "$ROOT/examples/Tutorials/10_Wireless_Bridge"   "$FQBN_BASE"
    ;;
  all)
    build_one HealthyPi5_NEXT    "$ROOT/examples/Applications/HealthyPi5_NEXT"    "$FQBN_BASE:os=freertos"
    build_one RawProcessing    "$ROOT/examples/Tutorials/09_RawProcessing"    "$FQBN_BASE:os=freertos"
    build_one OpenView_Stream  "$ROOT/examples/Tutorials/08_OpenView_Stream" "$FQBN_BASE"
    ;;
esac

echo
echo ">> Done. Artifacts in $BUILD_DIR/"
