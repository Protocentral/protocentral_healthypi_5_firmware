#!/usr/bin/env bash
#
# display-test.sh — build + flash the HealthyPi5_Display example, UNMODIFIED,
# to reproduce the "no SD card -> nothing on the display" fault.
# ============================================================================
# This is a DIAGNOSTIC script, not part of the normal build. It exists because
# HealthyPi5_Display is not wired into build.sh or CI, and it needs two
# libraries the repo does not declare in library.properties:
#
#   lvgl                     (LVGL 9.x — the sketch uses the v9 API:
#                             lv_display_create / lv_screen_active)
#   GFX Library for Arduino  (Arduino_GFX — ILI9488 / ST7796 panel driver)
#
# Flashes over USB (1200-bps touch into the UF2 bootloader), because that is
# how the board is connected during this investigation.
#
#   ./extras/scripts/display-test.sh                  # install deps, build, flash
#   ./extras/scripts/display-test.sh --build-only     # compile, do not flash
#   ./extras/scripts/display-test.sh --port /dev/cu.usbmodem1401
#   ./extras/scripts/display-test.sh --monitor        # then open UART0 (see below)
#   ./extras/scripts/display-test.sh --no-deps        # skip the library install
#   CLEAN=1 ./extras/scripts/display-test.sh          # wipe this target's build dir
#
# ---------------------------------------------------------------------------
# READ THIS: the diagnostics you want are NOT on USB.
# ---------------------------------------------------------------------------
# The sketch's traces (HPI_DISP ...) and the SD sink's traces (SD_MOUNT ...,
# SD_REC ...) are printed to Serial1 = UART0 = GP0/GP1 @115200. The USB-CDC
# port carries the BINARY OpenView stream and will look like garbage in a text
# monitor. To read the traces you need a UART adapter (or the Debug Probe's
# UART bridge) on GP0 (RP2040 TX) / GP1 (RP2040 RX), then:
#
#   ./extras/scripts/display-test.sh --monitor --mon-port /dev/cu.usbserial-XXXX
#
# WITHOUT a UART adapter you can still discriminate the two candidate faults
# by looking at the panel backlight — see the checklist this script prints.
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT"

FQBN="${FQBN:-rp2040:rp2040:rpipico:os=freertos}"
BUILD_DIR="${BUILD_DIR:-$ROOT/build}"
SKETCH="$ROOT/examples/Applications/HealthyPi5_Display"
OUT="$BUILD_DIR/HealthyPi5_Display"
MON_BAUD="${MON_BAUD:-115200}"

PORT="${PORT:-}"
MON_PORT="${MON_PORT:-}"
DO_FLASH=1
DO_DEPS=1
DO_MONITOR=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-only|-b)  DO_FLASH=0 ;;
    --no-deps)        DO_DEPS=0 ;;
    --monitor|-m)     DO_MONITOR=1 ;;
    --port|-p)        PORT="${2:-}"; shift ;;
    --mon-port)       MON_PORT="${2:-}"; shift ;;
    -h|--help)        grep '^#' "$0" | sed 's/^#\{1,2\} \{0,1\}//'; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

command -v arduino-cli >/dev/null 2>&1 || {
  echo "ERROR: arduino-cli not found in PATH." >&2; exit 1; }

[[ -f "$SKETCH/HealthyPi5_Display.ino" ]] || {
  echo "ERROR: $SKETCH/HealthyPi5_Display.ino not found." >&2
  echo "       Are you on the feature/display-support branch?" >&2; exit 1; }

# --- extra libraries this example needs -------------------------------------
# Deliberately NOT added to library.properties by this script: that is a
# reviewed change to the published library, not something a debug tool should
# do behind your back.
if [[ "$DO_DEPS" == "1" ]]; then
  echo ">> Checking the two extra libraries HealthyPi5_Display needs"
  need_lib() {  # $1 = index name, $2 = folder name to probe for
    if arduino-cli lib list 2>/dev/null | grep -qi "^$2 "; then
      echo "   - $1: already installed"
    else
      echo "   - $1: installing"
      arduino-cli lib install "$1" || {
        echo "     ERROR: could not install '$1'." >&2
        echo "     Install it manually, or re-run with --no-deps." >&2
        exit 1; }
    fi
  }
  need_lib "GFX Library for Arduino" "GFX Library for Arduino"
  need_lib "lvgl"                    "lvgl"

  # LVGL is configured by an lv_conf.h that must sit NEXT TO the lvgl folder
  # (lvgl/src/lv_conf_internal.h includes "../../lv_conf.h"); a copy inside the
  # sketch folder is NOT found. Install the repo's copy if there isn't one.
  SKETCHBOOK="$(arduino-cli config get directories.user 2>/dev/null || echo "$HOME/Documents/Arduino")"
  if [[ ! -f "$SKETCHBOOK/libraries/lv_conf.h" ]]; then
    echo "   - lv_conf.h: installing $ROOT/extras/lv_conf.h -> $SKETCHBOOK/libraries/"
    cp "$ROOT/extras/lv_conf.h" "$SKETCHBOOK/libraries/lv_conf.h"
  else
    echo "   - lv_conf.h: already present at $SKETCHBOOK/libraries/lv_conf.h (left alone)"
  fi
fi

if [[ "${CLEAN:-0}" == "1" ]]; then
  echo ">> CLEAN: removing $OUT"
  rm -rf "$OUT"
fi

# --- build ------------------------------------------------------------------
echo
echo "============================================================"
echo ">> Building HealthyPi5_Display  (UNMODIFIED — reproduction build)"
echo "   sketch : $SKETCH"
echo "   fqbn   : $FQBN"
echo "============================================================"
arduino-cli compile \
  --fqbn "$FQBN" \
  --library "$ROOT" \
  --build-path "$OUT" \
  --warnings default \
  "$SKETCH"

if [[ "$DO_FLASH" == "0" ]]; then
  echo
  echo ">> --build-only: not flashing. Artifacts in $OUT/"
  exit 0
fi

# --- flash over USB ---------------------------------------------------------
detect_port() {
  local p
  p="$(arduino-cli board list 2>/dev/null \
        | awk 'NR>1 && $1 ~ /(cu\.usbmodem|ttyACM)/ {print $1; exit}')"
  [[ -n "$p" ]] && { echo "$p"; return; }
  for g in /dev/cu.usbmodem* /dev/ttyACM*; do
    [[ -e "$g" ]] && { echo "$g"; return; }
  done
}
detect_bootsel_volume() {
  for v in /Volumes/RPI-RP2 /media/*/RPI-RP2; do
    [[ -d "$v" ]] && { echo "$v"; return; }
  done
}

[[ -z "$PORT" ]] && PORT="$(detect_port || true)"

echo
if [[ -n "$PORT" ]]; then
  echo ">> Flashing over USB on $PORT (1200-bps touch -> UF2 bootloader)"
  arduino-cli upload --fqbn "$FQBN" --port "$PORT" --input-dir "$OUT" "$SKETCH"
  echo ">> Flashed."
else
  VOL="$(detect_bootsel_volume || true)"
  if [[ -n "$VOL" ]]; then
    UF2="$(ls "$OUT"/*.uf2 2>/dev/null | head -1 || true)"
    [[ -z "$UF2" ]] && { echo "ERROR: no .uf2 in $OUT" >&2; exit 1; }
    echo ">> No serial port, but BOOTSEL volume at $VOL — copying the UF2"
    cp "$UF2" "$VOL"/
    echo ">> Copied; the board reboots into the new firmware."
  else
    echo "ERROR: no serial port and no BOOTSEL volume found." >&2
    echo "  - plug the board in, or hold BOOTSEL while plugging it in, or" >&2
    echo "  - pass --port /dev/cu.usbmodemXXXX" >&2
    exit 1
  fi
fi

# --- what to look at --------------------------------------------------------
cat <<'CHECKLIST'

============================================================
REPRODUCTION CHECKLIST — run each case from a COLD boot
(unplug/replug or tap RUN; a warm reset can leave the panel
 initialised from the previous run and mask the fault)
============================================================

  CASE A — SD card INSERTED   : expect the UI to appear
  CASE B — SD card REMOVED    : the reported fault

For CASE B, the panel BACKLIGHT is the discriminator, because
display_task turns it on only AFTER it has taken the SPI1 lock:

  backlight DARK, screen dark
      -> display_task is blocked (or was never created) while the
         SD sink's eager mount in SdSink::begin() holds the SPI1
         mutex. Time it: if the UI appears after several seconds,
         it is a stall; if it never appears, SdFat is not returning.

  backlight ON, screen black
      -> display_task ran and gfx->begin() completed, so the block
         is elsewhere: panel init / driver mismatch, not the mutex.
         (Note HPI_DISPLAY_ST7796 is never defined, so the build
         always uses the ILI9488 driver.)

Time it with a stopwatch: how many seconds from power-up to the
UI in each case? A multi-second delta between A and B points
straight at the SD mount timeout.

With a UART adapter on GP0/GP1 the traces settle it outright:

    SD_MOUNT: calling SDFS.begin()          <- mount starts, lock held
    SD_MOUNT: SDFS.begin() FAILED after N ms <- if this NEVER prints,
                                                SdFat is hung
    HPI_DISP start (backlight off, ...)      <- display_task finally runs

============================================================
CHECKLIST

if [[ "$DO_MONITOR" == "1" ]]; then
  [[ -z "$MON_PORT" ]] && MON_PORT="$PORT"
  if [[ -n "$MON_PORT" ]]; then
    echo
    echo ">> Monitor on $MON_PORT @ $MON_BAUD (Ctrl-C to exit)"
    if [[ "$MON_PORT" == "$PORT" ]]; then
      echo "   WARNING: this is the USB-CDC port. It carries the BINARY OpenView"
      echo "   stream, not the HPI_DISP/SD_MOUNT traces — expect garbage. Pass"
      echo "   --mon-port with a UART adapter on GP0/GP1 to read the traces."
    fi
    arduino-cli monitor --port "$MON_PORT" --config "baudrate=$MON_BAUD"
  else
    echo ">> --monitor: no port to open." >&2
  fi
fi
