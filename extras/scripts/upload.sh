#!/usr/bin/env bash
#
# upload.sh — build one example and flash it to a HealthyPi 5 / RP2040 board.
# ============================================================================
# DEFAULT programmer is the Raspberry Pi Debug Probe (CMSIS-DAP) over SWD — no
# serial port, no BOOTSEL button, and it works even when the firmware is busy or
# crashed (ideal for the multi-hour soak). Falls back to USB serial / UF2 with
# --serial. Build logic is reused from build.sh (single source of truth).
#
# Wiring (Debug Probe -> HealthyPi 5 debug header):
#   probe "D" / SWD 3-pin  ->  SWCLK, GND, SWDIO   (programming + debug)
#   probe "U" / UART       ->  GP0 (RP2040 TX) -> probe RX, GP1 (RX) <- probe TX
#                              (optional: lets --monitor read the HPI_INSTR console)
#
#   ./extras/extras/scripts/upload.sh                 # build HealthyPi5_NEXT, upload via Debug Probe
#   ./extras/extras/scripts/upload.sh raw             # RawProcessing
#   ./extras/extras/scripts/upload.sh openview        # 08_OpenView_Stream (single-core)
#   ./extras/extras/scripts/upload.sh wireless        # 10_Wireless_Bridge (HealthyBridge/ESP32)
#   ./extras/extras/scripts/upload.sh datalog         # 11_SD_Datalog (record waveforms to microSD)
#   ./extras/extras/scripts/upload.sh next --monitor  # upload via probe, then open the UART console
#   ./extras/extras/scripts/upload.sh next --serial   # use USB serial / UF2 instead of the probe
#   ./extras/extras/scripts/upload.sh next --serial --port /dev/cu.usbmodem1101
#
# Env overrides:
#   PROGRAMMER  CMSIS-DAP upload method id   (default picoprobe_cmsis_dap)
#   PORT        serial port for --serial uploads (default: auto-detect)
#   FQBN_BASE   board FQBN without options   (default rp2040:rp2040:rpipico)
#   MON_BAUD    baud for --monitor           (default 115200, the UART0 console)
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT"

FQBN_BASE="${FQBN_BASE:-rp2040:rp2040:rpipico}"
BUILD_DIR="${BUILD_DIR:-$ROOT/build}"
MON_BAUD="${MON_BAUD:-115200}"
PROGRAMMER="${PROGRAMMER:-picoprobe_cmsis_dap}"   # Raspberry Pi Debug Probe

# --- args -------------------------------------------------------------------
TARGET="next"
PORT="${PORT:-}"
METHOD="probe"        # probe (SWD, default) | serial (USB CDC / UF2)
MONITOR=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    next|raw|openview|datalog|display)  TARGET="$1" ;;
    ecg|resp|ppg|spo2|hr|temp|vitals|wireless) TARGET="$1" ;;
    --serial|--uf2)    METHOD="serial" ;;
    --probe|--swd)     METHOD="probe" ;;
    --port|-p)         PORT="${2:-}"; METHOD="serial"; shift ;;
    --monitor|-m)      MONITOR=1 ;;
    -h|--help)         grep '^#' "$0" | sed 's/^#\{1,2\} \{0,1\}//'; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

# --- map target -> build-output name, sketch dir, os menu option ------------
case "$TARGET" in
  next)       NAME="HealthyPi5_NEXT";       SKETCHDIR="Applications/HealthyPi5_NEXT";       OSOPT="os=freertos" ;;
  raw)        NAME="RawProcessing";       SKETCHDIR="Tutorials/09_RawProcessing";      OSOPT="os=freertos" ;;
  openview)   NAME="OpenView_Stream";     SKETCHDIR="Tutorials/08_OpenView_Stream";    OSOPT="" ;;
  datalog)    NAME="SD_Datalog";          SKETCHDIR="Tutorials/11_SD_Datalog";         OSOPT="os=freertos" ;;
  display)    NAME="Display_Vitals";      SKETCHDIR="Tutorials/12_Display_Vitals";     OSOPT="os=freertos" ;;
  ecg)    NAME="ECG_Plotter"; SKETCHDIR="Tutorials/01_ECG_Plotter";         OSOPT="" ;;
  resp)   NAME="Respiration"; SKETCHDIR="Tutorials/02_Respiration_Plotter"; OSOPT="" ;;
  ppg)    NAME="PPG_Plotter"; SKETCHDIR="Tutorials/03_PPG_Plotter";         OSOPT="" ;;
  spo2)   NAME="SpO2";        SKETCHDIR="Tutorials/04_SpO2";                OSOPT="" ;;
  hr)     NAME="HeartRate";   SKETCHDIR="Tutorials/05_HeartRate";           OSOPT="" ;;
  temp)   NAME="Temperature"; SKETCHDIR="Tutorials/06_Temperature";         OSOPT="" ;;
  vitals) NAME="Vitals";      SKETCHDIR="Tutorials/07_Vitals_Serial";       OSOPT="" ;;
  wireless) NAME="Wireless_Bridge"; SKETCHDIR="Tutorials/10_Wireless_Bridge"; OSOPT="" ;;
esac
SKETCH="$ROOT/examples/$SKETCHDIR"
OUT="$BUILD_DIR/$NAME"

command -v arduino-cli >/dev/null 2>&1 || {
  echo "ERROR: arduino-cli not found in PATH." >&2; exit 1; }

# --- helpers ----------------------------------------------------------------
detect_port() {
  local p
  p="$(arduino-cli board list 2>/dev/null \
        | awk 'NR>1 && $1 ~ /(cu\.usbmodem|ttyACM|ttyUSB|COM[0-9])/ {print $1; exit}')"
  [[ -n "$p" ]] && { echo "$p"; return; }
  for g in /dev/cu.usbmodem* /dev/ttyACM* /dev/ttyUSB*; do
    [[ -e "$g" ]] && { echo "$g"; return; }
  done
}
detect_bootsel_volume() {
  for v in /Volumes/RPI-RP2 /Volumes/RP2350 /media/*/RPI-RP2 /media/*/RP2350; do
    [[ -d "$v" ]] && { echo "$v"; return; }
  done
}
# Compose the full FQBN for a given uploadmethod menu value.
fqbn_with() {  # $1 = uploadmethod value
  local opts=""
  [[ -n "$OSOPT" ]] && opts="$OSOPT"
  opts="${opts:+$opts,}uploadmethod=$1"
  echo "$FQBN_BASE:$opts"
}

echo "============================================================"
echo ">> Upload target : $NAME"
echo "   method        : $METHOD"
echo "============================================================"

if [[ "$METHOD" == "probe" ]]; then
  # --- Debug Probe (CMSIS-DAP / SWD) — the default --------------------------
  FQBN_FULL="$(fqbn_with "$PROGRAMMER")"
  echo ">> Programmer    : $PROGRAMMER (SWD via OpenOCD)"
  echo "   fqbn          : $FQBN_FULL"

  # Build with the SAME FQBN so build + upload are consistent.
  FQBN_EXTRA="uploadmethod=$PROGRAMMER" "$SCRIPT_DIR/build.sh" "$TARGET"

  echo
  echo ">> Flashing over SWD (OpenOCD will probe for the CMSIS-DAP adapter)"
  # No --port: the picoprobe_cmsis_dap upload tool talks SWD, not a serial port.
  if ! arduino-cli upload --fqbn "$FQBN_FULL" --input-dir "$OUT" "$SKETCH"; then
    echo >&2
    echo "ERROR: SWD upload failed. Check that:" >&2
    echo "  - the Debug Probe is plugged into this host (USB), and" >&2
    echo "  - its SWD cable is on the board's SWCLK/GND/SWDIO, and the board is powered." >&2
    echo "  - or fall back to USB:  ./extras/extras/scripts/upload.sh $TARGET --serial" >&2
    exit 1
  fi
  echo ">> Flashed and reset via the Debug Probe."

else
  # --- USB serial / UF2 fallback --------------------------------------------
  [[ -z "$PORT" ]] && PORT="$(detect_port || true)"
  if [[ -n "$PORT" ]]; then
    echo ">> Serial upload on $PORT (1200-bps-touch reset into the bootloader)"
    PORT="$PORT" "$SCRIPT_DIR/build.sh" "$TARGET" --upload
  else
    VOL="$(detect_bootsel_volume || true)"
    if [[ -n "$VOL" ]]; then
      echo ">> No serial port; BOOTSEL volume at $VOL — build + copy UF2"
      "$SCRIPT_DIR/build.sh" "$TARGET"
      UF2="$(ls "$OUT"/*.uf2 2>/dev/null | head -1 || true)"
      [[ -z "$UF2" ]] && { echo "ERROR: no .uf2 produced in $OUT" >&2; exit 1; }
      echo ">> cp $UF2 -> $VOL/"; cp "$UF2" "$VOL"/
      echo ">> Copied; the board will reboot into the new firmware."
    else
      echo "ERROR: no serial port and no BOOTSEL volume found." >&2
      echo "  - plug the board in, or hold BOOTSEL while plugging in, or" >&2
      echo "  - use the Debug Probe (drop --serial), or pass --port." >&2
      exit 1
    fi
  fi
fi

# --- optional UART0 telemetry monitor ---------------------------------------
if [[ "$MONITOR" == "1" ]]; then
  MON_PORT="$PORT"
  [[ -z "$MON_PORT" ]] && MON_PORT="$(detect_port || true)"
  if [[ -n "$MON_PORT" ]]; then
    echo
    echo ">> Monitor on $MON_PORT @ ${MON_BAUD} (Ctrl-C to exit)"
    echo "   HPI_INSTR/APP telemetry is on UART0 (GP0/GP1, e.g. the Debug Probe's"
    echo "   UART bridge). The USB-CDC port carries the BINARY OpenView stream and"
    echo "   will look like garbage in a text monitor."
    arduino-cli monitor --port "$MON_PORT" --config "baudrate=$MON_BAUD"
  else
    echo ">> --monitor: no serial port found for the UART console." >&2
  fi
fi
