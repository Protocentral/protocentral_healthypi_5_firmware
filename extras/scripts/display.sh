#!/usr/bin/env bash
#
# display.sh — one-command build + flash for 12_Display_Vitals (ILI9488).
# ============================================================================
# 12_Display_Vitals is the only sketch in this repo that needs a library beyond
# the ProtoCentral sensor drivers: "GFX Library for Arduino" (Arduino_GFX), which
# it uses to drive the panel FROM THE SKETCH. The HealthyPi 5 library itself has
# no display dependency and never will — see the example's README.
#
# This script exists so you don't have to remember that. It checks for
# Arduino_GFX, installs it if missing, then hands off to upload.sh (which owns
# the FQBN / Debug-Probe / UF2-fallback logic — single source of truth).
#
#   ./extras/scripts/display.sh                  # build + flash via Debug Probe (SWD)
#   ./extras/scripts/display.sh --monitor        # ...then open the UART0 console
#   ./extras/scripts/display.sh --serial         # flash over USB serial / UF2 instead
#   ./extras/scripts/display.sh --port /dev/cu.usbmodem1101
#   ./extras/scripts/display.sh --build-only     # compile only, don't flash
#   ./extras/scripts/display.sh --no-install     # fail instead of installing Arduino_GFX
#
# Any other flag is passed straight through to upload.sh (--probe, --swd, ...).
#
# Env overrides: the same ones upload.sh honours (PROGRAMMER, PORT, FQBN_BASE,
# MON_BAUD, BUILD_DIR).
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

GFX_LIB="GFX Library for Arduino"
SKETCH="$ROOT/examples/Tutorials/12_Display_Vitals"

BUILD_ONLY=0
AUTO_INSTALL=1
PASS=()          # everything we forward to upload.sh

for arg in "$@"; do
  case "$arg" in
    --build-only|-b) BUILD_ONLY=1 ;;
    --no-install)    AUTO_INSTALL=0 ;;
    # Skip the shebang, then strip the leading '# ' from the header block.
    -h|--help)       grep '^#' "$0" | grep -v '^#!' | sed 's/^#\{1,2\} \{0,1\}//'; exit 0 ;;
    *)               PASS+=("$arg") ;;
  esac
done

# --- preflight --------------------------------------------------------------
command -v arduino-cli >/dev/null 2>&1 || {
  echo "ERROR: arduino-cli not found in PATH." >&2
  echo "       Install it, then run ./extras/scripts/install-core.sh." >&2
  exit 1; }

[[ -d "$SKETCH" ]] || { echo "ERROR: sketch not found: $SKETCH" >&2; exit 1; }

if ! arduino-cli core list 2>/dev/null | grep -qi 'rp2040:rp2040'; then
  echo "ERROR: the arduino-pico core (rp2040:rp2040) is not installed." >&2
  echo "       Run ./extras/scripts/install-core.sh first." >&2
  exit 1
fi

# --- the one dependency this sketch adds ------------------------------------
# Match the whole library name at the start of the line: 'lib list' is a padded
# column layout and the name itself contains spaces.
has_gfx() { arduino-cli lib list 2>/dev/null | grep -qE "^${GFX_LIB}[[:space:]]"; }

echo "============================================================"
echo ">> HealthyPi 5 — 12_Display_Vitals (ILI9488)"
echo "============================================================"

if has_gfx; then
  echo ">> $GFX_LIB ... already installed"
elif [[ "$AUTO_INSTALL" == "0" ]]; then
  echo "ERROR: '$GFX_LIB' is not installed and --no-install was given." >&2
  echo "       arduino-cli lib install \"$GFX_LIB\"" >&2
  exit 1
else
  echo ">> $GFX_LIB ... missing, installing"
  if ! arduino-cli lib install "$GFX_LIB"; then
    echo "ERROR: could not install '$GFX_LIB'." >&2
    echo "       Install it from the Arduino IDE Library Manager, or run:" >&2
    echo "       arduino-cli lib install \"$GFX_LIB\"" >&2
    exit 1
  fi
  # Don't trust the exit status alone — confirm it is actually resolvable now,
  # otherwise the compile fails later with a confusing missing-header error.
  has_gfx || { echo "ERROR: '$GFX_LIB' still not visible to arduino-cli." >&2; exit 1; }
fi

# --- build (+ flash) --------------------------------------------------------
# Both scripts already know the 'display' target: sketch dir + os=freertos.
if [[ "$BUILD_ONLY" == "1" ]]; then
  echo ">> Build only (not flashing)"
  exec "$SCRIPT_DIR/build.sh" display
fi

exec "$SCRIPT_DIR/upload.sh" display ${PASS[@]+"${PASS[@]}"}
