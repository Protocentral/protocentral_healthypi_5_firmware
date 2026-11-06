#!/usr/bin/env bash
#
# install-core.sh — install the arduino-pico (Earle Philhower) RP2040 core and
# the ProtoCentral sensor libraries this firmware needs, via arduino-cli.
# ============================================================================
# One-time setup. Idempotent: safe to re-run.
#
#   ./extras/extras/scripts/install-core.sh
# ============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PICO_URL="https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json"

if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "ERROR: arduino-cli not found in PATH. Install it first (e.g. 'brew install arduino-cli')." >&2
  exit 1
fi

echo ">> Registering the arduino-pico board package URL"
# add to the existing list if supported; fall back to set if not.
if ! arduino-cli config add board_manager.additional_urls "$PICO_URL" 2>/dev/null; then
  arduino-cli config set board_manager.additional_urls "$PICO_URL"
fi

echo ">> Updating the board index"
arduino-cli core update-index

echo ">> Installing rp2040:rp2040"
arduino-cli core install rp2040:rp2040

echo ">> Installed cores:"
arduino-cli core list | grep -i rp2040 || true

echo
echo ">> Installing ProtoCentral sensor libraries"
# MAX30001 v2.1.0 is the first release that enables the R-to-R (heart-rate)
# detector — pin it. Try Library Manager first, then the git tag (new releases
# can take a day to reach the Library Manager index). The AFE4490 library
# bundles BOTH protocentral_afe44xx.h and Protocentral_spo2_algorithm.h, so no
# separate SpO2 library is needed.
# If a name drifts, run 'arduino-cli lib search protocentral' for the exact string.
echo "   - ProtoCentral MAX30001@2.1.0"
arduino-cli lib install "ProtoCentral MAX30001@2.1.0" || {
  arduino-cli config set library.enable_unsafe_install true &&
  arduino-cli lib install --git-url "https://github.com/Protocentral/protocentral_max30001_arduino_library.git#v2.1.0"
} || \
  echo "     (could not install — install it manually)"
echo "   - ProtoCentral AFE4490 PPG and SpO2 boards library"
arduino-cli lib install "ProtoCentral AFE4490 PPG and SpO2 boards library" || \
  echo "     (could not install — install it manually)"

echo
echo ">> Done. Now build with: ./extras/extras/scripts/build.sh next"
