# HealthyPi 5 — Tutorials (Arduino IDE)

Simple, single-signal Arduino sketches for **HealthyPi 5 v5.2–v5.7**, meant to be
read top to bottom. Each one brings up **one** sensor and plots/prints **one**
kind of data, so you can learn the hardware a piece at a time. They are
standalone (no dual-core runtime) and share only `board.h` — the authoritative
pin map — so the pins always match the board.

Sketches **01–07** are the single-signal basics. **08–11** are the more advanced
tutorials: `08_OpenView_Stream` (full OpenView 2 stream),
`09_RawProcessing` (your own DSP in `loop()` over the dual-core spine),
`10_Wireless_Bridge` (streaming vitals to the ESP32-C3 for BLE/Wi-Fi — the
RP2040 has no radio of its own; see [`../../docs/WIRELESS.md`](../../docs/WIRELESS.md)),
and `11_SD_Datalog` (recording the raw waveforms to a microSD card). For the
full production firmware (all features at once) see
[`../Applications/HealthyPi5_NEXT`](../Applications/HealthyPi5_NEXT).

## The two visualization paths (important)

| Path | Use it for | How |
|---|---|---|
| **Arduino Serial Plotter** | one or a few numeric channels (these tutorial sketches) | Tools > Serial Plotter, **115200 baud** |
| **OpenView 2** | the full multi-channel binary stream | `08_OpenView_Stream` example |

The Serial Plotter **cannot** render the OpenView binary packet — that is the only
thing it "doesn't support." For single signals like ECG or PPG it is exactly the
right tool, and that is what these sketches use.

## The sketches

| # | Sketch | Sensor | Output |
|---|---|---|---|
| 01 | `01_ECG_Plotter` | MAX30001 (ECG) | ECG waveform → Serial **Plotter** |
| 02 | `02_Respiration_Plotter` | MAX30001 (BioZ) | breathing waveform → Serial **Plotter** |
| 03 | `03_PPG_Plotter` | AFE4400 | IR + RED pulse waveforms → Serial **Plotter** |
| 04 | `04_SpO2` | AFE4400 | SpO2 % + pulse, with no-finger handling → Serial **Monitor** |
| 05 | `05_HeartRate` | MAX30001 (RtoR) | heart rate + R-R interval → Serial **Monitor** |
| 06 | `06_Temperature` | MAX30205 (I²C) | body temperature °C, absent-safe → Serial **Monitor** |
| 07 | `07_Vitals_Serial` | all three | combined HR / SpO2 / temp line → Serial **Monitor** |
| 08 | `08_OpenView_Stream` | all sensors | byte-exact 29-byte OpenView 2 frame → **OpenView 2** |
| 09 | `09_RawProcessing` | all sensors | your own DSP in `loop()` over the dual-core spine → Serial |
| 10 | `10_Wireless_Bridge` | all sensors | vitals over HealthyBridge → **ESP32-C3** (BLE / Wi-Fi) |
| 11 | `11_SD_Datalog` | all sensors | raw waveforms recorded to a microSD card (`/REC*.BIN`) |
| 12 | `12_Display_Vitals` | all sensors | vitals tiles + sweeping ECG on an **ILI9488** SPI panel, drawn from the sketch (needs Arduino_GFX) |

Build/flash any of them with the repo scripts, e.g.
`./scripts/upload.sh ecg --monitor` (targets: `ecg resp ppg spo2 hr temp vitals
wireless`, plus `openview` / `raw` for 08 / 09, `datalog` for 11, or `tutorials`
to build the standalone set at once).

## One-time setup

1. **Board core:** install **arduino-pico** (Earle Philhower) and select
   **"Raspberry Pi Pico"**. (`../../scripts/install-core.sh` automates this.)
2. **Sensor libraries** (Library Manager, or the install script):
   - **ProtoCentral MAX30001** — v2.0.0 or newer (ECG/BioZ/heart-rate).
   - **ProtoCentral AFE4490 PPG and SpO2 boards library** — this provides
     `protocentral_afe44xx.h` **and** the SpO2 algorithm. The HealthyPi 5's PPG
     AFE is an **AFE4400** (same family as the AFE4490) — **not** a MAX3010x.
3. **`board.h`:** these sketches `#include <board.h>` from the in-repo
   **HealthyPi5** library. Add this repo's `libraries/` folder to your Arduino
   libraries (or build with `--libraries libraries` via `../../scripts/build.sh`).

## Pins (matches the v5.7 schematic)

Every sketch uses the `HPI_PIN_*` macros from `board.h`, carried over verbatim
from the HealthyPi 5 v5.7 schematic / Zephyr device tree (v5.2–v5.7 are
electrically identical). Notably:

| Signal | GPIO |
|---|---|
| SPI0 SCK / MOSI / MISO | 2 / 3 / 4 |
| MAX30001 CS | 5 |
| AFE4400 CS / PWDN | **19** / 18 |
| I²C1 SDA / SCL (temp, fuel gauge) | 6 / 7 |

> **If you have an old sketch** (e.g. `1_healthypi_rp2040_serial_ble_stream.ino`):
> it is **outdated** for v5.7. It mis-pins the AFE4400 CS (27 → should be **19**)
> and the SD CS (16 → should be **13**), and it uses the old MAX30001 interrupt
> API that does not compile against the v2.0.0 library. Use these sketches
> instead.

## Tips

- **Baud must match:** set the Serial Plotter/Monitor to **115200**.
- **Serial Plotter legend:** the sketches print `label:value` (e.g. `ECG:123`,
  `IR:900,RED:1200`) so each trace is named.
- **Filtering:** the ECG/PPG sketches high-pass the signal to remove baseline
  drift so the morphology fills the plot — comment those lines out to see the raw
  sensor data (a good exercise).
- **SpO2 no-finger threshold:** `FINGER_IR_MIN` is a starting value — watch
  `data.IR_data` with and without a finger on your board and tune it.
