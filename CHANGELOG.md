# Changelog

## 2.1.0

- New application example **`HealthyPi5_Display`**: the on-panel UI on the
  480x320 SPI LCD — live HR / SpO2 / respiration / temperature cards and a
  hold-OK record button, drawn with LVGL 9 on its own core0 task, over the same
  dual-core spine as `HealthyPi5_NEXT`. Needs `lvgl` and `Arduino_GFX`, which
  are not declared in `library.properties`, plus `extras/lv_conf.h` copied next
  to the installed `lvgl` folder.
- New public API: `sdCardPresent()`, and `hpiSpi1Lock()` / `hpiSpi1Unlock()` so
  a sketch-level SPI1 peripheral can share the bus with the SD card. Call them
  only after `begin()` — the mutex is created there.
- `SdSink::begin()` now mounts the card once at startup so `sdCardPresent()` is
  meaningful before the first `REC_START`. With no card, SdFat retries for its
  init timeout on that task while holding the SPI1 mutex, which delays (but does
  not block) the first transaction of another SPI1 user.
- Recording start recovers from a card that was swapped or went idle: one
  remount-and-retry instead of failing outright. `SDFS.info()` is no longer
  called on the recording-start path, where it could trigger a full
  free-cluster scan costing seconds on large cards.
- `HPI_PIN_SD_CS` is driven high in `begin()` so the SD chip select is
  deterministically deselected before any other SPI1 device is used.
- CI: `HealthyPi5_Display` gets its own job. It needs libraries that are not in
  `library.properties` plus an `lv_conf.h` dropped next to the installed `lvgl`
  folder, and `compile-sketches` installs and compiles in one step with nowhere
  to inject that file, so the job drives `arduino-cli` directly.
- `build.sh` and `upload.sh` gain a `display` target, kept out of `all` so the
  normal build does not depend on LVGL or Arduino_GFX.
- README: document the new application, and correct the stale `libraries/HealthyPi5/`
  and `scripts/` paths left over from before the repo became a root-level
  Arduino 1.5-format library.

## 2.0.2

- CI: Arduino Lint and example-compile workflows, matching the other ProtoCentral
  libraries. Examples are compiled in both build tiers — stock `arduino-pico`
  (tutorials 01–08, 10) and `os=freertos` (09, 11, `HealthyPi5_NEXT`).
- CI: tagging a release now builds `HealthyPi5_NEXT` and attaches
  `HealthyPi5_NEXT-<tag>.uf2` to the GitHub release.
- Removed the stub `esp32/` folder. The ESP32-C3 firmware lives in
  [`healthybridge-esp32`](https://github.com/Protocentral/healthybridge-esp32);
  the README now states prominently that it **must be reflashed** for BLE / Wi-Fi
  to work with v2, because the HealthyBridge Lite framing is new in v2.
- Docs: fix the stale `#include <HealthyPi5.h>` in the `09_RawProcessing` README
  (the header is `Protocentral_HealthyPi_5.h`).

## 2.0.1

- Fix: the `09_RawProcessing` tutorial streamed OpenView but never enabled the
  I²C sensors, so the OpenView **temperature** channel read 0. It now calls
  `enableSensors()` (temperature + battery), matching the full firmware.

## 2.0.0 — NEXT rewrite

A complete rewrite of the HealthyPi 5 RP2040 firmware as a proper Arduino library
(**ProtoCentral HealthyPi 5**) built on the production **NEXT** dual-core
architecture (arduino-pico / FreeRTOS-SMP):

- Lossless 128 SPS acquisition on core1 into a lock-free ring; core0 broker with
  per-sink drop-newest queues, a hardware watchdog, and 1 Hz `HPI_INSTR` telemetry.
- Byte-compatible **OpenView 2** stream and **HealthyBridge** link to the ESP32-C3.
- Correct v5.2–v5.7 pin map (the v1 sketches were mis-pinned: AFE4400 CS 27→19,
  SD CS 16→13) and the current **MAX30001 v2.0.0** API.
- Tutorials (01–11): one sensor/idea at a time, through OpenView, your-own-DSP in
  `loop()`, wireless, and SD datalogging — plus the full `HealthyPi5_NEXT` firmware.
- On-panel LCD (LVGL) support is **not** in this release; it will return later.

### Migrating from v1
- The previous firmware (2023 Arduino sketches) is preserved on the **`v1-legacy`**
  tag: <https://github.com/Protocentral/protocentral_healthypi_5_firmware/tree/v1-legacy>
- The ESP32-C3 firmware moved to
  [`healthybridge-esp32`](https://github.com/Protocentral/healthybridge-esp32).
- Install via the Arduino Library Manager ("ProtoCentral HealthyPi 5") or copy this
  repo into your Arduino `libraries/` folder; see the README to get started.
