# Changelog

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
