# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

An Arduino 1.5-format library (`library.properties` + `src/` at the root) named **ProtoCentral HealthyPi 5**, targeting the RP2040 main MCU of the HealthyPi 5 biosignal board via the [`arduino-pico`](https://github.com/earlephilhower/arduino-pico) core. It ships both the runtime library and the example sketches that consume it.

There is no test suite — "does it build" is `arduino-cli compile`, done through `extras/scripts/build.sh` locally and by CI on push.

## CI

Three workflows in `.github/workflows/`:

- **arduino-lint.yml** — `arduino/arduino-lint-action` at `library-manager: update` (the library is in the Library Manager index). Deliberately *not* `compliance: strict`: strict promotes rule LP015 to an error over the space in the published name `ProtoCentral HealthyPi 5`, and renaming would break existing installs. Two warnings (LP015, LS008) are expected and non-fatal.
- **compile-examples.yml** — matrix over the two *build tiers*, not two boards, since `architectures=rp2040`. Adding an example means adding it to the right tier's `sketch-paths`, mirroring `extras/scripts/build.sh`.
- **release.yml** — on a `v*` tag, compiles `examples/Applications/HealthyPi5_NEXT` at `os=freertos` and attaches `HealthyPi5_NEXT-<tag>.uf2` to the GitHub release. Needs `permissions: contents: write`.

`compile-sketches` does not resolve `library.properties` `depends`, so the two ProtoCentral sensor libraries are listed explicitly in both compile and release workflows. MAX30001 is pinned to 2.0.0 because [HPIAcq.cpp](src/HPIAcq.cpp) targets its polling API.

## Build / flash

```bash
./extras/scripts/install-core.sh            # one-time: arduino-pico core + ProtoCentral MAX30001@2.0.0 + AFE4490 libs
./extras/scripts/build.sh next              # compile HealthyPi5_NEXT
./extras/scripts/build.sh tutorials         # compile all single-core tutorial sketches
CLEAN=1 ./extras/scripts/build.sh all       # wipe build/ first
./extras/scripts/upload.sh next --monitor   # flash via Raspberry Pi Debug Probe (SWD, the default), then open UART0
./extras/scripts/upload.sh ecg --serial     # USB/UF2 fallback
```

Targets: `next` `openview` `raw` `datalog` · `ecg` `resp` `ppg` `spo2` `hr` `temp` `vitals` `wireless` · `tutorials` `all`.

`build.sh` passes `--library "$ROOT"` so the repo itself is the library under compilation; sensor libraries come from the global Arduino libraries folder. FQBN is `rp2040:rp2040:rpipico`, with `os=freertos` appended for any target that uses the runtime (`next`, `raw`, `datalog`). `upload.sh` builds with the identical FQBN it uploads with, including the `uploadmethod=` menu option.

Two serial ports, and confusing them is the most common mistake: **USB-CDC (`Serial`) carries the binary OpenView stream and the command plane; UART0 (`Serial1`, GP0/GP1 @115200) carries the `HPI_INSTR` telemetry and `HPI_FAULT` dumps.** Never route debug text to `Serial`.

## Architecture

The library owns **both RP2040 cores** — it defines strong `setup1()`/`loop1()` overriding arduino-pico's weak ones, so a sketch that includes `Protocentral_HealthyPi_5.h` must never define its own. It builds only under the FreeRTOS-SMP variant; the header `#error`s without `__FREERTOS`, and `Protocentral_HealthyPi_5.cpp` / `HPIAcq.cpp` compile to empty TUs without it.

**core1 — acquisition only.** [HPIAcq.cpp](src/HPIAcq.cpp) owns SPI0 exclusively, busy-paces itself to the 128 SPS deadline with `micros()`, and reads MAX30001 (ECG every cycle, BioZ + PPG every 2nd) and AFE4400. It calls **no FreeRTOS API**. Its only outputs are `hpi_ring_push()` and a handful of `volatile uint32_t` counters. This isolation is what makes lossless sampling structural rather than a tuning exercise — don't add anything to core1 that can block.

**The ring.** [HPIRing.cpp](src/HPIRing.cpp) is the only TU that includes pico's `queue.h` (FreeRTOS's `queue.h` is included in the runtime; the same-named headers must never meet). 512-deep SPSC, ~4 s of slack at 128 SPS.

**core0 — broker + sinks.** The broker task drains the ring and fans each `hpi_sample_t` out to one independent FreeRTOS queue per registered sink, **drop-newest with a per-sink counter**. A slow or absent sink fills only its own queue; it can never back-pressure core1 or another sink. Every core0 task is pinned to core0 (`vTaskCoreAffinitySet`) so nothing migrates onto core1.

Task priorities matter and are non-obvious: arduino-pico runs `loop()` at `configMAX_PRIORITIES/2`, and the `enableLoopStream()` consumer busy-drains at that priority. **Every service task therefore sits above `HPI_PRIO_LOOP`** — placing one below it silently starves telemetry and OpenView output.

Watchdog: a supervisor task polls the core1 and broker heartbeats every 200 ms and stops feeding the HW WDT on a stall, after printing an `HPI_FAULT` forensic dump (`g_c1_stage` / `g_acq_substep` pinpoint where core1 froze). Sink and instr tasks are deliberately *not* watchdogged — they legitimately block. core1 gets a 30 s grace period during sensor bring-up.

**Sinks** (`HPISink` subclasses, registered before `begin()`): `OpenViewSink`, `DspSink` (vitals), `SdSink`, `BridgeSink`, plus `onSample()` callbacks and `enableLoopStream()`/`read()` for DSP in `loop()`. All sink instances are `static` inside their enabler — no heap.

### Wire formats — byte-frozen, do not reorder

- **OpenView 2 DATA frame** ([HPIOpenView.h](src/HPIOpenView.h)): 29 bytes, `0A FA | len | 0x02 | payload[22] | 00 0B`. Changing field order breaks OpenView 2 and the NEXT host tools.
- **HealthyBridge Lite** ([HPIBridge.h](src/HPIBridge.h)): `0xAA55 | type | flags | len | seq | payload | crc16` over UART1 @921600 with RTS/CTS to the ESP32-C3. The header is shared verbatim with the [healthybridge-esp32](https://github.com/Protocentral/healthybridge-esp32) firmware.
- **Host command plane** ([cmd.h](src/cmd.h)): `AA 55 | type | len | payload | xor`. Bounded by construction — a peer length `> CMD_MAX_PAYLOAD` is rejected before any byte is stored. Keep it that way.
- **SD records** ([HPISd.h](src/HPISd.h)): 32-byte `$HP5` header + 16-byte int32-LE records, decodable by the NEXT firmware's `decode_rec.py`.

`hpi_sample_t` ([hpi_types.h](src/hpi_types.h)) mirrors the production NEXT firmware's field order 1:1. Keep it that way.

### Pins

[board.h](src/board.h) is the **single source of truth**, carried verbatim from the NEXT/Zephyr firmware, correct for board revisions 5.2–5.7. Never hardcode a GPIO; always use `HPI_PIN_*`. Two pins are historically mis-documented and the old v1 sketches got them wrong: **AFE4400 CS is 19 (not 27)** and **SD CS is 13 (not 16 — 16 is the LCD backlight)**.

`board.h` deliberately does *not* pull in the runtime header, so the single-core teaching sketches (01–08) can include it alone and build without FreeRTOS.

## Sketch tiers

Tutorials 01–08 talk to the sensor libraries directly and include only `<board.h>` — single-core, no runtime, Serial Plotter/Monitor at 115200. Tutorials 09/11 and `examples/Applications/HealthyPi5_NEXT` include `<FreeRTOS.h>` then `<Protocentral_HealthyPi_5.h>` and run the full spine. Keep that split: a change to the runtime should not force a teaching sketch to grow a FreeRTOS dependency.

## Known stale references

The README's "What's in this repo" table still points at `libraries/HealthyPi5/` and `scripts/` from before the repo became a root-level 1.5-format library — the real paths are `src/` and `extras/scripts/`. The usage comments inside the scripts say `./extras/extras/scripts/…`. Both are cosmetic; the scripts themselves resolve `ROOT` correctly.

The ESP32-C3 firmware is **not** in this repo (it lives in [healthybridge-esp32](https://github.com/Protocentral/healthybridge-esp32)); the old `esp32/` tombstone folder was removed in favour of the README callout. Because v2 introduced the HealthyBridge Lite framing, a board running pre-v2 ESP32-C3 firmware loses BLE/Wi-Fi until that chip is reflashed.

On-panel LCD (LVGL) support is intentionally absent from this release, though `board.h` still carries the LCD pins and `HPISpi1.cpp` already arbitrates SPI1 for a future LCD sharing the bus with the SD card.
