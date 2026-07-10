# 12 — Vitals + ECG on an ILI9488 display

Paints heart rate, SpO₂, respiration, temperature and battery as five tiles, with
a sweeping 128 SPS ECG trace underneath — **entirely from the sketch**. The
`HealthyPi5` library knows nothing about the display.

```
+----------------------------------------------------------+
| HealthyPi 5                                              |   title bar
+----------+----------+----------+----------+--------------+
| HR bpm   | SpO2 %   | RESP rpm | TEMP C   | BATT %       |   vitals tiles
|   72     |   98     |   14     |  36.7    |   87         |
+----------+----------+----------+----------+--------------+
| ECG 128 SPS                                              |
|      /\        /\        /\        /\                    |   sweeping trace
|  ___/  \___ __/  \___ __/  \___ __/  \___                |
+----------------------------------------------------------+
```

## Why the display is not in the library

The library owns both RP2040 cores and every timing-critical path. A display
driver is neither, so it stays out: the sketch pulls data through the ordinary
public API and paints it with whatever graphics library it likes.

| What | Where it comes from | Rate |
|---|---|---|
| HR, SpO₂, respiration | `HealthyPi5.vitals()` (the DSP sink) | ~1 Hz |
| Temperature, battery | `HealthyPi5.temperature_x100()`, `batterySoc()` (I²C poll) | ~1 Hz |
| ECG waveform | `HealthyPi5.enableLoopStream()` + `HealthyPi5.read(s)` | 128 SPS |

Nothing here touches a library internal, so **the core library gains no display
dependency** — you can delete this sketch and the library is unchanged.

**A slow display cannot break sampling.** `loop()` is the lowest-priority core0
task, draining a drop-newest queue. If painting can't keep up with 128 SPS, the
broker drops the newest sample (counted in `drops=` in the 1 Hz `HPI_INSTR`
telemetry on UART0) and core1 keeps acquiring losslessly. Watch `drops=` climb if
you lower the SPI clock — the trace thins out, acquisition does not.

## Hardware

ILI9488 320×480 SPI panel on **SPI1**, using the pins already in
[`board.h`](../../../src/board.h) (rev 5.2–5.7):

| Signal | GPIO |
|---|---|
| SCK | 10 |
| MOSI | 11 |
| MISO | 8 |
| CS | 29 |
| DC | 23 |
| RST | 28 |
| Backlight (PWM) | 16 |

## Setup

1. Board **"Raspberry Pi Pico"**, and **Tools → os: "FreeRTOS SMP"** (this sketch
   uses the dual-core runtime).
2. Install **"GFX Library for Arduino"** (Arduino_GFX, by Moon On Our Nation)
   from the Library Manager, in addition to the usual MAX30001 / AFE4490
   libraries.
3. Upload.

From the command line, one script does all of that:

```bash
./extras/scripts/display.sh              # install Arduino_GFX if needed, build, flash via Debug Probe
./extras/scripts/display.sh --monitor    # ...and then open the UART0 telemetry console
./extras/scripts/display.sh --serial     # flash over USB / UF2 instead of the probe
./extras/scripts/display.sh --build-only # compile only, don't flash
```

It is a thin wrapper: after the Arduino_GFX check it hands off to
`upload.sh display`, so the FQBN, Debug-Probe and UF2-fallback logic stay in one
place. `./extras/scripts/upload.sh display` works too if you already have the
library installed.

## Two gotchas worth knowing

**ILI9488 needs 18-bit colour over SPI.** The controller does *not* accept 16-bit
RGB565 pixel data on its SPI interface. The sketch therefore uses
`Arduino_ILI9488_18bit`, not `Arduino_ILI9488`; the latter gives a blank or
garbled screen. Colours are still written as `RGB565_*` constants — the driver
expands each pixel to RGB666 on the way out.

**SPI1 is shared with the microSD slot.** Display and card sit behind separate
chip-selects on the same bus. This sketch doesn't call `HealthyPi5.recordSD()`,
so it is the bus's only user. If you add SD recording, bracket the drawing calls
with the library's bus mutex so a card write can't interleave with a display
transfer:

```cpp
#include <HPISpi1.h>
...
hpi_spi1_lock();
drawWave(s.ecg);
hpi_spi1_unlock();
```

Take the lock once per batch of draws, not per pixel.

## Tuning the trace

`ECG_FULLSCALE` (default `40000`) maps to half the trace height. Lower it to
magnify a small signal; raise it if the trace clips. `HP_ALPHA` (`0.95`) is the
same 1-pole high-pass used by [`01_ECG_Plotter`](../01_ECG_Plotter) to strip
baseline wander — push it toward `1.0` for a gentler filter.

## Porting to another controller

Only two lines are panel-specific:

```cpp
static Arduino_DataBus *bus = new Arduino_HWSPI(HPI_PIN_LCD_DC, HPI_PIN_LCD_CS, &SPI1);
static Arduino_GFX     *gfx = new Arduino_ILI9488_18bit(bus, HPI_PIN_LCD_RST, 1, false);
```

Swap the second for `Arduino_ST7796`, `Arduino_ILI9341`, … and the rest of the
sketch is unchanged. See [`../README.md`](../README.md) for the other tutorials.
