# HealthyPi 5 — RP2040 Pin Map (authoritative)

This map is **identical for hardware revisions 5.2 through 5.7** — they are
electrically the same board. It is carried over verbatim from the production
HealthyPi 5 **NEXT** firmware (`include/board.h`), which took it from the Zephyr
board device tree. Use these numbers; do not trust the older Arduino sketches.

The machine-readable source of truth is
[`libraries/HealthyPi5/src/board.h`](../libraries/HealthyPi5/src/board.h).

## Pins (RP2040 GPIO numbers)

| Function | Signal | GPIO | Notes |
|---|---|---:|---|
| **SPI0** (sensors) | SCK | 2 | shared ECG + PPG bus |
| | MOSI / TX | 3 | |
| | MISO / RX | 4 | |
| **MAX30001** (ECG/BioZ/HR) | CS | 5 | SPI mode 0, 4 MHz |
| **AFE4400** (PPG/SpO₂) | CS | **19** | SPI **mode 3**, 8 MHz |
| | PWDN | 18 | active-low reset |
| **I²C1** | SDA | 6 | |
| | SCL | 7 | |
| MAX30205 (temp) | addr | 0x49 | older kits; QWIIC; 1/256 °C/LSB |
| AS6221 (temp) | addr | 0x48 | newer kits; QWIIC; 1/128 °C/LSB; auto-detected at boot |
| MAX17048 (fuel gauge) | addr | 0x36 | absent without a battery |
| **SPI1** (display + SD) | SCK | 10 | |
| | MOSI / TX | 11 | |
| | MISO / RX | 8 | |
| Display | CS | 29 | |
| Display | DC | 23 | |
| Display | RST | 28 | |
| microSD | CS | **13** | |
| LCD backlight | PWM | 16 | |
| **UART1** → ESP32-C3 | TX | 24 | 921600 8N1 + RTS/CTS in NEXT |
| | RX | 25 | |
| | RTS | 27 | |
| | CTS | 26 | |
| **UART0** (debug) | TX | 0 | 115200 |
| | RX | 1 | |
| LED | Blue | 22 | active-low |
| LED | Green | 21 | active-low |
| Button UP / SW1 | | 15 | active-low |
| Button OK / SW2 | | 14 | active-low |
| Button DOWN / SW3 | | 12 | active-low |

## What the old public Arduino sketch got wrong

These are **bugs in the old sketch**, not revision differences (5.2 == 5.7):

| Signal | Old sketch | Correct | Consequence of the bug |
|---|---:|---:|---|
| **AFE4400 CS** | 27 | **19** | AFE never selected → no PPG, garbage SpO₂ |
| **microSD CS** | 16 | **13** | 16 is the LCD backlight → SD never works / backlight conflict |
| **Button SW1 (UP)** | 12 | **15** | wrong button mapping |
| **Button SW3 (DOWN)** | 1 | **12** | wrong button mapping |
| MAX30001 INT2B | 19 | (n/a) | old code put an interrupt on GPIO 19 — which is the AFE4400 CS — a direct conflict; the robust approach is to poll/drain the FIFO (as NEXT does) |

The MAX30001 CS (5), AFE4400 PWDN (18), SPI0 (2/3/4) and I²C1 (6/7) pins were
already correct in the old sketch.
