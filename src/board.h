/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 ProtoCentral Electronics — HealthyPi 5 NEXT firmware. */
/*
 * HealthyPi 5 board — RP2040 pin map and bus assignments.
 *
 * Carried over VERBATIM from the HealthyPi 5 NEXT firmware
 * (healthypi5_next_rp2040/include/board.h), which in turn took it from the
 * Zephyr board DTS. This guarantees the Arduino firmware uses the exact same
 * pins as the production NEXT and Zephyr firmwares.
 *
 * Applies to HealthyPi 5 revisions 5.2 through 5.7 — they are electrically
 * identical, so this one pin map is correct for all of them.
 *
 * === SINGLE SOURCE OF TRUTH FOR PINS ===
 * Sketches and libraries must reference these HPI_PIN_* macros and never
 * hardcode a GPIO number. To re-pin the board, edit only this header.
 *
 * (The pico-sdk-only bits of the NEXT header — bus instance handles and SPI
 * device-descriptor macros — are omitted here because the Arduino core
 * addresses SPI via the SPI/SPI1 objects, not bus instances. Pin NUMBERS,
 * speeds and modes are identical.)
 */
#ifndef HPI_BOARD_H
#define HPI_BOARD_H

/* ===== SPI0 — biosignal sensors (ECG + PPG) ===== */
#define HPI_PIN_SPI0_SCK        2
#define HPI_PIN_SPI0_MOSI       3       /* TX */
#define HPI_PIN_SPI0_MISO       4       /* RX */
#define HPI_PIN_MAX30001_CS     5       /* MAX30001 ECG/BioZ/RTOR */
#define HPI_PIN_AFE4400_CS      19      /* AFE4400 PPG  (old Arduino sketch said 27 — WRONG) */
#define HPI_PIN_AFE4400_PWDN    18      /* active-low: LOW = reset, HIGH = run */
#define HPI_MAX30001_SPI_HZ     (4 * 1000 * 1000)
#define HPI_AFE4400_SPI_HZ      (8 * 1000 * 1000)
/* SPI modes (mode = (CPOL<<1)|CPHA), preserved from the Zephyr drivers:
 *   MAX30001 = mode 0 (CPOL=0,CPHA=0);  AFE4400 = mode 3 (CPOL=1,CPHA=1). */
#define HPI_MAX30001_SPI_MODE   0
#define HPI_AFE4400_SPI_MODE    3

/* ===== I2C1 — temperature / fuel gauge / auth ===== */
#define HPI_PIN_I2C1_SDA        6
#define HPI_PIN_I2C1_SCL        7
#define HPI_I2C1_HZ             (100 * 1000)
#define HPI_ADDR_MAX17048       0x36    /* fuel gauge (absent without a battery) */
#define HPI_ADDR_MAX30205       0x49    /* temperature — older kits (QWIIC): 1/256 C/LSB */
#define HPI_ADDR_AS6221         0x48    /* temperature — newer kits (QWIIC): 1/128 C/LSB */
#define HPI_ADDR_ATECC608A      0x6A    /* crypto auth (unused for now) */

/* Both temperature sensors expose the temperature in register 0x00 as a signed
 * 16-bit big-endian value; only the I2C address and the per-LSB resolution
 * differ (MAX30205 = 1/256 C, AS6221 = 1/128 C). Boot code probes for whichever
 * is on the QWIIC connector — see HPII2c.cpp (firmware) and 06_Temperature. */
#define HPI_TEMP_REG            0x00

/* ===== SPI1 — SD card + LCD (CS-muxed) ===== */
#define HPI_PIN_SPI1_SCK        10
#define HPI_PIN_SPI1_MOSI       11      /* TX */
#define HPI_PIN_SPI1_MISO       8       /* RX */
#define HPI_PIN_LCD_CS          29
#define HPI_PIN_LCD_DC          23      /* data/command select (high = data) */
#define HPI_PIN_LCD_RST         28      /* reset (active low) */
#define HPI_PIN_SD_CS           13      /* old Arduino sketch said 16 — WRONG (16 is the backlight) */
#define HPI_SD_SPI_HZ           (2 * 1000 * 1000)
#define HPI_LCD_SPI_HZ          (24 * 1000 * 1000)

/* ===== UART1 — link to the ESP32-C3 wireless co-processor ===== */
#define HPI_PIN_UART1_TX        24      /* RP2040 TX -> ESP32 RX */
#define HPI_PIN_UART1_RX        25      /* RP2040 RX <- ESP32 TX */
#define HPI_PIN_UART1_RTS       27
#define HPI_PIN_UART1_CTS       26
#define HPI_UART1_BAUD          921600

/* ===== UART0 — debug console ===== */
#define HPI_PIN_UART0_TX        0
#define HPI_PIN_UART0_RX        1
#define HPI_UART0_BAUD          115200

/* ===== GPIO keys (active-low) — single thumb-wheel: Up / OK / Down ===== */
#define HPI_PIN_KEY_UP          15      /* sw1 */
#define HPI_PIN_KEY_OK          14      /* sw2 */
#define HPI_PIN_KEY_DOWN        12      /* sw3 */

/* ===== Misc ===== */
#define HPI_PIN_LCD_BACKLIGHT   16      /* PWM */

/* ===== Sampling parameters ===== */
#define HPI_ECG_SPS             128     /* MAX30001 ECG output data rate */
#define HPI_BIOZ_SPS            64      /* MAX30001 BioZ output data rate */
#define HPI_PPG_DIVIDER         2       /* read AFE4400 every Nth ECG cycle (~64 SPS PPG) */
#define HPI_ACQ_PERIOD_US       (1000000 / HPI_ECG_SPS)  /* ~7813 us */

#endif /* HPI_BOARD_H */
