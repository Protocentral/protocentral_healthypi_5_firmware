/*
 * HealthyPi 5 — Tutorial 03: PPG (IR + RED) on the Serial Plotter
 * ============================================================================
 * Read the two photoplethysmogram (PPG) channels — infrared and red — from the
 * AFE4400 pulse-oximetry analog front end, and plot their pulse waveforms live.
 *
 *   Board    : "Raspberry Pi Pico"  (arduino-pico core)
 *   Sensor   : AFE4400 PPG/SpO2 AFE, on SPI0  (NOT a MAX3010x!)
 *   Library  : "ProtoCentral AFE4490 PPG and SpO2 boards library"
 *              (provides protocentral_afe44xx.h; the AFE4400 is the same family)
 *   Also needs: the in-repo HealthyPi5 library, for board.h (the pin map)
 *   Output   : Arduino IDE 2.x  ->  Tools > Serial Plotter  @ 115200 baud
 *
 * HOW TO RUN
 *   1. Rest a fingertip gently on the HealthyPi 5 PPG/SpO2 sensor.
 *   2. Upload, open Tools > Serial Plotter at 115200.
 *   3. You should see two pulsatile waveforms (IR and RED) beating with the
 *      heart — each with the characteristic PPG upstroke and dicrotic notch.
 *
 * WHY HIGH-PASS THE PPG?  A raw PPG channel is mostly a large, slowly-varying DC
 * level (how much light reaches the detector) with a small pulsatile "AC" riding
 * on top — that AC IS the pulse. Plotting raw shows two nearly-flat lines. A
 * one-pole high-pass removes the DC so the pulse morphology fills the plot.
 * (SpO2 itself is computed from the DC and AC of both colors — see example 04.)
 */
#include <SPI.h>
#include "protocentral_afe44xx.h"
#include <board.h>

AFE44XX      afe(HPI_PIN_AFE4400_CS, HPI_PIN_AFE4400_PWDN);
afe44xx_data data;

// One-pole high-pass state for each channel (removes the DC light level).
static float ir_in = 0, ir_out = 0, red_in = 0, red_out = 0;
const float  HP_ALPHA = 0.97f;    // lower cutoff than ECG — the pulse is ~1-3 Hz

// Plot decimation. We read/filter the AFE at ~125 Hz but only PRINT one line
// every 5th sample (~25 Hz). The pulse is ~1-1.5 Hz, so at 125 Hz the Arduino
// Serial Plotter's small window (~50 points in IDE 2.x) shows under half a
// pulse; at ~25 Hz it shows ~2 pulses while still resolving the waveform shape.
// The AFE read and the high-pass filter still run at the full 125 Hz.
const uint8_t PLOT_EVERY = 5;

void setup() {
  Serial.begin(115200);

  // ---- SPI0 (shared bus; the AFE4400 uses SPI mode 3, handled by the library) ----
  pinMode(HPI_PIN_AFE4400_CS, OUTPUT);
  digitalWrite(HPI_PIN_AFE4400_CS, HIGH);
  // The MAX30001 (ECG/BioZ) shares this SPI0 bus. We don't use it here, but its
  // CS must be driven HIGH (deselected) — left floating it partially responds to
  // the AFE's SPI traffic and drives MISO, corrupting every AFE read (PPG = 0).
  pinMode(HPI_PIN_MAX30001_CS, OUTPUT);
  digitalWrite(HPI_PIN_MAX30001_CS, HIGH);
  SPI.setRX(HPI_PIN_SPI0_MISO);
  SPI.setTX(HPI_PIN_SPI0_MOSI);
  SPI.setSCK(HPI_PIN_SPI0_SCK);
  SPI.begin();

  afe.afe44xx_init();               // reset + configure the AFE4400
}

void loop() {
  // Read the AFE at ~125 Hz — plenty to resolve the pulse waveform.
  static uint32_t last_ms = 0;
  if (millis() - last_ms < 8) return;
  last_ms = millis();

  afe.get_AFE44XX_Data(&data);
  float ir  = (float)data.IR_data;
  float red = (float)data.RED_data;

  // high-pass each channel (full rate; comment out to plot the raw DC-coupled signals)
  float ir_y  = HP_ALPHA * (ir_out  + ir  - ir_in);   ir_in  = ir;  ir_out  = ir_y;
  float red_y = HP_ALPHA * (red_out + red - red_in);  red_in = red; red_out = red_y;

  // Print at ~25 Hz so a couple of pulses fit the plotter window (see PLOT_EVERY).
  static uint8_t plot_n = 0;
  if (++plot_n >= PLOT_EVERY) {
    plot_n = 0;
    // Two named traces on the Serial Plotter, comma-separated.
    Serial.print("IR:");
    Serial.print((int32_t)ir_y);
    Serial.print(",RED:");
    Serial.println((int32_t)red_y);
  }
}
