/*
 * HealthyPi 5 — Tutorial 01: ECG on the Arduino Serial Plotter
 * ============================================================================
 * The simplest possible HealthyPi 5 sketch: read the ECG waveform from the
 * MAX30001 biopotential AFE and plot it live in the Arduino IDE Serial Plotter.
 * Read it top to bottom — there is no hidden magic.
 *
 *   Board    : "Raspberry Pi Pico"  (Earle Philhower arduino-pico core)
 *   Sensor   : MAX30001 (single-lead ECG / BioZ / heart-rate), on SPI0
 *   Library  : "ProtoCentral MAX30001" v2.0.0+  (Library Manager)
 *   Also needs: the in-repo HealthyPi5 library, for board.h (the pin map)
 *   Output   : Arduino IDE 2.x  ->  Tools > Serial Plotter  @ 115200 baud
 *
 * HOW TO RUN
 *   1. Attach 3 ECG electrodes (RA / LA / RL) to the HealthyPi 5 ECG connector.
 *   2. Upload, then open Tools > Serial Plotter and set the baud to 115200.
 *   3. You should see the classic ECG trace (P-QRS-T) scroll across.
 *
 * WHY A HIGH-PASS FILTER?  The raw MAX30001 sample carries slow "baseline
 * wander" (breathing, electrode drift) as a large DC-ish offset. Left in, the
 * plot drifts up and down and the beats look tiny. A one-pole high-pass removes
 * that drift so the P-QRS-T morphology fills the plot. This is a great first
 * lesson in why biosignals get filtered — comment it out to see the raw signal.
 */
#include <SPI.h>
#include <protocentral_max30001.h>
#include <board.h>            // HealthyPi 5 pin map (never hard-code a GPIO)

MAX30001 ecg(HPI_PIN_MAX30001_CS);

// One-pole high-pass filter state (baseline removal). y[n] = a*(y[n-1]+x[n]-x[n-1])
static float hp_in = 0.0f, hp_out = 0.0f;
const float  HP_ALPHA = 0.95f;    // ~1 Hz cutoff at 128 SPS; raise toward 1.0 for less

void setup() {
  Serial.begin(115200);

  // ---- bring up SPI0 (the bus the MAX30001 lives on) ----
  pinMode(HPI_PIN_MAX30001_CS, OUTPUT);
  digitalWrite(HPI_PIN_MAX30001_CS, HIGH);   // deselect the chip
  // The AFE4400 (PPG) shares this SPI0 bus. Deselect it (CS HIGH) even though we
  // don't use it, so it can't drive MISO and corrupt the MAX30001 reads.
  pinMode(HPI_PIN_AFE4400_CS, OUTPUT);
  digitalWrite(HPI_PIN_AFE4400_CS, HIGH);
  SPI.setRX(HPI_PIN_SPI0_MISO);
  SPI.setTX(HPI_PIN_SPI0_MOSI);
  SPI.setSCK(HPI_PIN_SPI0_SCK);
  SPI.begin();

  // ---- start the MAX30001 streaming ECG at 128 samples/second ----
  ecg.begin();                        // probe + reset the chip
  ecg.startECG(MAX30001_RATE_128);
}

void loop() {
  // Pace the loop to the MAX30001's 128 SPS output rate. The v2.0.0 library
  // returns ONE FIFO sample per getECGSample() call and does not signal "empty",
  // so we read exactly one sample per output period instead of polling flat-out.
  static uint32_t next_us = micros();
  if ((int32_t)(micros() - next_us) < 0) return;
  next_us += HPI_ACQ_PERIOD_US;        // ~7813 us  (1 / 128 SPS)

  max30001_ecg_sample_t sample;
  if (ecg.getECGSample(&sample) == MAX30001_SUCCESS) {
    float x = (float)sample.ecg_sample;

    // high-pass (baseline removal) — comment these 3 lines out to plot the raw ECG
    float y = HP_ALPHA * (hp_out + x - hp_in);
    hp_in = x;
    hp_out = y;

    // We print every sample (full 128 SPS): the QRS complex is fast (~80-100 ms),
    // so decimating would distort its shape. The trade-off is that the Arduino
    // Serial Plotter's small window (~50 points in IDE 2.x) then holds under one
    // beat. For a wider view, use OpenView 2 or the legacy IDE 1.8 plotter
    // (500-point window). (The slower PPG/respiration plots CAN be decimated —
    // see the PLOT_EVERY note in examples 02 and 03.)
    // Serial Plotter format: "label:value" gives the trace a name in the legend.
    Serial.print("ECG:");
    Serial.println((int32_t)y);

    // Tip: sample.lead_off_detected is true when an electrode is loose.
  }
}
