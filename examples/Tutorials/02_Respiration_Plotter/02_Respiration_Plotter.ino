/*
 * HealthyPi 5 — Tutorial 02: Respiration (BioZ) on the Serial Plotter
 * ============================================================================
 * Plot the breathing waveform derived from the MAX30001's bio-impedance (BioZ)
 * channel. As the chest expands and contracts, the electrical impedance across
 * the ECG electrodes changes slightly — that slow rise and fall IS respiration
 * (this technique is called impedance pneumography).
 *
 *   Board    : "Raspberry Pi Pico"  (arduino-pico core)
 *   Sensor   : MAX30001 BioZ channel, on SPI0
 *   Library  : "ProtoCentral MAX30001" v2.0.0+
 *   Also needs: the in-repo HealthyPi5 library, for board.h
 *   Output   : Arduino IDE 2.x  ->  Tools > Serial Plotter  @ 115200 baud
 *
 * HOW TO RUN
 *   1. Attach the ECG electrodes (the SAME 3 electrodes as example 01 — BioZ
 *      drives a tiny current through them and measures the impedance).
 *   2. Upload, open the Serial Plotter at 115200, and breathe: you should see a
 *      slow wave (~0.2-0.4 Hz, i.e. 12-24 breaths/min) rise and fall.
 *
 * We start the MAX30001 in combined ECG+BioZ mode and read the BioZ channel; the
 * ECG channel simply isn't used here (see examples 01/05 for ECG and heart rate).
 * A gentle low-pass smooths the trace so the breathing curve is easy to see.
 *
 * As well as the waveform we turn it into a NUMBER — the respiration rate in
 * breaths/min — using resp_process.c from the HealthyPi5 library (the SAME
 * adaptive threshold-crossing detector the dual-core HealthyPi5_NEXT uses). It is
 * plotted as a second series; note the plotter shares one Y axis, so the small
 * RR value sits near the bottom under the much larger waveform. RR reads 0 for
 * the first ~10 s and until at least 3 breaths have been detected.
 */
#include <SPI.h>
#include <protocentral_max30001.h>
#include <board.h>
#include <resp_process.h>   // BioZ respiration detector (HealthyPi5 lib)

MAX30001 max30001(HPI_PIN_MAX30001_CS);

// Exponential moving average (low-pass): breathing is slow, so smooth out the
// faster fluctuations for a clean respiration curve. Raise ALPHA for less smoothing.
static float resp_lp = 0.0f;
const float  LP_ALPHA = 0.10f;

// Plot decimation. We ACQUIRE at 128 SPS but only PRINT one line every 25th
// sample (~5 Hz). Breathing is ~0.2-0.4 Hz, so at 128 SPS the Arduino Serial
// Plotter's small window (~50 points in IDE 2.x) holds under half a second —
// far less than one breath. At ~5 Hz that same window spans ~10 s, so 2-3 full
// breaths are visible. Only the DISPLAY is decimated; the low-pass and the RR
// detector below still run on every sample.
const uint8_t PLOT_EVERY = 25;

// Respiration RATE from the same BioZ samples. The detector must be fed at
// ~32 Hz — 128 SPS / 4 — which its RESP_CALIBRATION_FACTOR assumes; we read BioZ
// at the 128 SPS pace below, so batching by 4 hits exactly that. Updates `rr`.
uint8_t  rr = 0;                 // breaths/min, 0 = not available yet
static int16_t resp_batch[4];
static uint8_t resp_fill = 0;
void feed_respiration(int32_t bioz_sample) {
  resp_batch[resp_fill++] = (int16_t)bioz_sample;
  if (resp_fill >= 4) {
    int16_t filtered[4];
    resp_process_sample(resp_batch, filtered);   // pass-through (optional LPF)
    resp_algo_process(filtered, &rr);
    resp_fill = 0;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(HPI_PIN_MAX30001_CS, OUTPUT);
  digitalWrite(HPI_PIN_MAX30001_CS, HIGH);
  // The AFE4400 (PPG) shares this SPI0 bus. Deselect it (CS HIGH) even though we
  // don't use it, so it can't drive MISO and corrupt the MAX30001 reads.
  pinMode(HPI_PIN_AFE4400_CS, OUTPUT);
  digitalWrite(HPI_PIN_AFE4400_CS, HIGH);
  SPI.setRX(HPI_PIN_SPI0_MISO);
  SPI.setTX(HPI_PIN_SPI0_MOSI);
  SPI.setSCK(HPI_PIN_SPI0_SCK);
  SPI.begin();

  max30001.begin();
  max30001.startECGBioZ(MAX30001_RATE_128);   // ECG + BioZ; we read BioZ below
}

void loop() {
  // Pace to the acquisition rate (one FIFO read per period, as in example 01).
  static uint32_t next_us = micros();
  if ((int32_t)(micros() - next_us) < 0) return;
  next_us += HPI_ACQ_PERIOD_US;

  max30001_bioz_sample_t bioz;
  if (max30001.getBioZSample(&bioz) == MAX30001_SUCCESS) {
    resp_lp += LP_ALPHA * ((float)bioz.bioz_sample - resp_lp);   // low-pass (full rate)
    feed_respiration(bioz.bioz_sample);                          // raw -> rate (full rate)

    // Print at ~5 Hz so a couple of breaths fit the plotter window (see PLOT_EVERY).
    static uint8_t plot_n = 0;
    if (++plot_n >= PLOT_EVERY) {
      plot_n = 0;
      Serial.print("Respiration:");
      Serial.print((int32_t)resp_lp);
      Serial.print("\tRR_bpm:");
      Serial.println(rr);
    }
  }
}
