/*
 * HealthyPi 5 — Tutorial 05: Heart rate from the MAX30001
 * ============================================================================
 * Print the heart rate (beats per minute) and the R-to-R interval (time between
 * heartbeats) using the MAX30001's built-in hardware R-peak detector. Unlike a
 * software peak detector, the MAX30001 finds the QRS complex on-chip and reports
 * each beat — you just read the result.
 *
 *   Board    : "Raspberry Pi Pico"  (arduino-pico core)
 *   Sensor   : MAX30001, on SPI0  (RtoR / heart-rate engine)
 *   Library  : "ProtoCentral MAX30001" v2.0.0+
 *   Also needs: the in-repo HealthyPi5 library, for board.h
 *   Output   : Arduino IDE  ->  Tools > Serial Monitor  @ 115200 baud
 *
 * HOW TO RUN
 *   1. Attach the 3 ECG electrodes (as in example 01).
 *   2. Upload, open the Serial Monitor at 115200. After a few beats settle, each
 *      detected heartbeat prints a fresh HR and R-R interval.
 *
 * NOTE: at startup (and if electrodes are loose) the chip reports 0xFFFF / an
 * out-of-range value until it locks onto a stable rhythm — we show "--" for that
 * instead of a nonsense number. R-to-R (ms) and HR (bpm) are related by
 * HR = 60000 / R-R.
 */
#include <SPI.h>
#include <protocentral_max30001.h>
#include <board.h>

MAX30001 max30001(HPI_PIN_MAX30001_CS);

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
  max30001.startECGBioZ(MAX30001_RATE_128);   // enables the R-to-R (beat) engine
  Serial.println("HealthyPi 5 heart rate — attach ECG electrodes and wait a few beats...");
}

void loop() {
  // Poll the R-to-R result ~50x/second. rr_detected is true only on a new beat,
  // so this prints once per heartbeat rather than continuously.
  static uint32_t last_ms = 0;
  if (millis() - last_ms < 20) return;
  last_ms = millis();

  max30001_rtor_data_t rr;
  if (max30001.getRtoRData(&rr) == MAX30001_SUCCESS && rr.rr_detected) {
    uint16_t bpm = rr.heart_rate_bpm;
    if (bpm == 0xFFFF || bpm > 300) {
      Serial.println("HR: --   (no stable rhythm yet)");
    } else {
      Serial.print("HR: ");
      Serial.print(bpm);
      Serial.print(" bpm    R-R: ");
      Serial.print(rr.rr_interval_ms);
      Serial.println(" ms");
    }
  }
}
