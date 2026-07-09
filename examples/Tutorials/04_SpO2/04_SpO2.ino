/*
 * HealthyPi 5 — Tutorial 04: SpO2 with no-finger detection
 * ============================================================================
 * Compute blood-oxygen saturation (SpO2) and pulse rate from the AFE4400, and
 * print them to the Serial Monitor — with proper handling for "no finger" and
 * "signal not yet valid" instead of showing a misleading ~100 %.
 *
 *   Board    : "Raspberry Pi Pico"  (arduino-pico core)
 *   Sensor   : AFE4400 PPG/SpO2 AFE, on SPI0
 *   Library  : "ProtoCentral AFE4490 PPG and SpO2 boards library"
 *              (its afe44xx driver computes SpO2 internally from IR/RED)
 *   Also needs: the in-repo HealthyPi5 library, for board.h (the pin map)
 *   Output   : Arduino IDE  ->  Tools > Serial Monitor  @ 115200 baud
 *
 * HOW SpO2 WORKS (the short version): oxygenated and deoxygenated haemoglobin
 * absorb infrared and red light differently. The AFE measures the pulsatile
 * (AC) and steady (DC) parts of both colors; the "ratio of ratios"
 * R = (AC_red/DC_red)/(AC_ir/DC_ir) maps to SpO2. The library does this and
 * hands back data.spo2 once it has buffered enough beats.
 *
 * WHY NO-FINGER DETECTION MATTERS: with no finger, the AC is just noise and the
 * ratio is meaningless — a naive reading often shows ~100 %. We gate on the IR
 * DC level: below a threshold there is no finger, so we report that honestly
 * rather than a fake number. TUNE the threshold for your board (see below).
 */
#include <SPI.h>
#include "protocentral_afe44xx.h"
#include <board.h>

AFE44XX      afe(HPI_PIN_AFE4400_CS, HPI_PIN_AFE4400_PWDN);
afe44xx_data data;

// No-finger gate: a fingertip raises the IR DC level well above the empty
// baseline. Watch data.IR_data with and without a finger and set this between
// the two. 50000 is a sensible starting point for the HealthyPi 5 AFE4400.
const long FINGER_IR_MIN = 50000;   // TUNE per board

void setup() {
  Serial.begin(115200);

  pinMode(HPI_PIN_AFE4400_CS, OUTPUT);
  digitalWrite(HPI_PIN_AFE4400_CS, HIGH);
  // The MAX30001 (ECG/BioZ) shares this SPI0 bus. We don't use it here, but its
  // CS must be driven HIGH (deselected) — left floating it partially responds to
  // the AFE's SPI traffic and drives MISO, corrupting every AFE read (SpO2 = 0).
  pinMode(HPI_PIN_MAX30001_CS, OUTPUT);
  digitalWrite(HPI_PIN_MAX30001_CS, HIGH);
  SPI.setRX(HPI_PIN_SPI0_MISO);
  SPI.setTX(HPI_PIN_SPI0_MOSI);
  SPI.setSCK(HPI_PIN_SPI0_SCK);
  SPI.begin();

  afe.afe44xx_init();
  Serial.println("HealthyPi 5 SpO2 — place a fingertip on the sensor...");
}

void loop() {
  // Read the AFE at ~125 Hz; the library flags buffer_count_overflow once it has
  // gathered enough samples to have a fresh SpO2/HR estimate (~once per second).
  static uint32_t last_ms = 0;
  if (millis() - last_ms < 8) return;
  last_ms = millis();

  afe.get_AFE44XX_Data(&data);

  bool finger_present = (data.IR_data > FINGER_IR_MIN);

  if (!finger_present) {
    // Rate-limit the "no finger" message so it doesn't flood the monitor.
    static uint32_t last_msg = 0;
    if (millis() - last_msg >= 1000) {
      last_msg = millis();
      Serial.println("No finger detected — SpO2 = -- (place a fingertip)");
    }
    return;
  }

  // A fresh estimate is ready only when the library sets this flag.
  if (data.buffer_count_overflow) {
    data.buffer_count_overflow = false;

    // The driver uses -999 as a "could not compute" sentinel (poor signal).
    if (data.spo2 == -999) {
      Serial.println("SpO2: --  (signal too weak — hold still, warm the finger)");
    } else {
      Serial.print("SpO2: ");
      Serial.print(data.spo2);
      Serial.print(" %    Pulse: ");
      Serial.print(data.heart_rate);
      Serial.println(" bpm");
    }
  }
}
