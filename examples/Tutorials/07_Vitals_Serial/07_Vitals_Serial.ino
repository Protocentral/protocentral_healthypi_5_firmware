/*
 * HealthyPi 5 — Tutorial 07: All vitals to the Serial Monitor
 * ============================================================================
 * Bring up all three sensors at once and print a one-line vitals summary every
 * second: heart rate (MAX30001), SpO2 (AFE4400), and temperature (MAX30205).
 * This shows how the individual examples (01/03/04/05/06) combine into a simple
 * multi-sensor readout — the foundation of a patient monitor.
 *
 *   Board    : "Raspberry Pi Pico"  (arduino-pico core)
 *   Sensors  : MAX30001 (SPI0), AFE4400 (SPI0), MAX30205 or AS6221 (I2C1/QWIIC)
 *   Libraries: "ProtoCentral MAX30001" v2.0.0+,
 *              "ProtoCentral AFE4490 PPG and SpO2 boards library"
 *   Also needs: the in-repo HealthyPi5 library, for board.h
 *   Output   : Arduino IDE  ->  Tools > Serial Monitor  @ 115200 baud
 *
 * The MAX30001 and AFE4400 share SPI0 with different SPI modes; each library
 * applies its own settings per transaction, so we can read them one after the
 * other in the same loop. Missing values show as "--".
 *
 * Respiration rate (breaths/min) is derived from the BioZ waveform (example 02)
 * by resp_process.c — the SAME adaptive threshold-crossing detector the
 * dual-core HealthyPi5_NEXT uses. It needs the BioZ fed at a steady ~32 Hz, so we
 * add a 128 SPS paced read below (BioZ is a 64 SPS channel, so we read it every
 * 2nd tick and HOLD it in between, exactly like the HealthyPi5 library). RR
 * reads 0 for the first ~10 s and until at least 3 breaths are detected.
 */
#include <SPI.h>
#include <Wire.h>
#include <protocentral_max30001.h>
#include "protocentral_afe44xx.h"
#include <board.h>
#include <resp_process.h>   // BioZ respiration detector (HealthyPi5 lib)

MAX30001      max30001(HPI_PIN_MAX30001_CS);
AFE44XX       afe(HPI_PIN_AFE4400_CS, HPI_PIN_AFE4400_PWDN);
afe44xx_data  ppg;

const long FINGER_IR_MIN = 50000;    // no-finger gate (see example 04); TUNE

uint16_t hr = 0;          // 0 = not available
uint8_t  spo2 = 0;        // 0 = not available / no finger
uint8_t  rr = 0;          // respiration, breaths/min (0 = not available yet)
int32_t  g_bioz = 0;      // latest BioZ, held between 64 SPS reads
float    temp_c = 0.0f;
bool     temp_present = false;
uint8_t  temp_addr = 0;   // detected QWIIC temp sensor address (0 = none)
float    temp_lsb  = 0.0f; // C per LSB: MAX30205 1/256, AS6221 1/128

// Push the (held) BioZ to the respiration detector; it batches by 4 internally
// -> the 32 Hz the RESP_CALIBRATION_FACTOR assumes. Updates `rr` in place.
static int16_t resp_batch[4];
static uint8_t resp_fill = 0;
void feed_respiration(int32_t bioz_held) {
  resp_batch[resp_fill++] = (int16_t)bioz_held;
  if (resp_fill >= 4) {
    int16_t filtered[4];
    resp_process_sample(resp_batch, filtered);   // pass-through (optional LPF)
    resp_algo_process(filtered, &rr);
    resp_fill = 0;
  }
}

void setup() {
  Serial.begin(115200);

  // ---- SPI0: MAX30001 + AFE4400 ----
  pinMode(HPI_PIN_MAX30001_CS, OUTPUT); digitalWrite(HPI_PIN_MAX30001_CS, HIGH);
  pinMode(HPI_PIN_AFE4400_CS,  OUTPUT); digitalWrite(HPI_PIN_AFE4400_CS,  HIGH);
  SPI.setRX(HPI_PIN_SPI0_MISO);
  SPI.setTX(HPI_PIN_SPI0_MOSI);
  SPI.setSCK(HPI_PIN_SPI0_SCK);
  SPI.begin();

  // ---- I2C1 / QWIIC: temperature sensor (MAX30205 or AS6221, optional) ----
  Wire1.setSDA(HPI_PIN_I2C1_SDA);
  Wire1.setSCL(HPI_PIN_I2C1_SCL);
  Wire1.setClock(HPI_I2C1_HZ);
  Wire1.begin();
  // Detect which QWIIC temperature sensor is present (MAX30205 or AS6221).
  Wire1.beginTransmission(HPI_ADDR_MAX30205);
  if (Wire1.endTransmission() == 0) { temp_addr = HPI_ADDR_MAX30205; temp_lsb = 1.0f / 256; }
  else {
    Wire1.beginTransmission(HPI_ADDR_AS6221);
    if (Wire1.endTransmission() == 0) { temp_addr = HPI_ADDR_AS6221; temp_lsb = 1.0f / 128; }
  }
  temp_present = (temp_addr != 0);

  afe.afe44xx_init();
  max30001.begin();
  max30001.startECGBioZ(MAX30001_RATE_128);
}

void loop() {
  // --- heart rate: poll the R-to-R engine ~50 Hz (updates once per beat) ---
  static uint32_t last_hr = 0;
  if (millis() - last_hr >= 20) {
    last_hr = millis();
    max30001_rtor_data_t rtor;
    if (max30001.getRtoRData(&rtor) == MAX30001_SUCCESS && rtor.rr_detected) {
      uint16_t bpm = rtor.heart_rate_bpm;
      hr = (bpm == 0xFFFF || bpm > 300) ? 0 : bpm;
    }
  }

  // --- SpO2: read the AFE ~125 Hz; new estimate on buffer_count_overflow ---
  static uint32_t last_ppg = 0;
  if (millis() - last_ppg >= 8) {
    last_ppg = millis();
    afe.get_AFE44XX_Data(&ppg);
    if (ppg.IR_data <= FINGER_IR_MIN) {
      spo2 = 0;                                    // no finger
    } else if (ppg.buffer_count_overflow) {
      spo2 = (ppg.spo2 == -999) ? 0 : (uint8_t)ppg.spo2;
      ppg.buffer_count_overflow = false;
    }
  }

  // --- respiration: pace one BioZ read per 128 SPS tick, feed the detector ---
  // BioZ is a 64 SPS channel, so read it every 2nd tick and hold in between
  // (matches the HealthyPi5 library); feed the held value every tick at 128 SPS.
  static uint32_t next_bioz_us = micros();
  if ((int32_t)(micros() - next_bioz_us) >= 0) {
    next_bioz_us += HPI_ACQ_PERIOD_US;
    static uint8_t bioz_phase = 0;
    if ((bioz_phase++ & 1) == 0) {
      max30001_bioz_sample_t bs;
      if (max30001.getBioZSample(&bs) == MAX30001_SUCCESS) g_bioz = bs.bioz_sample;
    }
    feed_respiration(g_bioz);
  }

  // --- temperature: 1 Hz ---
  static uint32_t last_temp = 0;
  if (temp_present && millis() - last_temp >= 1000) {
    last_temp = millis();
    Wire1.beginTransmission(temp_addr);
    Wire1.write(HPI_TEMP_REG);             // 0x00 on both sensors
    if (Wire1.endTransmission(false) == 0 &&
        Wire1.requestFrom(temp_addr, (uint8_t)2) >= 2) {
      int16_t raw = ((int16_t)Wire1.read() << 8) | Wire1.read();
      temp_c = raw * temp_lsb;             // scale by the detected sensor's LSB
    }
  }

  // --- print the combined vitals line once a second ---
  static uint32_t last_print = 0;
  if (millis() - last_print >= 1000) {
    last_print = millis();
    Serial.print("HR: ");
    if (hr) Serial.print(hr); else Serial.print("--");
    Serial.print(" bpm    SpO2: ");
    if (spo2) Serial.print(spo2); else Serial.print("--");
    Serial.print(" %    RR: ");
    if (rr) Serial.print(rr); else Serial.print("--");
    Serial.print(" brpm    Temp: ");
    if (temp_present) { Serial.print(temp_c, 1); Serial.print(" C"); }
    else              { Serial.print("-- (no sensor)"); }
    Serial.println();
  }
}
