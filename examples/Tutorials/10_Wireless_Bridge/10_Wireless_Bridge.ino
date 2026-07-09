/*
 * HealthyPi 5 — Tutorial 10: Wireless via the ESP32-C3 (HealthyBridge)
 * ============================================================================
 * The big idea: **the RP2040 on the HealthyPi 5 has no radio.** It is a plain
 * RP2040 (not a Pico W), so there is no on-chip BLE or Wi-Fi. All wireless is
 * done by the on-board **ESP32-C3 co-processor**, which runs its OWN firmware
 * (Protocentral/healthybridge-esp32) and owns the entire BLE stack (and Wi-Fi).
 *
 * So an "Arduino BLE example" on this board never calls BLEDevice.begin(). The
 * RP2040's job is only to HAND its data to the ESP32 over a wire — the
 * **HealthyBridge** UART link (UART1 @ 921600, RTS/CTS). The ESP32 then relays
 * it to a phone over BLE or to a browser over Wi-Fi. BLE and Wi-Fi are the SAME
 * on this side: identical frames go out UART1; the ESP32 (or the host) chooses
 * the radio. That is why there is one wireless example, not two.
 *
 *   Board    : "Raspberry Pi Pico"  (arduino-pico core)
 *   Sensors  : MAX30001 (SPI0), AFE4400 (SPI0), MAX30205 or AS6221 (I2C1/QWIIC)
 *   Link     : UART1 (Serial2) GP24 TX / GP25 RX / GP27 RTS / GP26 CTS @ 921600
 *   Libraries: "ProtoCentral MAX30001" v2.0.0+,
 *              "ProtoCentral AFE4490 PPG and SpO2 boards library"
 *   Also needs: the in-repo HealthyPi5 library, for board.h
 *   Output   : Tools > Serial Monitor @ 115200 shows what is being sent; the
 *              vitals themselves travel over UART1 to the ESP32-C3.
 *
 * The frame format below is a MINIMAL, byte-identical copy of the HealthyBridge
 * VITALS frame so this sketch is self-contained and you can read the whole
 * packet in one file. The CANONICAL definition (all frame types, the biosignal
 * stream, RX/status parsing, drop-newest transport) lives in the HealthyPi5
 * library: libraries/HealthyPi5/src/HPIBridge.h / .cpp. The production firmware
 * (examples/Applications/HealthyPi5_NEXT) uses that; keep the two in sync if you
 * change the wire format.
 *
 * To actually see the data on a phone/browser, flash the companion ESP32-C3
 * firmware (Protocentral/healthybridge-esp32) to the on-board ESP32. See this
 * example's README and docs/WIRELESS.md.
 */
#include <SPI.h>
#include <Wire.h>
#include <protocentral_max30001.h>
#include "protocentral_afe44xx.h"
#include <board.h>
#include <resp_process.h>   // BioZ respiration detector (HealthyPi5 lib)

/* ===== HealthyBridge frame (subset) — canonical: HPIBridge.h ===============
 * Frame (little-endian):
 *   SYNC(0xAA55) | TYPE(1) | FLAGS(1) | LENGTH(2) | SEQ(2) | PAYLOAD | CRC16(2)
 * CRC-16/CCITT (poly 0x1021, init 0xFFFF) covers TYPE..end-of-PAYLOAD. These
 * constants MUST match HPIBridge.h and the ESP32 side, or the link won't parse.
 */
#define HB_SYNC_WORD    0xAA55
#define HB_HEADER_SIZE  8            // sync(2)+type(1)+flags(1)+len(2)+seq(2)
#define HB_TYPE_VITALS  0x40         // computed HR/SpO2/RR/temp + lead-off
#define HB_FLAG_LAST    (1 << 2)
#define HB_UART_BAUD    921600

// VITALS payload — identical layout to struct hb_vitals_payload in HPIBridge.h.
struct hb_vitals_payload {
  uint16_t hr;             // beats/min (0 = invalid)
  uint8_t  spo2;           // % (0 = invalid / no finger)
  uint8_t  rr;             // breaths/min (0 = not available yet)
  int16_t  temp_c_x100;    // temperature * 100 (e.g. 3673 = 36.73 C)
  uint8_t  flags;          // HB_VITAL_* validity bits
} __attribute__((packed));

#define HB_VITAL_HR_VALID    (1 << 0)
#define HB_VITAL_SPO2_VALID  (1 << 1)
#define HB_VITAL_RR_VALID    (1 << 2)
#define HB_VITAL_ECG_LEADOFF (1 << 3)

// Set to 0 to bench-test WITHOUT an ESP32 attached: without hardware flow
// control the frames still go out GP24 (Serial2 TX) so you can sniff them with
// a logic analyser / USB-UART adapter. With an ESP32 present, keep this 1.
#define USE_FLOW_CONTROL 1

// ---- CRC-16/CCITT, byte-identical to the library / ESP32 (do not "optimise") ----
static uint16_t hb_crc16_ccitt(uint16_t seed, const uint8_t *src, size_t len) {
  for (; len > 0; len--) {
    uint8_t e = (uint8_t)(seed ^ *src++);
    uint8_t f = (uint8_t)(e ^ (e << 4));
    seed = (uint16_t)((seed >> 8) ^ ((uint16_t)f << 8) ^
                      ((uint16_t)f << 3) ^ ((uint16_t)f >> 4));
  }
  return seed;
}

static uint16_t s_seq = 0;
static uint32_t s_tx_frames = 0, s_tx_drops = 0;

// Build SYNC|type|flags|len|seq|payload|crc16 and write it to UART1.
// Bounded, byte-by-byte write: arduino-pico's Serial2.availableForWrite() is a
// WRITABLE FLAG (1 = FIFO has room, 0 = full), NOT a byte count. So we wait up
// to 2 ms per byte for room; if the link stalls that long (ESP absent / CTS
// deasserted) we drop the rest of the frame instead of blocking forever.
static void hb_send_vitals(const hb_vitals_payload &vp) {
  uint8_t frame[HB_HEADER_SIZE + sizeof(vp) + 2];
  uint16_t len = sizeof(vp), n = 0;

  frame[n++] = (uint8_t)HB_SYNC_WORD;          // 0x55
  frame[n++] = (uint8_t)(HB_SYNC_WORD >> 8);   // 0xAA
  frame[n++] = HB_TYPE_VITALS;
  frame[n++] = HB_FLAG_LAST;
  frame[n++] = (uint8_t)len;
  frame[n++] = (uint8_t)(len >> 8);
  frame[n++] = (uint8_t)s_seq;
  frame[n++] = (uint8_t)(s_seq >> 8);
  s_seq++;
  memcpy(frame + n, &vp, len); n += len;

  uint16_t crc = hb_crc16_ccitt(0xFFFF, &frame[2], (HB_HEADER_SIZE - 2) + len);
  frame[n++] = (uint8_t)crc;
  frame[n++] = (uint8_t)(crc >> 8);

  for (uint16_t i = 0; i < n; i++) {
    uint32_t t0 = micros();
    while (Serial2.availableForWrite() < 1) {
      if ((uint32_t)(micros() - t0) > 2000u) { s_tx_drops++; return; }
      yield();
    }
    Serial2.write(frame[i]);
  }
  s_tx_frames++;
}

// ===== sensors (same bring-up as example 07) =====
MAX30001      max30001(HPI_PIN_MAX30001_CS);
AFE44XX       afe(HPI_PIN_AFE4400_CS, HPI_PIN_AFE4400_PWDN);
afe44xx_data  ppg;

const long FINGER_IR_MIN = 50000;    // no-finger gate (see example 04); TUNE

uint16_t hr = 0;
uint8_t  spo2 = 0;
uint8_t  rr = 0;          // respiration, breaths/min (0 = not available yet)
uint8_t  leadoff = 0;     // 1 = an ECG electrode is off (from the MAX30001)
int32_t  g_bioz = 0;      // latest BioZ, held between 64 SPS reads
float    temp_c = 0.0f;
bool     temp_present = false;
uint8_t  temp_addr = 0;   // detected QWIIC temp sensor address (0 = none)
float    temp_lsb  = 0.0f; // C per LSB: MAX30205 1/256, AS6221 1/128

// Respiration from the BioZ waveform via resp_process.c (the SAME detector the
// dual-core HealthyPi5_NEXT uses). It must be fed at ~32 Hz — 128 SPS / 4 — which
// its RESP_CALIBRATION_FACTOR assumes; feed_respiration() batches by 4 and
// updates `rr`. RR reads 0 for the first ~10 s and until >=3 breaths are seen.
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
  Serial.begin(115200);              // USB console: shows what we transmit

  // ---- UART1: the HealthyBridge link to the ESP32-C3 ----
  Serial2.setTX(HPI_PIN_UART1_TX);
  Serial2.setRX(HPI_PIN_UART1_RX);
#if USE_FLOW_CONTROL
  Serial2.setRTS(HPI_PIN_UART1_RTS);
  Serial2.setCTS(HPI_PIN_UART1_CTS);
#endif
  Serial2.begin(HB_UART_BAUD);

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

  Serial.println("HealthyBridge wireless demo: streaming VITALS to the ESP32-C3");
  Serial.println("(RP2040 has no radio; the ESP32 relays these over BLE / Wi-Fi)");
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

  // --- respiration + lead-off: pace at 128 SPS (the ECG ODR) ---
  // BioZ is a 64 SPS channel, so read it every 2nd tick and hold in between
  // (matches the HealthyPi5 library); feed the held value every tick at 128 SPS.
  // We also read one ECG sample per tick for its lead-off flag (electrode off),
  // so the VITALS flags byte matches the library's on the wire.
  static uint32_t next_bioz_us = micros();
  if ((int32_t)(micros() - next_bioz_us) >= 0) {
    next_bioz_us += HPI_ACQ_PERIOD_US;
    static uint8_t bioz_phase = 0;
    if ((bioz_phase++ & 1) == 0) {
      max30001_bioz_sample_t bs;
      if (max30001.getBioZSample(&bs) == MAX30001_SUCCESS) g_bioz = bs.bioz_sample;
    }
    feed_respiration(g_bioz);

    max30001_ecg_sample_t es;
    if (max30001.getECGSample(&es) == MAX30001_SUCCESS) leadoff = es.lead_off_detected ? 1 : 0;
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

  // --- send ONE HealthyBridge VITALS frame per second ---
  static uint32_t last_tx = 0;
  if (millis() - last_tx >= 1000) {
    last_tx = millis();

    hb_vitals_payload vp;
    vp.hr          = hr;
    vp.spo2        = spo2;
    vp.rr          = rr;                                  // respiration from BioZ
    vp.temp_c_x100 = temp_present ? (int16_t)(temp_c * 100.0f) : 0;
    vp.flags       = (hr      ? HB_VITAL_HR_VALID    : 0) |
                     (spo2    ? HB_VITAL_SPO2_VALID  : 0) |
                     (rr      ? HB_VITAL_RR_VALID    : 0) |
                     (leadoff ? HB_VITAL_ECG_LEADOFF : 0);
    hb_send_vitals(vp);

    // Mirror to the USB console so you can watch the link without the ESP32.
    Serial.print("TX VITALS  HR=");    Serial.print(hr ? hr : 0);
    Serial.print(" SpO2=");            Serial.print(spo2 ? spo2 : 0);
    Serial.print(" RR=");              Serial.print(rr ? rr : 0);
    Serial.print(" Temp=");            Serial.print(temp_present ? temp_c : 0.0f, 2);
    Serial.print("  [frames=");        Serial.print(s_tx_frames);
    Serial.print(" drops=");           Serial.print(s_tx_drops);
    Serial.println("]");
    if (s_tx_drops && s_tx_frames == 0)
      Serial.println("  (all frames dropping — is the ESP32 attached / CTS asserted?)");
  }
}
