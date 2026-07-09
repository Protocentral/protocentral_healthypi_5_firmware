/*
 * HealthyPi 5 — OpenView 2 streaming firmware (Arduino)
 * ============================================================================
 * Single-file, single-core teaching firmware that streams ECG + BioZ + PPG +
 * SpO2 + HR + respiration rate + temperature to ProtoCentral OpenView 2 (the
 * HealthyPi / HealthyPi 6 device profile). This is the supported Arduino
 * workflow for HealthyPi 5 v5.7 — read it top to bottom; there is no hidden
 * magic. Respiration is derived from the BioZ waveform by resp_process.c (the
 * same detector the dual-core HealthyPi5_NEXT uses); see feed_respiration() below.
 *
 *   Board    : "Raspberry Pi Pico"  (Earle Philhower arduino-pico core)
 *   Sensors  : MAX30001 (ECG/BioZ/HR), AFE4400 (PPG/SpO2), MAX30205/AS6221 (temp, opt.)
 *   Output   : USB-CDC -> OpenView 2
 *   Libraries: protocentral_max30001, protocentral_afe44xx (its afe44xx driver
 *              computes SpO2 internally), HealthyPi5 (this repo, for board.h)
 *
 * WHY OPENVIEW 2 SHOWED "PACKETS RECEIVED" BUT NO WAVEFORM BEFORE:
 *   OpenView 2 expects the CURRENT OpenView DATA packet — a 22-byte payload in a
 *   fixed field order (ECG, BioZ, skip, RED, IR, temp, SpO2, HR, RR). The old
 *   sketch sent a 20-byte payload with RED/IR swapped and no temp/HR/RR, so
 *   OpenView could frame it but not map the channels. This sketch sends the
 *   exact 29-byte frame OpenView 2 expects (see build_openview_packet()).
 *
 * STATUS: starting point for v5.7 — pins/format match the production firmware.
 *         Validate on a board, then tune FINGER_PRESENT_IR_MIN.
 */

#include <SPI.h>
#include <Wire.h>
#include <protocentral_max30001.h>
#include "protocentral_afe44xx.h"
#include <board.h>
#include <resp_process.h>   // BioZ respiration detector (HealthyPi5 lib)

// ---------------------------------------------------------------------------
// OpenView 2 DATA packet (byte-identical to the production NEXT/Zephyr firmware)
//   frame:   0A FA | len_lo len_hi | 0x02 | payload[22] | 00 0B   (29 bytes)
//   payload: ECG i32 | BioZ i32 | bioz_skip u8 | RED i32 | IR i32
//            | temp_x100 i16 | SpO2 u8 | HR u8 | RR u8
// ---------------------------------------------------------------------------
#define OV_START_1     0x0A
#define OV_START_2     0xFA
#define OV_STOP_1      0x00
#define OV_STOP_2      0x0B
#define OV_TYPE_DATA   0x02
#define OV_DATA_LEN    22
#define OV_PACKET_LEN  (5 + OV_DATA_LEN + 2)   // 29

// ---- sensor objects --------------------------------------------------------
MAX30001     max30001(HPI_PIN_MAX30001_CS);
AFE44XX      afe4400(HPI_PIN_AFE4400_CS, HPI_PIN_AFE4400_PWDN);
afe44xx_data afe_data;

// ---- latest values shared into each packet ---------------------------------
int32_t  g_ppg_red = 0, g_ppg_ir = 0;
int32_t  g_bioz = 0;                 // held between half-rate BioZ reads
int16_t  g_temp_x100 = 0;
uint8_t  g_spo2 = 0, g_hr = 0, g_rr = 0;
bool     g_temp_present = false;

// No-finger gate: a finger raises the IR DC level well above the empty baseline.
// TUNE on your board (watch IR with/without a finger), and confirm the sign of
// IR_data first. Without a finger we send SpO2 = 0 (invalid) rather than a
// misleading ~100%.
const int32_t FINGER_PRESENT_IR_MIN = 50000;

uint32_t g_last_ppg_ms  = 0;
uint32_t g_last_temp_ms = 0;

// Little-endian helpers keep the packet builder readable.
static inline void put_i32(uint8_t *p, int32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline void put_i16(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}

// ---- Respiration rate from the BioZ waveform -------------------------------
// resp_process.c (from the HealthyPi5 library) is an adaptive threshold-crossing
// detector. It must be fed at ~32 Hz — exactly 128 SPS / 4 — which is what its
// RESP_CALIBRATION_FACTOR assumes. So we push the (held) BioZ value once per
// 128 SPS cycle below, collect 4, then run one detector step. g_rr stays 0 until
// the detector has seen ~10 s of signal AND at least 3 breaths.
static int16_t resp_batch[4];
static uint8_t resp_fill = 0;
void feed_respiration(int32_t bioz_held) {
  resp_batch[resp_fill++] = (int16_t)bioz_held;
  if (resp_fill >= 4) {
    int16_t filtered[4];
    resp_process_sample(resp_batch, filtered);   // pass-through (optional LPF)
    resp_algo_process(filtered, &g_rr);          // averages 4 -> 32 Hz detector
    resp_fill = 0;
  }
}

// Build one OpenView DATA packet for a single ECG/BioZ sample plus latest vitals.
void build_openview_packet(uint8_t *pkt, int32_t ecg, int32_t bioz)
{
  pkt[0] = OV_START_1;
  pkt[1] = OV_START_2;
  pkt[2] = OV_DATA_LEN;     // len LSB
  pkt[3] = 0x00;            // len MSB
  pkt[4] = OV_TYPE_DATA;
  put_i32(&pkt[5],  ecg);            // [0..3]
  put_i32(&pkt[9],  bioz);           // [4..7]
  pkt[13] = 0x00;                    // [8]   BioZ skip flag (live)
  put_i32(&pkt[14], g_ppg_red);      // [9..12]
  put_i32(&pkt[18], g_ppg_ir);       // [13..16]
  put_i16(&pkt[22], g_temp_x100);    // [17..18]
  pkt[24] = g_spo2;                  // [19]
  pkt[25] = g_hr;                    // [20]
  pkt[26] = g_rr;                    // [21]
  pkt[27] = OV_STOP_1;
  pkt[28] = OV_STOP_2;
}

// ---- optional QWIIC temperature: MAX30205 (0x49) or AS6221 (0x48) ----------
// Both put the temperature in reg 0x00 as signed 16-bit big-endian; only the
// address and per-LSB resolution differ. Detect once, then scale accordingly.
uint8_t temp_addr = 0;               // detected sensor address (0 = none found)
float   temp_lsb  = 0.0f;            // C per LSB: MAX30205 1/256, AS6221 1/128

bool detect_temp_sensor()            // first sensor to ACK wins
{
  Wire1.beginTransmission(HPI_ADDR_MAX30205);
  if (Wire1.endTransmission() == 0) { temp_addr = HPI_ADDR_MAX30205; temp_lsb = 1.0f / 256; return true; }
  Wire1.beginTransmission(HPI_ADDR_AS6221);
  if (Wire1.endTransmission() == 0) { temp_addr = HPI_ADDR_AS6221;   temp_lsb = 1.0f / 128; return true; }
  return false;
}
int16_t read_temp_x100()
{
  Wire1.beginTransmission(temp_addr);
  Wire1.write(HPI_TEMP_REG);         // 0x00 on both sensors
  if (Wire1.endTransmission(false) != 0) return g_temp_x100;
  Wire1.requestFrom(temp_addr, 2);
  if (Wire1.available() < 2) return g_temp_x100;
  int16_t raw = (Wire1.read() << 8) | Wire1.read();
  return (int16_t)(raw * temp_lsb * 100.0f);   // scale by the detected sensor's LSB
}

void setup()
{
  Serial.begin(115200);              // USB-CDC to OpenView 2

  // --- SPI0: MAX30001 + AFE4400 ---
  pinMode(HPI_PIN_MAX30001_CS, OUTPUT);
  digitalWrite(HPI_PIN_MAX30001_CS, HIGH);
  SPI.setRX(HPI_PIN_SPI0_MISO);
  SPI.setTX(HPI_PIN_SPI0_MOSI);
  SPI.setSCK(HPI_PIN_SPI0_SCK);
  SPI.begin();

  // --- I2C1 / QWIIC: temperature (MAX30205 or AS6221, optional) ---
  Wire1.setSDA(HPI_PIN_I2C1_SDA);
  Wire1.setSCL(HPI_PIN_I2C1_SCL);
  Wire1.setClock(HPI_I2C1_HZ);
  Wire1.begin();
  g_temp_present = detect_temp_sensor();   // probe once; don't poll if absent

  afe4400.afe44xx_init();
  max30001.begin();                              // probe + reset
  max30001.startECGBioZ(MAX30001_RATE_128);      // ECG + BioZ + R-to-R @128 SPS
}

void loop()
{
  uint32_t now = millis();
  uint8_t  pkt[OV_PACKET_LEN];

  // 1) PPG / SpO2 — paced so the AFE read never stalls ECG draining.
  if (now - g_last_ppg_ms >= 8) {        // ~125 Hz
    g_last_ppg_ms = now;
    afe4400.get_AFE44XX_Data(&afe_data);
    g_ppg_red = (int32_t)afe_data.RED_data;
    g_ppg_ir  = (int32_t)afe_data.IR_data;

    bool finger = (g_ppg_ir > FINGER_PRESENT_IR_MIN);
    if (!finger) {
      g_spo2 = 0;                        // no finger -> invalid (NOT ~100%)
    } else if (afe_data.buffer_count_overflow) {
      g_spo2 = (afe_data.spo2 == -999) ? 0 : (uint8_t)afe_data.spo2;
      afe_data.buffer_count_overflow = false;
    }
  }

  // 2) Temperature — slow, and only if the sensor is present.
  if (g_temp_present && (now - g_last_temp_ms >= 1000)) {
    g_last_temp_ms = now;
    g_temp_x100 = read_temp_x100();
  }

  // 3) Heart rate / R-to-R from the MAX30001. The library reports 0xFFFF (and
  //    nonsense BPM) until a stable rhythm locks; surface that as 0 ("no HR")
  //    rather than letting the uint8_t cast wrap 65535 -> 255 in the packet.
  max30001_rtor_data_t rr;
  if (max30001.getRtoRData(&rr) == MAX30001_SUCCESS && rr.rr_detected) {
    uint16_t bpm = rr.heart_rate_bpm;
    g_hr = (bpm == 0xFFFF || bpm > 300) ? 0 : (uint8_t)bpm;
  }

  // 4) ECG/BioZ — paced at 128 SPS (the MAX30001 ODR). The v2.0.0 library
  //    returns one FIFO word per call and never signals "empty", so we PACE
  //    one read per sample period rather than draining a FIFO. One OpenView
  //    packet per ECG sample, only while OpenView has the port open (Serial is
  //    true when DTR is asserted on arduino-pico).
  static uint32_t next_ecg_us = micros();
  if ((int32_t)(micros() - next_ecg_us) >= 0) {
    next_ecg_us += HPI_ACQ_PERIOD_US;

    // BioZ runs at half the ECG rate; read it every 2nd cycle, hold in between.
    static uint8_t bioz_phase = 0;
    if ((bioz_phase++ & 1) == 0) {
      max30001_bioz_sample_t bs;
      if (max30001.getBioZSample(&bs) == MAX30001_SUCCESS) g_bioz = bs.bioz_sample;
    }

    // Feed the held BioZ to the respiration detector at the full 128 SPS pace
    // (it batches by 4 internally -> the 32 Hz the algorithm is calibrated for).
    feed_respiration(g_bioz);

    max30001_ecg_sample_t es;
    if (max30001.getECGSample(&es) == MAX30001_SUCCESS && Serial) {
      build_openview_packet(pkt, es.ecg_sample, g_bioz);
      Serial.write(pkt, OV_PACKET_LEN);
    }
  }
}
