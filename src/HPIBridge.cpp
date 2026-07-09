/*
 * HealthyBridge Lite transport (A9) — Arduino port of the NEXT healthybridge.c.
 * ============================================================================
 * The frame format and CRC are byte-identical to the production firmware (and
 * the ESP32-C3 side), so the two MCUs cannot drift. Only the transport changes:
 * the pico-sdk UART + ISR ring is replaced by arduino-pico's Serial2 (UART1,
 * RTS/CTS), which buffers RX itself; TX is bounded-and-drop via availableForWrite
 * so an absent/slow ESP becomes drop-newest (counted) rather than a core0 stall.
 */
#ifdef __FREERTOS   /* part of the FreeRTOS-only runtime; empty TU otherwise */

#include "HPIBridge.h"
#include <Arduino.h>
#include <string.h>

static uint16_t s_seq;
static uint32_t s_tx_drops;
static uint32_t s_tx_frames;

/* CRC-16/CCITT, byte-identical to Zephyr's crc16_ccitt (poly 0x1021, reflected
 * form) so the ESP32-C3 computes the same value. Do not "optimise" — the wire
 * must match. */
static uint16_t hb_crc16_ccitt(uint16_t seed, const uint8_t *src, size_t len)
{
  for (; len > 0; len--) {
    uint8_t e = (uint8_t)(seed ^ *src++);
    uint8_t f = (uint8_t)(e ^ (e << 4));
    seed = (uint16_t)((seed >> 8) ^ ((uint16_t)f << 8) ^
                      ((uint16_t)f << 3) ^ ((uint16_t)f >> 4));
  }
  return seed;
}

uint32_t hb_tx_drops(void)         { return s_tx_drops; }
uint32_t hb_tx_frames(void)        { return s_tx_frames; }

/* App-overridable RX dispatch — weak no-ops by default. */
__attribute__((weak)) void hb_on_host_cmd(const uint8_t *, uint16_t) { }
__attribute__((weak)) void hb_on_status(const struct hb_status_payload *) { }

void hb_init(void)
{
  Serial2.setFIFOSize(1024);             /* RX buffer; TX writes the HW FIFO */
  Serial2.setTX(HPI_PIN_UART1_TX);
  Serial2.setRX(HPI_PIN_UART1_RX);
#ifndef HPI_BRIDGE_NO_FLOWCTRL
  Serial2.setRTS(HPI_PIN_UART1_RTS);     /* HW flow control (resolves §5.3) */
  Serial2.setCTS(HPI_PIN_UART1_CTS);
#endif                                   /* -DHPI_BRIDGE_NO_FLOWCTRL: bench-test
                                          * without an ESP32 (TX ignores CTS, so
                                          * frames go out GP24 to be sniffed).   */
  Serial2.begin(HB_UART_BAUD);
}

int hb_send_frame(uint8_t type, uint8_t flags, const uint8_t *payload, uint16_t len)
{
  if (len > HB_MAX_PAYLOAD)        return -1;
  if (len > 0 && payload == NULL)  return -1;

  uint8_t frame[HB_HEADER_SIZE + HB_MAX_PAYLOAD + HB_CRC_SIZE];

  struct hb_frame_header hdr;
  hdr.sync = HB_SYNC_WORD; hdr.type = type; hdr.flags = flags;
  hdr.length = len;        hdr.seq = s_seq++;
  memcpy(frame, &hdr, HB_HEADER_SIZE);
  if (len > 0) memcpy(frame + HB_HEADER_SIZE, payload, len);

  /* CRC over TYPE..PAYLOAD (everything after the 2-byte sync). */
  uint16_t crc = hb_crc16_ccitt(0xFFFF, &frame[2], (HB_HEADER_SIZE - 2) + len);
  uint16_t total = HB_HEADER_SIZE + len;
  frame[total++] = (uint8_t)crc;
  frame[total++] = (uint8_t)(crc >> 8);

  /* Bounded, yielding write — the faithful port of the NEXT hb_uart_write_bounded.
   * arduino-pico's availableForWrite() is a WRITABLE FLAG (1 = the HW FIFO has
   * room, 0 = full), exactly like the SDK's uart_is_writable() — NOT a byte
   * count. So write byte-by-byte: wait (yielding) up to 2 ms per byte for FIFO
   * room, and if the link stalls that long (ESP absent / CTS deasserted), drop
   * the rest of the frame (counted) rather than blocking core0. 2 ms is ~180x
   * the ~11 us it takes one byte to clear at 921600, so a healthy link never
   * false-drops. */
  for (uint16_t i = 0; i < total; i++) {
    uint32_t t0 = micros();
    while (Serial2.availableForWrite() < 1) {
      if ((uint32_t)(micros() - t0) > 2000u) { s_tx_drops++; return 0; }
      yield();
    }
    Serial2.write(frame[i]);
  }
  s_tx_frames++;
  return 0;
}

void hb_rx_poll(void)
{
  static enum { S0, S1, HDR, PL, CRC } st = S0;
  static uint8_t  hdr6[6], payload[HB_MAX_PAYLOAD], crc_rx[2];
  static uint16_t len, idx, hidx, cidx;

  int c;
  while ((c = Serial2.read()) >= 0) {
    uint8_t b = (uint8_t)c;
    switch (st) {
    case S0:
      if (b == 0x55) st = S1;
      break;
    case S1:
      if (b == 0xAA)      { hidx = 0; st = HDR; }
      else if (b == 0x55) { st = S1; }
      else                { st = S0; }
      break;
    case HDR:
      hdr6[hidx++] = b;
      if (hidx >= 6) {
        len = (uint16_t)hdr6[2] | ((uint16_t)hdr6[3] << 8);
        if (len > HB_MAX_PAYLOAD) { st = S0; break; }
        idx = 0; cidx = 0;
        st = (len == 0) ? CRC : PL;
      }
      break;
    case PL:
      payload[idx++] = b;
      if (idx >= len) { st = CRC; cidx = 0; }
      break;
    case CRC:
      crc_rx[cidx++] = b;
      if (cidx >= 2) {
        uint16_t rx   = (uint16_t)crc_rx[0] | ((uint16_t)crc_rx[1] << 8);
        uint16_t calc = hb_crc16_ccitt(0xFFFF, hdr6, 6);
        calc = hb_crc16_ccitt(calc, payload, len);
        if (rx == calc) {
          if (hdr6[0] == HB_TYPE_HOST_CMD) {
            hb_on_host_cmd(payload, len);
          } else if (hdr6[0] == HB_TYPE_STATUS &&
                     len >= sizeof(struct hb_status_payload)) {
            hb_on_status((const struct hb_status_payload *)payload);
          }
        }
        st = S0;
      }
      break;
    }
  }
}

/* ===== BridgeSink ========================================================= */
void BridgeSink::begin()
{
  hb_init();
}

void BridgeSink::consume(const hpi_sample_t &s)
{
  hb_rx_poll();                          /* service host commands / status */

  /* Batch biosignal samples into one BIOSIG frame. */
  _batch[_n++] = s;
  if (_n >= BATCH) {
    uint8_t buf[sizeof(struct hb_biosig_payload) +
                BATCH * sizeof(struct hb_biosig_sample)];
    struct hb_biosig_payload *p = (struct hb_biosig_payload *)buf;
    p->timestamp_ms   = millis();
    p->sample_count   = _n;
    p->sample_rate_hz = HPI_ECG_SPS;
    for (uint16_t i = 0; i < _n; i++) {
      p->samples[i].ecg     = _batch[i].ecg;
      p->samples[i].bioz    = _batch[i].bioz;
      p->samples[i].ppg_red = _batch[i].ppg_red;
      p->samples[i].ppg_ir  = _batch[i].ppg_ir;
    }
    uint16_t plen = sizeof(struct hb_biosig_payload) +
                    _n * sizeof(struct hb_biosig_sample);
    hb_send_frame(HB_TYPE_BIOSIG, HB_FLAG_LAST, buf, plen);
    _n = 0;
  }

  /* Emit vitals (+ battery) ~1 Hz. */
  if (++_since_vitals >= HPI_ECG_SPS) {
    _since_vitals = 0;

    const hpi_vitals_t &v = HealthyPi5.vitals();
    struct hb_vitals_payload vp;
    vp.hr          = v.hr;
    vp.spo2        = v.spo2;
    vp.rr          = v.resp_rate;
    vp.temp_c_x100 = HealthyPi5.temperature_x100();   /* live store, like OpenView */
    vp.flags = (uint8_t)((v.hr_valid   ? HB_VITAL_HR_VALID   : 0) |
                         (v.spo2_valid ? HB_VITAL_SPO2_VALID : 0) |
                         (v.resp_valid ? HB_VITAL_RR_VALID   : 0) |
                         (s.leadoff    ? HB_VITAL_ECG_LEADOFF: 0));
    hb_send_frame(HB_TYPE_VITALS, HB_FLAG_LAST, (const uint8_t *)&vp, sizeof(vp));

    if (HealthyPi5.batteryPresent()) {
      struct hb_battery_payload bp;
      bp.soc        = HealthyPi5.batterySoc();
      bp.flags      = 0;
      bp.millivolts = HealthyPi5.batteryMillivolts();
      hb_send_frame(HB_TYPE_BATTERY, HB_FLAG_LAST, (const uint8_t *)&bp, sizeof(bp));
    }
  }
}

#endif /* __FREERTOS */
