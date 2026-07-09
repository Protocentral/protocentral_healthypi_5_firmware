/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ProtoCentral Electronics
 *
 * HealthyBridge Lite — RP2040 <-> ESP32-C3 inter-processor protocol.
 *
 * A trimmed single-UART variant of HealthyPi 6's HealthyBridge IPC. HP6 splits
 * transports (SPI @ 20 MHz for bulk data, UART @ 921600 JSON for control);
 * HP5 has only UART1 (RTS/CTS) between the two MCUs, so HealthyBridge Lite runs
 * ONE binary frame format over that UART for BOTH data and control — no JSON,
 * no SPI. The RP2040 (acquisition/DSP/USB core) streams biosignals + vitals to
 * the ESP32-C3, which owns the entire BLE stack (and, later, Wi-Fi — the C3 has
 * both radios). Control/status flow the other direction over the same link.
 *
 * Frame (little-endian throughout), identical in spirit to HP6 HealthyBridge:
 *
 *   +--------+------+-------+--------+------+-----------+--------+
 *   | SYNC   | TYPE | FLAGS | LENGTH | SEQ  | PAYLOAD   | CRC16  |
 *   | 0xAA55 | 1B   | 1B    | 2B     | 2B   | LENGTH B  | 2B     |
 *   +--------+------+-------+--------+------+-----------+--------+
 *      2B                                                  CRC-CCITT over
 *                                                          TYPE..PAYLOAD
 *
 * Header is 8 bytes; CRC16 (CCITT, poly 0x1021, init 0xFFFF) covers TYPE
 * through the end of PAYLOAD. This header is shared verbatim by the ESP32-C3
 * firmware so the two sides cannot drift.
 */

#ifndef HPI_BRIDGE_H
#define HPI_BRIDGE_H

#include <stdint.h>
#include "Protocentral_HealthyPi_5.h"

#define HB_SYNC_WORD      0xAA55
#define HB_HEADER_SIZE    8        /* sync(2)+type(1)+flags(1)+len(2)+seq(2) */
#define HB_CRC_SIZE       2
#define HB_MAX_PAYLOAD    512
#define HB_UART_BAUD      921600   /* UART1 + RTS/CTS; plenty for HP5 rates */

/* ---- Message types (TYPE byte) ---- */
/* Data, RP2040 -> ESP32 */
#define HB_TYPE_BIOSIG    0x20     /* batched ECG/BioZ/PPG samples            */
#define HB_TYPE_VITALS    0x40     /* computed HR/SpO2/RR/temp + lead-off     */
#define HB_TYPE_BATTERY   0x41     /* fuel-gauge SoC / voltage (MAX17048)     */
/* Control, RP2040 -> ESP32 */
#define HB_TYPE_CTRL_CMD  0x50     /* command (payload[0] = HB_CMD_*)         */
/* Response/status, ESP32 -> RP2040 */
#define HB_TYPE_CTRL_RESP 0x51     /* command response                        */
#define HB_TYPE_HOST_CMD  0x52     /* ESP -> RP2040: raw phone command bytes  */
#define HB_TYPE_HOST_RESP 0x53     /* RP2040 -> ESP: phone command response   */
#define HB_TYPE_STATUS    0x61     /* link/BLE/Wi-Fi status (struct hb_status)*/

/* ---- FLAGS bitfield ---- */
#define HB_FLAG_LAST      (1 << 2)
#define HB_FLAG_ACK_REQ   (1 << 3)
#define HB_FLAG_ERROR     (1 << 7)

/* ---- Control commands (CTRL_CMD payload[0]) ---- */
#define HB_CMD_PING          0x01
#define HB_CMD_BLE_ADV_START 0x10
#define HB_CMD_BLE_ADV_STOP  0x11
#define HB_CMD_BLE_SET_NAME  0x12  /* payload[1..] = name string             */
#define HB_CMD_GET_STATUS    0x30
/* Reserved for the Wi-Fi follow-on (C3 has Wi-Fi too; IPC stays unchanged): */
#define HB_CMD_WIFI_ENABLE   0x20
#define HB_CMD_WIFI_DISABLE  0x21
#define HB_CMD_WIFI_SOFTAP   0x22  /* start captive-portal provisioning      */

/* ---- Frame header (packed, little-endian) ---- */
struct hb_frame_header {
    uint16_t sync;      /* HB_SYNC_WORD */
    uint8_t  type;      /* HB_TYPE_*    */
    uint8_t  flags;     /* HB_FLAG_*    */
    uint16_t length;    /* payload bytes */
    uint16_t seq;       /* incrementing per frame */
} __attribute__((packed));

/* ---- Payloads ---- */

/* One acquisition sample, all HP5 channels (matches the core's gf_sample_t). */
struct hb_biosig_sample {
    int32_t ecg;
    int32_t bioz;
    int32_t ppg_red;
    int32_t ppg_ir;
} __attribute__((packed));   /* 16 bytes */

/* HB_TYPE_BIOSIG payload: header + a batch of samples. */
struct hb_biosig_payload {
    uint32_t timestamp_ms;       /* RP2040 uptime of first sample */
    uint16_t sample_count;
    uint16_t sample_rate_hz;     /* nominal (128) */
    struct hb_biosig_sample samples[];   /* sample_count entries */
} __attribute__((packed));

/* HB_TYPE_VITALS payload. */
#define HB_VITAL_HR_VALID    (1 << 0)
#define HB_VITAL_SPO2_VALID  (1 << 1)
#define HB_VITAL_RR_VALID    (1 << 2)
#define HB_VITAL_ECG_LEADOFF (1 << 3)
#define HB_VITAL_PPG_LEADOFF (1 << 4)
struct hb_vitals_payload {
    uint16_t hr;
    uint8_t  spo2;
    uint8_t  rr;
    int16_t  temp_c_x100;
    uint8_t  flags;          /* HB_VITAL_* */
} __attribute__((packed));

/* HB_TYPE_STATUS payload (ESP32 -> RP2040). Wi-Fi fields are valid only once
 * the Wi-Fi follow-on ships; until then they read 0. */
struct hb_status_payload {
    uint8_t ble_advertising;
    uint8_t ble_connected;
    uint8_t wifi_connected;
    uint8_t wifi_ap_mode;
} __attribute__((packed));

/* HB_TYPE_BATTERY payload (RP2040 -> ESP32). Not yet sent by app_gf — the
 * MAX17048 fuel-gauge read is a deferred GF gap; the ESP side already parses
 * and displays this once the RP2040 starts emitting it. */
#define HB_BATT_CHARGING (1 << 0)
struct hb_battery_payload {
    uint8_t  soc;          /* state of charge, 0..100 %        */
    uint8_t  flags;        /* bit0 = charging                  */
    uint16_t millivolts;   /* cell voltage, mV                 */
} __attribute__((packed));

/* ---- RP2040-side transport API (UART1, HW flow control) ---- */

/* Bring up the HealthyBridge UART link (UART1 @ 921600, RTS/CTS). */
void hb_init(void);

/* Frame and transmit one HealthyBridge frame. Builds
 * SYNC|type|flags|len|seq|payload|crc16 and writes it atomically to the link.
 * Returns 0 on success, negative errno on bad args. Both MCUs are
 * little-endian so the packed header is wire-compatible. */
int hb_send_frame(uint8_t type, uint8_t flags, const uint8_t *payload, uint16_t len);

/* Count of frames dropped because the ESP stopped draining the TX FIFO (link
 * back-pressure / absent co-processor). Monotonic; exposed for the instr line. */
uint32_t hb_tx_drops(void);

/* Count of frames successfully written to the link. If this climbs and
 * hb_tx_drops stays 0, the ESP is draining the UART (link up). */
uint32_t hb_tx_frames(void);

/* Drain the UART RX ring and parse any complete frames, dispatching to
 * hb_on_host_cmd / hb_on_status. Call periodically from the bridge task (the
 * RP2040 port replaces Zephyr's dedicated RX thread with this cooperative poll;
 * the UART ISR keeps filling the ring meanwhile, so no bytes are lost). */
void hb_rx_poll(void);

/* RX dispatch (weak in HPIBridge.cpp; override in a sketch to handle phone
 * commands forwarded by the ESP, or BLE/Wi-Fi status):
 *  - hb_on_host_cmd: raw phone command bytes forwarded by the ESP.
 *  - hb_on_status:   BLE/Wi-Fi link status from the ESP. */
struct hb_status_payload;
void hb_on_host_cmd(const uint8_t *payload, uint16_t len);
void hb_on_status(const struct hb_status_payload *st);

/*
 * HealthyBridge sink: batches biosignal samples into HB_TYPE_BIOSIG frames,
 * emits a HB_TYPE_VITALS frame ~1 Hz, and polls the RX link each call. Like
 * every sink it is drop-newest: an absent/slow ESP only drops bridge frames
 * (hb_tx_drops), never back-pressuring acquisition.
 */
class BridgeSink : public HPISink {
public:
  void        begin() override;                 /* hb_init() on the task        */
  void        consume(const hpi_sample_t &s) override;
  const char *name() const override { return "brdg"; }

private:
  static const uint16_t BATCH = 8;              /* samples per BIOSIG frame      */
  hpi_sample_t _batch[BATCH];
  uint16_t     _n = 0;
  uint32_t     _since_vitals = 0;
};

#endif /* HPI_BRIDGE_H */
