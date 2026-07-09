/*
 * OpenView 2 sink + packetiser.
 * ============================================================================
 * openview_build() is lifted field-for-field from the 08_OpenView_Stream
 * example's build_openview_packet(); keeping it identical is what lets OpenView
 * 2 and the NEXT host tools consume both firmwares interchangeably.
 */
#ifdef __FREERTOS   /* part of the FreeRTOS-only runtime; empty TU otherwise */

#include "HPIOpenView.h"
#include "cmd.h"

#define OV_START_1     0x0A
#define OV_START_2     0xFA
#define OV_STOP_1      0x00
#define OV_STOP_2      0x0B
#define OV_TYPE_DATA   0x02
#define OV_DATA_LEN    22

static inline void put_i32(uint8_t *p, int32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline void put_i16(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}

void openview_build(uint8_t *pkt, const hpi_sample_t *s,
                    uint16_t hr, uint8_t spo2, uint8_t rr, int16_t temp_x100)
{
  pkt[0] = OV_START_1;
  pkt[1] = OV_START_2;
  pkt[2] = OV_DATA_LEN;          /* len LSB */
  pkt[3] = 0x00;                 /* len MSB */
  pkt[4] = OV_TYPE_DATA;
  put_i32(&pkt[5],  s->ecg);     /* [0..3]   */
  put_i32(&pkt[9],  s->bioz);    /* [4..7]   */
  pkt[13] = 0x00;                /* [8]   BioZ skip flag (live) */
  put_i32(&pkt[14], s->ppg_red); /* [9..12]  */
  put_i32(&pkt[18], s->ppg_ir);  /* [13..16] */
  put_i16(&pkt[22], temp_x100);  /* [17..18] */
  pkt[24] = spo2;                /* [19] */
  pkt[25] = (uint8_t)hr;         /* [20] */
  pkt[26] = rr;                  /* [21] respiration rate */
  pkt[27] = OV_STOP_1;
  pkt[28] = OV_STOP_2;
}

OpenViewSink::OpenViewSink(Stream &port)
  : _port(port),
    _isUsb((Stream *)&Serial == &port)   /* default target is USB CDC -> DTR-gate */
{
}

void OpenViewSink::consume(const hpi_sample_t &s)
{
  /* Only emit while OpenView has the CDC port open (Serial is true when DTR is
   * asserted on arduino-pico). For a non-USB stream there is no DTR to check. */
  if (_isUsb && !(bool)Serial) return;

  /* Respect the host command plane's STREAM_START/STOP gate (default: on). */
  if (!cmd_streaming_enabled()) return;

  /* Vitals: the DSP sink's computed values when enabled, else the raw
   * per-sample HR/SpO2 from the AFEs (RR only exists with the DSP). */
  uint16_t hr;
  uint8_t  spo2, rr;
  if (HealthyPi5.vitalsEnabled()) {
    const hpi_vitals_t &v = HealthyPi5.vitals();
    hr = v.hr; spo2 = v.spo2; rr = v.resp_rate;
  } else {
    hr = s.hr; spo2 = s.spo2; rr = 0;
  }

  uint8_t pkt[OV_PACKET_LEN];
  openview_build(pkt, &s, hr, spo2, rr, HealthyPi5.temperature_x100());
  _port.write(pkt, OV_PACKET_LEN);
}

#endif /* __FREERTOS */
