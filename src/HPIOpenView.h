/*
 * HealthyPi 5 NEXT (Arduino) — OpenView 2 sink + packetiser.
 * ============================================================================
 * The built-in sink that streams the byte-exact 29-byte OpenView DATA frame
 * OpenView 2 expects (HealthyPi / HealthyPi 6 profile). The frame layout is
 * identical to the 08_OpenView_Stream example and the production NEXT/Zephyr
 * firmware — do NOT change the field order, or OpenView 2 and the NEXT host
 * tools stop decoding it.
 *
 *   frame:   0A FA | len_lo len_hi | 0x02 | payload[22] | 00 0B   (29 bytes)
 *   payload: ECG i32 | BioZ i32 | bioz_skip u8 | RED i32 | IR i32
 *            | temp_x100 i16 | SpO2 u8 | HR u8 | RR u8
 */
#ifndef HPI_OPENVIEW_H
#define HPI_OPENVIEW_H

#include "Protocentral_HealthyPi_5.h"

#define OV_PACKET_LEN  29

/* Serialise one sample into a 29-byte OpenView DATA frame (pkt must be >=29).
 * Waveforms (ECG/BioZ/PPG) come from the sample; hr/spo2/rr are passed in
 * because they are slowly-updated vitals (from the DSP sink or the raw AFEs),
 * not per-sample fields. temp_x100 is an I2C value; passed 0 until A9. */
void openview_build(uint8_t *pkt, const hpi_sample_t *s,
                    uint16_t hr, uint8_t spo2, uint8_t rr, int16_t temp_x100);

/* A sink that writes the OpenView frame to a byte stream. The default target is
 * the USB-CDC Serial port; when that port is the target, output is DTR-gated
 * (nothing is sent unless OpenView 2 has the port open), matching the teaching
 * firmware. consume() is bounded — one fixed-size write per sample. */
class OpenViewSink : public HPISink {
public:
  explicit OpenViewSink(Stream &port);
  void        consume(const hpi_sample_t &s) override;
  const char *name() const override { return "usb"; }

private:
  Stream &_port;
  bool    _isUsb;   /* target is the global USB Serial -> apply the DTR gate */
};

#endif /* HPI_OPENVIEW_H */
