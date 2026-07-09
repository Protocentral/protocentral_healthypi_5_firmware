/*
 * HealthyPi 5 NEXT (Arduino) — SD recording sink (A8).
 * ============================================================================
 * Records the four waveform channels to /REC*.BIN on a FAT SD card (SPI1).
 * Acquisition never touches the card: this is an ordinary HPISink, so the broker
 * feeds it through its own drop-newest queue on its own core0 task — a slow card
 * only drops SD records (counted), never back-pressures core1.
 *
 * On-disk format is byte-identical to the NEXT firmware / tools/host/decode_rec.py:
 *   32-byte header ($HP5 magic) + 16-byte int32-LE records (ecg, bioz, red, ir).
 *
 * REC_START/REC_STOP arrive via the A6 command plane's weak hpi_rec_start/stop
 * hooks, which this module overrides.
 */
#ifndef HPI_SD_H
#define HPI_SD_H

#include "Protocentral_HealthyPi_5.h"

/* One on-disk record: 16 bytes, int32 LE. */
typedef struct {
  int32_t ecg;
  int32_t bioz;
  int32_t ppg_red;
  int32_t ppg_ir;
} hpi_sd_rec_t;

class SdSink : public HPISink {
public:
  void        begin() override;                 /* mount the card on the task   */
  void        consume(const hpi_sample_t &s) override;
  uint16_t    stackWords() const override { return 2048; }  /* FatFS needs room */
  const char *name() const override { return "sd"; }
};

/* Published state, for telemetry / the sketch. */
bool     hpi_sd_recording(void);
bool     hpi_sd_card_present(void);
uint32_t hpi_sd_drops(void);
uint32_t hpi_sd_record_secs(void);

#endif /* HPI_SD_H */
