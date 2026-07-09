/*
 * HealthyPi 5 NEXT (Arduino) — DSP / vitals sink (A5).
 * ============================================================================
 * Consumes the sample stream (one frame per ECG sample, 128 SPS) and computes
 * vitals into a shared hpi_vitals_t store — a faithful port of the production
 * NEXT dsp_task (healthypi5_next_rp2040/src/tasks/dsp_task.c):
 *   - HR   : taken straight from the MAX30001 RTOR field (non-zero on a beat)
 *   - SpO2 : clean-room gf_spo2 over sliding 256-sample IR/RED windows (~1 Hz)
 *   - RR   : resp_process over BioZ, fed at 128/4 = ~32 Hz (its calibration rate)
 *
 * consume() is bounded: fixed-size windows, no data-dependent indexing, no heap.
 * The 2 KB of SpO2 window buffers live in the sink instance (static storage),
 * never on a task stack.
 */
#ifndef HPI_DSP_H
#define HPI_DSP_H

#include "Protocentral_HealthyPi_5.h"

extern "C" {
#include "gf_spo2.h"
#include "resp_process.h"
}

class DspSink : public HPISink {
public:
  explicit DspSink(hpi_vitals_t *store);
  void        consume(const hpi_sample_t &s) override;
  const char *name() const override { return "dsp"; }

private:
  hpi_vitals_t *_v;

  /* SpO2 sliding windows (raw AFE ADC units). */
  uint32_t _ir[GF_SPO2_WIN];
  uint32_t _red[GF_SPO2_WIN];
  size_t   _spo2_fill;

  /* Respiration batching: feed resp_process every 4th sample (~32 Hz). */
  int16_t  _resp_buf[4];
  int      _resp_count;
};

#endif /* HPI_DSP_H */
