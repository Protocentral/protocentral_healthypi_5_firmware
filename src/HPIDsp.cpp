/*
 * DSP / vitals sink (A5) — see HPIDsp.h. Mirrors the NEXT dsp_task orchestration.
 */
#ifdef __FREERTOS   /* part of the FreeRTOS-only runtime; empty TU otherwise */

#include "HPIDsp.h"
#include <string.h>

DspSink::DspSink(hpi_vitals_t *store)
  : _v(store), _spo2_fill(0), _resp_count(0)
{
}

void DspSink::consume(const hpi_sample_t &s)
{
  /* ---- HR straight from the MAX30001 RTOR (non-zero only on a beat). ------- */
  if (s.hr > 0) {
    _v->hr       = s.hr;
    _v->hr_valid = (s.leadoff == 0);
  }

  /* ---- SpO2 from the PPG over a sliding 256-sample window (~1 Hz). --------- */
  _ir[_spo2_fill]  = (s.ppg_ir  < 0) ? 0u : (uint32_t)s.ppg_ir;
  _red[_spo2_fill] = (s.ppg_red < 0) ? 0u : (uint32_t)s.ppg_red;
  _spo2_fill++;
  if (_spo2_fill >= GF_SPO2_WIN) {
    struct gf_spo2_result r = gf_spo2_compute(_ir, _red, GF_SPO2_WIN);
    _v->spo2         = r.valid ? r.spo2 : 0;
    _v->spo2_valid   = r.valid;
    _v->spo2_pi_x100 = r.pi_x100;

    /* Slide: keep the most-recent (WIN-SLIDE) samples. Bounded. */
    memmove(_ir,  &_ir[GF_SPO2_SLIDE],
            (GF_SPO2_WIN - GF_SPO2_SLIDE) * sizeof(_ir[0]));
    memmove(_red, &_red[GF_SPO2_SLIDE],
            (GF_SPO2_WIN - GF_SPO2_SLIDE) * sizeof(_red[0]));
    _spo2_fill = GF_SPO2_WIN - GF_SPO2_SLIDE;
  }

  /* ---- Respiration from BioZ, batched by 4 (~32 Hz calibration rate). ------ */
  if (_resp_count < 4) {
    _resp_buf[_resp_count++] = (int16_t)s.bioz;
  }
  if (_resp_count >= 4) {
    int16_t filt[4];
    resp_process_sample(_resp_buf, filt);
    uint8_t rr = 0;
    resp_algo_process(filt, &rr);
    _v->resp_rate  = rr;
    _v->resp_valid = (rr > 0 && rr < 60);
    _resp_count = 0;
  }
}

#endif /* __FREERTOS */
