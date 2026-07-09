/*
 * core1 acquisition — MAX30001 (ECG/BioZ/RTOR) + AFE4400 (PPG/SpO2) on SPI0.
 * ============================================================================
 * Runs on core1 ONLY (driven by the library's loop1()). Owns SPI0 exclusively.
 * Calls NO FreeRTOS API — its only outputs are hpi_ring_push() into the
 * lock-free ring and a handful of volatile counters in hpi_internal.h (single
 * 32-bit words, atomic on M0+). This isolation is what makes lossless 128 SPS
 * sampling a structural guarantee rather than a tuning job.
 *
 * The sensor objects and "latest value" state are file-static and touched ONLY
 * by core1, so they need no synchronisation. Sensor APIs are used exactly as in
 * the validated 08_OpenView_Stream example.
 */
#ifdef __FREERTOS   /* part of the FreeRTOS-only runtime; empty TU otherwise */

#include "hpi_internal.h"

#include <SPI.h>
#include <protocentral_max30001.h>
#include "protocentral_afe44xx.h"

static MAX30001     s_max30001(HPI_PIN_MAX30001_CS);
static AFE44XX      s_afe4400(HPI_PIN_AFE4400_CS, HPI_PIN_AFE4400_PWDN);
static afe44xx_data s_afe_data;

/* Latest held values (PPG and BioZ run at half the ECG rate; held in between). */
static int32_t  s_red = 0, s_ir = 0;
static int32_t  s_bioz = 0;
static uint16_t s_hr = 0, s_rtor = 0;
static uint8_t  s_spo2 = 0;
static uint8_t  s_leadoff = 0;

void hpi_acq_init(void)
{
  g_c1_stage = 2;                                /* acq_init entered          */

  /* SPI0 pins (core1 owns this bus exclusively). */
  pinMode(HPI_PIN_MAX30001_CS, OUTPUT);
  digitalWrite(HPI_PIN_MAX30001_CS, HIGH);
  SPI.setRX(HPI_PIN_SPI0_MISO);
  SPI.setTX(HPI_PIN_SPI0_MOSI);
  SPI.setSCK(HPI_PIN_SPI0_SCK);
  SPI.begin();
  g_c1_stage = 3;                                /* SPI0 up                   */

  s_afe4400.afe44xx_init();
  g_c1_stage = 4;                                /* AFE4400 init done         */

  s_max30001.begin();                            /* probe + reset             */
  g_c1_stage = 5;                                /* MAX30001 begin done       */

  s_max30001.startECGBioZ(MAX30001_RATE_128);    /* ECG + BioZ + R-to-R @128  */
  g_c1_stage = 6;                                /* streaming configured      */
}

/* NOTE on the v2.0.0 MAX30001 library: getECGSample()/getBioZSample() read ONE
 * FIFO word per call and always report SUCCESS — they do NOT signal FIFO-empty.
 * So we must not "drain until empty" (that would loop forever); instead core1
 * is paced at exactly the 128 SPS ODR and reads exactly one ECG sample per
 * tick (BioZ every 2nd tick = 64 SPS). The micros() pace keeps us locked to the
 * sensor's output rate. */
void hpi_acq_step(void)
{
  static uint32_t cycle = 0;

  /* PPG/SpO2 every HPI_PPG_DIVIDER-th cycle (~64 SPS). Same no-finger gate as
   * the OpenView example: report SpO2 = 0 when no finger rather than ~100 %. */
  if ((cycle % HPI_PPG_DIVIDER) == 0) {
    g_acq_substep = 1;
    s_afe4400.get_AFE44XX_Data(&s_afe_data);
    s_red = (int32_t)s_afe_data.RED_data;
    s_ir  = (int32_t)s_afe_data.IR_data;

    if (s_ir <= HPI_FINGER_IR_MIN) {
      s_spo2 = 0;                                  /* no finger -> invalid */
    } else if (s_afe_data.buffer_count_overflow) {
      s_spo2 = (s_afe_data.spo2 == -999) ? 0 : (uint8_t)s_afe_data.spo2;
      s_afe_data.buffer_count_overflow = false;
    }
  }

  /* BioZ at half the ECG rate; hold the last value between reads. */
  if ((cycle % 2) == 0) {
    g_acq_substep = 2;
    max30001_bioz_sample_t bs;
    if (s_max30001.getBioZSample(&bs) == MAX30001_SUCCESS) s_bioz = bs.bioz_sample;
  }
  cycle++;

  /* Heart rate / R-to-R interval. */
  g_acq_substep = 3;
  max30001_rtor_data_t rr;
  if (s_max30001.getRtoRData(&rr) == MAX30001_SUCCESS && rr.rr_detected) {
    /* The library reports 0xFFFF (and nonsense BPM) until a stable rhythm is
     * locked; surface that as 0 ("no HR") rather than 65535 -> 255 in OpenView. */
    uint16_t bpm = rr.heart_rate_bpm;
    s_hr   = (bpm == 0xFFFF || bpm > 300) ? 0 : bpm;
    s_rtor = rr.rr_interval_ms;
  }

  /* One ECG sample per paced cycle -> one ring sample. */
  g_acq_substep = 4;
  max30001_ecg_sample_t es;
  if (s_max30001.getECGSample(&es) == MAX30001_SUCCESS) {
    s_leadoff = es.lead_off_detected ? 1 : 0;

    hpi_sample_t s;
    s.seq     = ++g_seq_core1;
    s.ecg     = es.ecg_sample;
    s.bioz    = s_bioz;
    s.ppg_red = s_red;
    s.ppg_ir  = s_ir;
    s.hr      = s_hr;
    s.rtor    = s_rtor;
    s.spo2    = s_spo2;
    s.leadoff = s_leadoff;

    g_acq_substep = 5;
    if (!hpi_ring_push(&s)) {
      g_ring_overflow++;   /* core0 fell behind for >4 s — should never happen */
    }
  }
  g_acq_substep = 0;
}

#endif /* __FREERTOS */
