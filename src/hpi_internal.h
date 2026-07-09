/*
 * HealthyPi 5 NEXT (Arduino) — library-private internals.
 * ============================================================================
 * NOT part of the public API. A sketch includes <Protocentral_HealthyPi_5.h>, never this. It
 * declares the volatile inter-core state and the C-style ring/acquisition
 * functions that the runtime (Protocentral_HealthyPi_5.cpp), the ring (HPIRing.cpp) and the
 * core1 acquisition (HPIAcq.cpp) translation units share.
 *
 * Concurrency contract (the whole point of the dual-core design):
 *   - g_seq_core1 / g_ring_overflow / g_hb_core1 / latest vitals are WRITTEN
 *     only on core1 and READ on core0. They are plain 32-bit volatile words;
 *     32-bit aligned loads/stores are atomic on the RP2040 (Cortex-M0+), so no
 *     lock is needed for these single-word stats.
 *   - g_hb_broker / g_ring_level_max are owned by core0.
 *   - The ONLY structured data path between cores is the lock-free pico queue_t
 *     ring (HPIRing.cpp). core1 never calls a FreeRTOS API.
 */
#ifndef HPI_INTERNAL_H
#define HPI_INTERNAL_H

#include "hpi_types.h"

/* ---- inter-core / inter-task statistics (defined in Protocentral_HealthyPi_5.cpp) ------- */
extern volatile bool     g_ring_ready;       /* core0 set: ring is initialised  */
extern volatile bool     g_acq_started;      /* core1 set: sensors initialised   */

extern volatile uint32_t g_seq_core1;        /* last seq produced on core1       */
extern volatile uint32_t g_ring_overflow;    /* core1 push-failed (ring full)    */
extern volatile uint32_t g_ring_level_max;   /* peak ring occupancy seen         */

extern volatile uint16_t g_last_hr;          /* latest vitals, for telemetry     */
extern volatile uint8_t  g_last_spo2;

/* Heartbeats — each loop bumps its own counter; the watchdog checks they move. */
extern volatile uint32_t g_hb_core1;         /* core1 acquisition pace loop      */
extern volatile uint32_t g_hb_broker;        /* core0 ring drainer               */

/* core1 bring-up progress, so a fault dump shows exactly how far core1 got:
 *   0 = core1 never ran        1 = setup1 entered (past ring wait)
 *   2 = acq_init entered       3 = SPI0 up
 *   4 = AFE4400 init done      5 = MAX30001 begin done
 *   6 = ECG/BioZ streaming     7 = first loop1() iteration reached            */
extern volatile uint8_t  g_c1_stage;

/* Sub-step within acq_step(), so a fault dump pinpoints where core1 froze:
 *   0 = between cycles   1 = AFE4400 read    2 = BioZ read
 *   3 = R-to-R read      4 = ECG read        5 = ring push                    */
extern volatile uint8_t  g_acq_substep;

#ifdef __cplusplus
extern "C" {
#endif

/* ---- SPSC ring (HPIRing.cpp — the ONLY TU that includes pico queue.h) ----- */
void     hpi_ring_init(void);                /* core0, before core1 touches it   */
bool     hpi_ring_push(const hpi_sample_t *s); /* core1 only; false = full       */
bool     hpi_ring_pop(hpi_sample_t *s);      /* core0 only; false = empty         */
uint32_t hpi_ring_level(void);
uint32_t hpi_ring_capacity(void);

/* ---- core1 acquisition (HPIAcq.cpp — runs on core1 only) ----------------- */
void     hpi_acq_init(void);                 /* bring up SPI0 + both AFEs        */
void     hpi_acq_step(void);                 /* one paced 128 SPS cycle          */

#ifdef __cplusplus
}
#endif

#endif /* HPI_INTERNAL_H */
