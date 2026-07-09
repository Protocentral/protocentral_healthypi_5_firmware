/*
 * HealthyPi 5 NEXT (Arduino) — the cross-core sample type and sampling params.
 * ============================================================================
 * This is the ONE data structure that crosses the core1 -> core0 boundary. Its
 * field order mirrors the production NEXT firmware's hpi_sample_t so every
 * downstream sink (OpenView, future DSP/SD/bridge/display) maps 1:1.
 *
 * The volatile inter-core statistics/heartbeats that used to live here have
 * moved to hpi_internal.h (library-private) — a sketch never touches them.
 */
#ifndef HPI_TYPES_H
#define HPI_TYPES_H

#include <Arduino.h>
#include <board.h>

/* One acquired multi-channel sample. SpO2 is carried here from the AFE4400 so
 * the OpenView stream stays meaningful before the dedicated DSP/vitals rung
 * (A5) exists; it will move to the DSP sink at A5. */
typedef struct {
  uint32_t seq;       /* monotonic sample counter (gap-detect on the host)     */
  int32_t  ecg;       /* MAX30001 ECG                                          */
  int32_t  bioz;      /* MAX30001 BioZ (paired sample, held between reads)     */
  int32_t  ppg_red;   /* AFE4400 RED (latest, held between PPG cycles)         */
  int32_t  ppg_ir;    /* AFE4400 IR  (latest)                                  */
  uint16_t hr;        /* heart rate from MAX30001 RTOR (bpm)                   */
  uint16_t rtor;      /* last R-to-R interval (ms)                             */
  uint8_t  spo2;      /* SpO2 % (0 = invalid / no finger)                      */
  uint8_t  leadoff;   /* lead-off status bitfield                              */
} hpi_sample_t;

/* Depth of the lock-free core1 -> core0 ring. 512 samples at 128 SPS is ~4 s of
 * slack, so a transient core0 stall never costs a sample. */
#define HPI_RING_DEPTH        512

/* No-finger gate (carried from 08_OpenView_Stream — TUNE per board). Without a
 * finger we report SpO2 = 0 rather than a misleading ~100 %. */
#define HPI_FINGER_IR_MIN     50000

#endif /* HPI_TYPES_H */
