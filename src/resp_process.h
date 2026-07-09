/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2025 ProtoCentral Electronics
 *
 * Respiration-rate detection from the BioZ waveform. Ported verbatim from
 * app/src/resp_process.c (adaptive-threshold-crossing algorithm); the only
 * change is dropping the Zephyr logging shim. The calibration
 * (RESP_CALIBRATION_FACTOR) assumes resp_rate_detect is fed at ~32 Hz — i.e. one
 * call per 4 acquisition samples (128 SPS / 4), which dsp_task preserves.
 */
#ifndef HPI_RESP_PROCESS_H
#define HPI_RESP_PROCESS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void resp_process_sample(int16_t *CurrAqsSample, int16_t *respFiltered);
void resp_algo_process(int16_t *CurrSample, volatile uint8_t *RespirationRate);
void resp_rate_detect(int16_t Resp_wave, volatile uint8_t *RespirationRate);

#ifdef __cplusplus
}
#endif

#endif /* HPI_RESP_PROCESS_H */
