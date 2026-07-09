/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ProtoCentral Electronics
 *
 * Clean-room SpO2 estimator for the greenfield core, replacing the legacy
 * spo2_process.c (a complex, magic-number-tuned algorithm with peak-location
 * arrays, sorts and per-pulse tables that intermittently corrupted memory under
 * low-perfusion / breath-hold edges, tripping the scheduler-wedge WDT).
 *
 * This implementation is fully BOUNDED by construction: it only computes
 * min / max / mean over the input windows — no peak arrays, no sort, no
 * data-dependent indexing — so it cannot scribble out of bounds.
 *
 * Ratio-of-ratios method:
 *   DC = mean(window), AC = max(window) - min(window)   (per channel)
 *   R  = (AC_red / DC_red) / (AC_ir / DC_ir)
 *   SpO2 = 110 - 25*R   (standard empirical curve), clamped to [70,100]
 */
#ifndef GF_SPO2_H
#define GF_SPO2_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GF_SPO2_WIN   256   /* ~2 s window at ~128 SPS */
#define GF_SPO2_SLIDE 128   /* recompute ~1 Hz with 50% overlap */

struct gf_spo2_result {
    uint8_t  spo2;       /* 0..100 %, 0 when invalid */
    bool     valid;
    uint16_t pi_x100;    /* IR perfusion index (AC/DC) x100, e.g. 250 = 2.50% */
    uint16_t ac;         /* IR AC amplitude (signal strength) */
};

/* Compute SpO2 over two equal-length raw-ADC windows. Bounded; n may be 0. */
struct gf_spo2_result gf_spo2_compute(const uint32_t *ir, const uint32_t *red, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* GF_SPO2_H */
