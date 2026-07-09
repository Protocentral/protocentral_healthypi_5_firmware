/*
 * SPDX-License-Identifier: MIT
 * Clean-room SpO2 estimator — see gf_spo2.h.
 */
#include "gf_spo2.h"

/* Validity floors in raw AFE4400 ADC units (after the driver's >>10 scaling).
 * Tunable; chosen to admit the real signals seen on this hardware (DC a few
 * thousand, AC ~100-300, PI ~2-7%) while rejecting probe-off / no-pulse. */
#define GF_DC_MIN    500    /* finger present: DC above this        */
#define GF_AC_MIN    40     /* pulsatile: IR AC above this          */
#define GF_PI_MIN    30     /* 0.30% perfusion floor                */

struct gf_spo2_result gf_spo2_compute(const uint32_t *ir, const uint32_t *red, size_t n)
{
    struct gf_spo2_result r = { .spo2 = 0, .valid = false, .pi_x100 = 0, .ac = 0 };
    if (n == 0) {
        return r;
    }

    uint64_t sum_ir = 0, sum_red = 0;
    uint32_t min_ir = UINT32_MAX, max_ir = 0;
    uint32_t min_red = UINT32_MAX, max_red = 0;

    for (size_t i = 0; i < n; i++) {
        uint32_t a = ir[i], b = red[i];
        sum_ir += a; sum_red += b;
        if (a < min_ir)  min_ir = a;
        if (a > max_ir)  max_ir = a;
        if (b < min_red) min_red = b;
        if (b > max_red) max_red = b;
    }

    uint32_t dc_ir  = (uint32_t)(sum_ir / n);
    uint32_t dc_red = (uint32_t)(sum_red / n);
    uint32_t ac_ir  = max_ir - min_ir;
    uint32_t ac_red = max_red - min_red;
    r.ac = (ac_ir > UINT16_MAX) ? UINT16_MAX : (uint16_t)ac_ir;

    /* Finger present + pulsatile? */
    if (dc_ir < GF_DC_MIN || dc_red < GF_DC_MIN ||
        ac_ir < GF_AC_MIN || ac_ir == 0 || dc_red == 0) {
        return r;   /* probe off / no pulse -> invalid */
    }

    r.pi_x100 = (uint16_t)(((uint64_t)ac_ir * 10000U) / dc_ir);
    if (r.pi_x100 < GF_PI_MIN) {
        return r;
    }

    /* R = (AC_red/DC_red) / (AC_ir/DC_ir) = (AC_red*DC_ir)/(AC_ir*DC_red).
     * Scale by 1000 for integer precision; 64-bit guards overflow. */
    uint64_t den = (uint64_t)ac_ir * dc_red;
    if (den == 0) {
        return r;
    }
    uint32_t r_x1000 = (uint32_t)(((uint64_t)ac_red * dc_ir * 1000U) / den);

    /* SpO2 = 110 - 25*R. */
    int32_t spo2 = 110 - (int32_t)((25U * r_x1000) / 1000U);
    if (spo2 > 100) {
        spo2 = 100;
    }
    if (spo2 < 70) {
        return r;   /* implausibly low -> poor signal, report invalid */
    }

    r.spo2 = (uint8_t)spo2;
    r.valid = true;
    return r;
}
