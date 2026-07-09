/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2025 ProtoCentral Electronics
 *
 * Respiration-rate detection — ported verbatim from app/src/resp_process.c.
 * The algorithm is unchanged; only the Zephyr kernel/logging includes are
 * replaced with stdint + no-op LOG_* macros so it builds bare-metal on the Pico.
 */
#include <stdint.h>
#include <stdbool.h>

#include "resp_process.h"

/* Zephyr logging shim — the algorithm's LOG_* calls become no-ops. */
#define LOG_INF(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define LOG_WRN(...) ((void)0)

// Module-level respiration rate (persists between calls; the detector's other
// working state lives in the statics inside resp_rate_detect).
uint8_t Respiration_Rate = 0;

// Calibration factor for rate calculation (see app/src/resp_process.c for the
// derivation): Rate (BPM) = RESP_CALIBRATION_FACTOR / interval_in_samples, with
// resp_rate_detect fed at ~32 Hz (128 SPS / 4).
#define RESP_CALIBRATION_FACTOR 2980

void resp_process_sample(int16_t *CurrAqsSample, int16_t *respFiltered)
{
    // Pass through without filtering for now
    // Can add gentle lowpass filter here if needed for noise reduction
    for (int i = 0; i < 4; i++)
    {
        respFiltered[i] = CurrAqsSample[i];
    }
}

void resp_algo_process(int16_t *CurrSample, volatile uint8_t *RespirationRate)
{
    // Average the 4 input samples
    long Mac = 0;
    for (int k = 0; k < 4; k++)
    {
        Mac += CurrSample[k];
    }

    // Pass averaged sample to rate detection
    resp_rate_detect((int16_t)(Mac >> 2), RespirationRate);
}

void resp_rate_detect(int16_t Resp_wave, volatile uint8_t *RespirationRate)
{
    // State variables
    static int32_t baseline = 0;          // Running average baseline (Q16 fixed-point)
    static uint16_t samplesSinceCrossing = 0;
    static uint16_t breathIntervals[8] = {0};  // Store last 8 breath intervals
    static uint8_t intervalIndex = 0;
    static uint8_t validIntervals = 0;
    static uint16_t startupDelay = 312;   // Skip first 10 seconds
    static bool waitingForLow = false;     // State machine for hysteresis
    static int16_t minAmplitude = 32767;   // Track signal amplitude (initialized to max int16)
    static int16_t maxAmplitude = -32768;  // Track signal amplitude (initialized to min int16)
    static uint16_t amplitudeCheckTimer = 0;

    // Skip startup transient
    if (startupDelay > 0)
    {
        startupDelay--;
        baseline = (int32_t)Resp_wave << 16;  // Initialize baseline
        if (startupDelay == 0)
        {
            LOG_INF("RESP: Adaptive threshold detection started, initial baseline=%d", (int16_t)(baseline >> 16));
        }

        // Log a few samples during startup to see signal
        if (startupDelay % 31 == 0)  // Every ~1 second
        {
            LOG_DBG("RESP startup: sample=%d, baseline=%d", Resp_wave, (int16_t)(baseline >> 16));
        }
        return;
    }

    // Update baseline using exponential moving average
    // Alpha = 1/256 gives very slow tracking to follow DC drift only, not the breath signal
    // For 20 BPM: period = 3 sec = 94 samples, time constant should be >> 94 samples
    // Alpha = 1/256 gives time constant of 256 samples (~8 sec), good for 8-80 BPM range
    baseline = baseline - (baseline >> 8) + ((int32_t)Resp_wave << 8);
    int16_t baselineValue = (int16_t)(baseline >> 16);

    // Track amplitude for signal quality monitoring
    if (Resp_wave < minAmplitude) minAmplitude = Resp_wave;
    if (Resp_wave > maxAmplitude) maxAmplitude = Resp_wave;

    amplitudeCheckTimer++;
    if (amplitudeCheckTimer >= 125)  // Every 4 seconds
    {
        int16_t peakToPeak = maxAmplitude - minAmplitude;
        LOG_DBG("RESP: Amplitude=%d, Baseline=%d, Signal range: [%d, %d]",
                peakToPeak, baselineValue, minAmplitude, maxAmplitude);

        // Check for signal loss
        static uint32_t weak_signal_count = 0;
        if (peakToPeak < 50)
        {
            // Only log every 50th occurrence to reduce noise
            if (++weak_signal_count % 50 == 1) {
                LOG_WRN("RESP: Weak signal (count=%u)", weak_signal_count);
            }
            Respiration_Rate = 0;
            validIntervals = 0;
        }

        minAmplitude = 32767;
        maxAmplitude = -32768;
        amplitudeCheckTimer = 0;
    }

    // Increment sample counter
    samplesSinceCrossing++;

    // Periodic logging to see signal values
    static uint16_t signalLogTimer = 0;
    if (signalLogTimer++ >= 31)  // Every ~1 second
    {
        signalLogTimer = 0;
        LOG_DBG("RESP signal: wave=%d, baseline=%d, upper=%d, lower=%d, state=%s",
                Resp_wave, baselineValue, baselineValue + 40, baselineValue - 40,
                waitingForLow ? "waitLow" : "waitHigh");
    }

    // State machine with hysteresis to detect breath cycle
    // Reduced hysteresis for better sensitivity - using ±20 instead of ±40
    // This allows detection of smaller amplitude variations
    int16_t hysteresis = 20;
    int16_t upperThreshold = baselineValue + hysteresis;
    int16_t lowerThreshold = baselineValue - hysteresis;

    if (!waitingForLow)
    {
        // State: Above baseline, waiting to go below
        // Looking for end of inhalation / start of exhalation
        if (Resp_wave < lowerThreshold)
        {
            waitingForLow = true;
            LOG_DBG("RESP: Crossed below baseline (sample %d)", samplesSinceCrossing);
        }
    }
    else
    {
        // State: Below baseline, waiting to go above
        // Looking for end of exhalation / start of inhalation (= one breath complete)
        if (Resp_wave > upperThreshold)
        {
            // Complete breath cycle detected!
            waitingForLow = false;

            // Validate interval using calibrated ranges
            // With CALIBRATION_FACTOR = 2980:
            //   80 BPM: 2980 / 80 = 37 samples minimum
            //   8 BPM: 2980 / 8 = 372 samples maximum
            // Use slightly wider range for safety: 30 to 400 samples
            if (samplesSinceCrossing >= 30 && samplesSinceCrossing <= 400)
            {
                // Store this breath interval
                breathIntervals[intervalIndex] = samplesSinceCrossing;
                intervalIndex++;
                if (intervalIndex >= 8)
                {
                    intervalIndex = 0;
                }

                if (validIntervals < 8)
                {
                    validIntervals++;
                }

                // Calculate and display the breath rate from this single interval
                // Using calibrated formula: Rate = CALIBRATION_FACTOR / interval
                uint16_t instantRate = RESP_CALIBRATION_FACTOR / samplesSinceCrossing;
                LOG_INF("RESP: Breath detected! Interval=%d samples = %d BPM, validIntervals=%d",
                        samplesSinceCrossing, instantRate, validIntervals);

                // Calculate rate if we have enough data (at least 3 breaths)
                if (validIntervals >= 3)
                {
                    // Average the stored breath intervals
                    uint32_t sum = 0;
                    for (int i = 0; i < validIntervals; i++)
                    {
                        sum += breathIntervals[i];
                    }
                    uint16_t avgInterval = sum / validIntervals;

                    // Calculate respiration rate using calibration factor
                    uint16_t calculatedRate = RESP_CALIBRATION_FACTOR / avgInterval;

                    LOG_INF("RESP: Calculated rate=%d BPM from avgInterval=%d (factor=%d)",
                            calculatedRate, avgInterval, RESP_CALIBRATION_FACTOR);

                    // Sanity check: rate should be 8-80 BPM
                    if (calculatedRate >= 8 && calculatedRate <= 80)
                    {
                        Respiration_Rate = (uint8_t)calculatedRate;
                        LOG_INF("RESP RATE: %d BPM (avg interval: %d samples, n=%d)",
                                Respiration_Rate, avgInterval, validIntervals);
                    }
                    else
                    {
                        LOG_WRN("RESP: Calculated rate %d BPM out of range (expected 8-80)", calculatedRate);
                    }
                }
                else
                {
                    LOG_DBG("RESP: Waiting for more breaths (%d/3)", validIntervals);
                }
            }
            else
            {
                // Invalid interval - log sparingly to reduce noise
                static uint32_t invalid_interval_count = 0;
                if (++invalid_interval_count % 100 == 1) {
                    LOG_WRN("RESP: Invalid intervals (count=%u)", invalid_interval_count);
                }

                // If interval is way too long, might have lost signal
                if (samplesSinceCrossing > 400)
                {
                    validIntervals = 0;  // Reset, need fresh measurements
                }
            }

            // Reset counter for next breath
            samplesSinceCrossing = 0;
        }
    }

    // Timeout detection: if no crossing for >10 seconds, reset
    if (samplesSinceCrossing > 312)  // 312 samples × 32ms = 10 seconds
    {
        static uint32_t timeout_count = 0;
        // Only log every 10th timeout to reduce noise
        if (++timeout_count % 10 == 1) {
            LOG_WRN("RESP: No breath >10s (count=%u)", timeout_count);
        }
        Respiration_Rate = 0;
        validIntervals = 0;
        waitingForLow = false;
        samplesSinceCrossing = 0;
    }

    // Update output
    *RespirationRate = Respiration_Rate;
}
