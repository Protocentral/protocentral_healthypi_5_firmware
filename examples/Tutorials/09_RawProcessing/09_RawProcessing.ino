/*
 * HealthyPi 5 NEXT — process raw samples in loop() with your own algorithm.
 * ============================================================================
 * The "simple sketch, robust acquisition" example. The HealthyPi5 library still
 * runs the full dual-core NEXT spine (core1 acquires losslessly at 128 SPS into
 * a 512-deep ring; core0 brokers it), but instead of writing a sink class you
 * just drain the samples in loop() and run whatever DSP you like — exactly like
 * a basic Arduino sketch.
 *
 * Why this is safe: loop() is an ordinary core0 task draining a drop-newest
 * queue. However slow your code in loop() is, acquisition NEVER stalls — a full
 * loop queue simply drops the newest sample (counted in HPI_INSTR `drops=`) and
 * can never back-pressure core1. You cannot break lossless sampling from here.
 *
 *   Board  : "Raspberry Pi Pico"  (arduino-pico)  — Tools > os: "FreeRTOS SMP"
 *   Output : USB-CDC (Serial)  -> OpenView 2  (still streamed, optional)
 *            UART0  (Serial1)  -> your prints + the library's HPI_INSTR line
 *
 * The algorithm below (1-pole high-pass + threshold beat counter) is a TOY to
 * show the mechanism — the thresholds are arbitrary, not a real HR detector.
 * Replace the body of the while-loop with your own processing.
 */
#include <FreeRTOS.h>        // selects arduino-pico's FreeRTOS-SMP variant
#include <Protocentral_HealthyPi_5.h>

// ---- your algorithm's state -----------------------------------------------
static float    hp_prev_in  = 0.0f;   // 1-pole high-pass (removes baseline wander)
static float    hp_prev_out = 0.0f;
static uint32_t beats = 0;
static bool     above = false;

void setup()
{
  HealthyPi5.enableLoopStream();   // raw samples -> loop() (your own DSP)
  HealthyPi5.computeVitals();      // library DSP sink: HR + SpO2 + respiration.
                                   // Without this the vitals store stays 0, so
                                   // the OpenView stream carries RR = 0. It is a
                                   // separate broker sink — enabling it does NOT
                                   // take samples away from your loop().
  HealthyPi5.streamOpenView();     // optional: also feed OpenView 2 in parallel
  HealthyPi5.begin();              // dual-core spine + broker + watchdog
}

void loop()
{
  // Drain everything available, run your DSP on each raw sample.
  hpi_sample_t s;
  while (HealthyPi5.read(s)) {
    float x = (float)s.ecg;

    // 1-pole high-pass ~0.5 Hz: y[n] = a*(y[n-1] + x[n] - x[n-1])
    float y = 0.95f * (hp_prev_out + x - hp_prev_in);
    hp_prev_in  = x;
    hp_prev_out = y;

    // Toy beat detector: count upward threshold crossings (TUNE for your signal).
    const float THRESH = 30000.0f;
    if (!above && y > THRESH)            { above = true; beats++; }
    else if (above && y < THRESH * 0.5f) { above = false; }

    // s.ppg_red / s.ppg_ir / s.bioz / s.hr / s.spo2 are all here too.
  }

  // Report once a second on the debug UART (NOT the binary OpenView USB port).
  // Alongside our toy beat count, print the library's computed vitals so RR is
  // visible here too. RR reads 0 for the first ~10 s and until >=3 breaths are
  // detected (the resp detector's warm-up); HR/SpO2 need contact / a finger.
  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last += 1000;
    const hpi_vitals_t &v = HealthyPi5.vitals();
    Serial1.printf("APP up=%lus beats=%lu queued=%lu | HR=%u SpO2=%u RR=%u\r\n",
                   (unsigned long)HealthyPi5.uptimeSeconds(),
                   (unsigned long)beats,
                   (unsigned long)HealthyPi5.available(),
                   v.hr, v.spo2, v.resp_rate);
  }
}
