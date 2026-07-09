# RawProcessing — your own algorithm in `loop()`

The **"simple sketch, robust acquisition"** example. The `HealthyPi5` library
runs the full dual-core NEXT spine (core1 acquires losslessly at 128 SPS into a
512-deep ring; core0 brokers it), but instead of writing a sink class you just
drain samples in `loop()` and run whatever DSP you like — like a basic Arduino
sketch.

```cpp
#include <FreeRTOS.h>
#include <HealthyPi5.h>

void setup() {
  HealthyPi5.enableLoopStream();   // raw samples -> loop()
  HealthyPi5.streamOpenView();     // optional: also feed OpenView 2 in parallel
  HealthyPi5.begin();
}

void loop() {
  hpi_sample_t s;
  while (HealthyPi5.read(s)) {
    // your algorithm on s.ecg, s.bioz, s.ppg_red, s.ppg_ir, s.hr, s.spo2 ...
  }
}
```

**Why it's safe.** `loop()` is an ordinary core0 task draining a *drop-newest*
queue. However slow your code is, acquisition never stalls — a full loop queue
drops the newest sample (counted in the `HPI_INSTR drops=` field) and can never
back-pressure core1. You cannot break lossless sampling from here.

## The API

| Call | Does |
|---|---|
| `HealthyPi5.enableLoopStream(depth=256)` | before `begin()`: open the loop stream (queue `depth` samples of slack) |
| `HealthyPi5.read(hpi_sample_t &s)` | pop one sample; returns `false` when none are waiting |
| `HealthyPi5.available()` | how many samples are queued for `loop()` right now |

`enableLoopStream()` and the OpenView/SD/etc. sinks are independent — enable any
combination. The loop stream is just one more drop-newest output of the broker.

## `hpi_sample_t` fields

```c
uint32_t seq;       // monotonic sample counter (host gap-detect)
int32_t  ecg;       // MAX30001 ECG
int32_t  bioz;      // MAX30001 BioZ (64 SPS, held between reads)
int32_t  ppg_red;   // AFE4400 RED (held between PPG cycles)
int32_t  ppg_ir;    // AFE4400 IR
uint16_t hr;        // heart rate from MAX30001 RTOR (bpm)
uint16_t rtor;      // last R-to-R interval (ms)
uint8_t  spo2;      // SpO2 % (0 = invalid / no finger)
uint8_t  leadoff;   // lead-off status
```

## Build

Same as HealthyPi5_NEXT — needs the FreeRTOS-SMP variant:

```bash
./scripts/build.sh raw
# or: arduino-cli compile --fqbn rp2040:rp2040:rpipico:os=freertos --libraries libraries examples/Tutorials/09_RawProcessing
```

> The toy high-pass + beat counter in the sketch is illustrative only (arbitrary
> thresholds) — replace it with your real processing.
