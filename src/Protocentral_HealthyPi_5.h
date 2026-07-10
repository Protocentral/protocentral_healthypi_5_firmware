/*
 * HealthyPi5 — dual-core NEXT runtime for the ProtoCentral HealthyPi 5 (RP2040).
 * ============================================================================
 * A library that encapsulates the production HealthyPi 5 NEXT dual-core AMP
 * architecture (Protocentral/healthypi5_next_rp2040) behind a small singleton
 * so the sketch stays as simple as the basic OpenView example:
 *
 *     #include <FreeRTOS.h>      // selects arduino-pico's FreeRTOS-SMP variant
 *     #include <Protocentral_HealthyPi_5.h>
 *
 *     void setup() {
 *       HealthyPi5.streamOpenView();   // pick your sink(s)
 *       HealthyPi5.begin();            // bring up the dual-core runtime
 *     }
 *     void loop() {}                   // all work runs in pinned core0 tasks
 *
 * The library OWNS both cores:
 *   - core1 (setup1/loop1, provided by the library) does acquisition ONLY and
 *     owns SPI0. It calls no FreeRTOS API; its only output is the lock-free
 *     512-deep SPSC ring (pico queue_t) and a few volatile counters.
 *   - core0 runs a broker that drains the ring and fans each sample out to one
 *     independent FreeRTOS queue per registered sink (drop-newest, counted), a
 *     1 Hz HPI_INSTR telemetry task, and a hardware-watchdog supervisor. Every
 *     core0 task is pinned to core0 so nothing migrates onto core1 and steals
 *     acquisition cycles.
 *
 * Because the library defines setup1()/loop1(), a sketch using it must NOT
 * define its own.
 */
#ifndef PROTOCENTRAL_HEALTHYPI_5_H
#define PROTOCENTRAL_HEALTHYPI_5_H

#include "hpi_types.h"

/* The runtime is built on arduino-pico's FreeRTOS-SMP core. That variant is
 * selected by the board menu (Tools > "FreeRTOS SMP", i.e. FQBN ...:os=freertos),
 * which defines __FREERTOS. Fail loudly here rather than with a confusing linker
 * error if a sketch pulls in this header without it. (board.h does not include
 * this header, so the single-core teaching example is unaffected.) */
#if !defined(__FREERTOS)
#error "HealthyPi5 requires the arduino-pico FreeRTOS-SMP variant. Select Tools > 'FreeRTOS SMP' (FQBN option os=freertos), and #include <FreeRTOS.h> before <Protocentral_HealthyPi_5.h>."
#endif

/* Maximum number of sinks that can be registered (OpenView, SD, bridge,
 * display, user callbacks...). Bump if a build needs more. */
#ifndef HPI_MAX_SINKS
#define HPI_MAX_SINKS  6
#endif

/* core0 consumer-scheduling policy. Only the FreeRTOS engine — the validated
 * NEXT architecture — is implemented today; HPI_ENGINE_COOP is reserved for a
 * future FreeRTOS-free cooperative build and currently falls back to FreeRTOS. */
enum HPIEngine {
  HPI_ENGINE_FREERTOS = 0,
  HPI_ENGINE_COOP     = 1,
};

/*
 * A consumer of acquired samples. Runs on core0. consume() MUST be bounded and
 * non-blocking enough never to back-pressure acquisition: under the FreeRTOS
 * engine each sink owns its own queue, so a slow sink only fills its queue
 * (drop-newest, counted) and can never stall core1 or another sink. Drop-newest
 * happens in the broker, upstream of consume().
 */
class HPISink {
public:
  virtual ~HPISink() {}

  /* Optional one-time init, called on the sink's own core0 task before the
   * first consume(). */
  virtual void begin() {}

  /* Handle exactly one sample. */
  virtual void consume(const hpi_sample_t &s) = 0;

  /* Per-sink queue depth for the FreeRTOS engine (samples of slack before
   * drop-newest kicks in). 256 @128 SPS is ~2 s. */
  virtual uint16_t queueDepth() const { return 256; }

  /* Sink task stack, in words. The default suits lightweight sinks; a sink that
   * pulls in a filesystem (SD/FatFS) overrides this with a larger stack. */
  virtual uint16_t stackWords() const { return 1024; }

  /* Short label for HPI_INSTR / fault dumps. */
  virtual const char *name() const { return "sink"; }
};

/* Lightweight user hook: register a function instead of a full HPISink. */
typedef void (*HPISampleCb)(const hpi_sample_t &s);

/* Computed vitals, updated ~1 Hz by the DSP sink (computeVitals()). HR comes
 * straight from the MAX30001 RTOR; SpO2 from the bounded gf_spo2 estimator over
 * the PPG; respiration from the BioZ waveform. */
struct hpi_vitals_t {
  uint16_t hr;            /* bpm, 0 = invalid                     */
  uint8_t  spo2;         /* %, 0 = invalid                       */
  uint8_t  resp_rate;    /* breaths/min, 0 = invalid             */
  uint16_t spo2_pi_x100; /* IR perfusion index (AC/DC) x100      */
  bool     hr_valid;
  bool     spo2_valid;
  bool     resp_valid;
};

class HealthyPi5Class {
public:
  HealthyPi5Class();

  /* Register sinks BEFORE begin() (the simple, deterministic order). Adding a
   * sink after begin() is also supported and starts its task immediately. */
  void streamOpenView(Stream &port = Serial);  /* built-in OpenView 2 streamer */
  void addSink(HPISink *sink);                  /* any custom sink              */
  void onSample(HPISampleCb cb);                /* function hook (DSP, logging) */

  /* Process raw samples in your own loop(). Call before begin(), then drain in
   * loop():
   *     hpi_sample_t s;
   *     while (HealthyPi5.read(s)) { ... your algorithm on s.ecg, s.ppg_ir ... }
   * The spine keeps acquiring losslessly no matter how slow loop() is — a full
   * loop queue drops the newest sample (counted in totalDrops()) and never
   * back-pressures core1. This is the "simple sketch, robust acquisition" path. */
  void     enableLoopStream(uint16_t depth = 256);
  bool     read(hpi_sample_t &s);   /* pop one sample for loop(); false if none */
  uint32_t available() const;       /* samples currently waiting for loop()     */

  /* Enable the DSP sink (A5): compute HR / SpO2 / respiration into a shared
   * vitals store, updated ~1 Hz. Call before begin(). When enabled, the OpenView
   * stream carries these computed vitals; otherwise it carries the raw
   * per-sample HR/SpO2 from the AFEs. */
  void                 computeVitals();
  bool                 vitalsEnabled() const { return _vitalsEnabled; }
  const hpi_vitals_t  &vitals() const { return _vitals; }

  /* Enable the host command/control plane (A6): a bounded "AA 55 | type | len |
   * payload | xor" parser fed from the port's RX (default USB Serial). Supports
   * PING, STREAM_START/STOP, SET_NAME, SET_AUTOSTREAM, REC_START/STOP. Call
   * before begin(). The OpenView stream is gated by STREAM_START/STOP. */
  void        enableCommands(Stream &port = Serial);
  bool        streaming() const;        /* OpenView stream gate (STREAM_*)     */
  const char *deviceName() const;       /* current SET_NAME value              */
  uint32_t    commandCount() const;     /* commands dispatched                 */

  /* Persist device_name + auto_stream across reboots (A7), in the last flash
   * sector via EEPROM. Call before begin(): it loads the stored values over the
   * defaults at boot, and the instr task flushes changes (from SET_NAME /
   * SET_AUTOSTREAM) ~1 Hz. A flush briefly parks core1 for the flash write. */
  void        persistConfig();

  /* SD recording (A8): stream the four waveforms to /REC*.BIN on a FAT card
   * (SPI1), in the decode_rec.py-compatible format. Call before begin(). Start/
   * stop via the A6 REC_START/STOP commands or directly with record*(). The
   * card is a normal drop-newest sink — a slow card never stalls acquisition. */
  void        recordSD();
  void        recordStart();            /* begin a new REC file                */
  void        recordStop();             /* close the current REC file          */
  bool        recording() const;        /* true while a file is open           */

  /* HealthyBridge link to the ESP32-C3 (A9): stream biosignals + vitals (+
   * battery) over UART1 (Serial2, RTS/CTS @921600) as byte-identical
   * HealthyBridge frames, so the existing ESP32-C3 firmware just works. Call
   * before begin(). A drop-newest sink — an absent ESP never stalls acq. */
  void        enableBridge();
  uint32_t    bridgeTxDrops() const;

  /* I2C sensors (A9): poll MAX30205 temperature + MAX17048 battery on Wire1,
   * absent-sensor-safe. Call before begin(). Feeds the OpenView temp field and
   * the HealthyBridge vitals/battery frames. */
  void        enableSensors();

  /* Temperature / battery store (written by the I2C poll, read by the sinks). */
  int16_t     temperature_x100() const { return _temp_x100; }
  bool        temperaturePresent() const { return _tempPresent; }
  uint8_t     batterySoc() const { return _battSoc; }
  uint16_t    batteryMillivolts() const { return _battMv; }
  bool        batteryPresent() const { return _battPresent; }
  void        setTemperature(int16_t x100, bool present);    /* I2C poll -> store */
  void        setBattery(uint8_t soc, uint16_t mv, bool present);

  /* NOTE: on-panel display support (A10, LVGL) is NOT part of this release. It is
   * held back pending hardware validation and will return in a later version.
   * A sketch can drive a panel today WITHOUT any library support: read vitals()
   * and read() from loop() and paint them with any graphics library. See
   * examples/Tutorials/12_Display_Vitals (ILI9488 via Arduino_GFX). */

  /* Route HPI_INSTR telemetry + HPI_FAULT dumps to a text stream. Default is
   * Serial1 (UART0, GP0/GP1) — call before begin() to override. Never route
   * this to the USB Serial that carries the binary OpenView stream. */
  void setDebug(Stream &dbg) { _dbg = &dbg; }

  /* Bring up the whole runtime: ring, sink tasks, broker, telemetry, watchdog,
   * and release core1's acquisition loop. Returns false on a setup error. */
  bool begin(HPIEngine engine = HPI_ENGINE_FREERTOS);

  /* No-op under the FreeRTOS engine (everything runs in tasks); reserved for
   * the future cooperative engine. Safe to call from loop(). */
  void update();

  /* Aggregate drop-newest count across all sink queues (for telemetry). */
  uint32_t totalDrops() const;

  /* Seconds since boot. Resets to 0 on every restart, so it doubles as an
   * "uptime without a reset" counter for soak monitoring. */
  uint32_t uptimeSeconds() const;

  /* ---- internal: invoked by the library's setup1()/loop1(). Not for sketch
   * use. Public only so the free-function core1 entry points can reach them. */
  void _core1Setup();
  void _core1Loop();

private:
  struct SinkSlot {
    HPISink         *sink;
    void            *q;     /* QueueHandle_t (opaque here to keep FreeRTOS.h
                             * out of the public header) */
    volatile uint32_t drops;
    volatile uint32_t hb;
  };

  void startSinkTask(uint8_t idx);
  void startCmdTask();

  /* FreeRTOS task entry points (static so they are plain function pointers; as
   * members they can still reach private state through the passed argument). */
  static void brokerTramp(void *arg);
  static void sinkTramp(void *arg);
  static void instrTramp(void *arg);
  static void watchdogTramp(void *arg);
  static void cmdTramp(void *arg);

  void forensicDump(const char *stalled);

  SinkSlot  _slots[HPI_MAX_SINKS];
  uint8_t   _nsinks;
  Stream   *_dbg;
  HPIEngine _engine;
  bool      _started;

  /* Optional raw-sample stream drained by the sketch's loop() (see read()). The
   * broker treats it as one more drop-newest output; there is no sink task. */
  void             *_loopQ;       /* QueueHandle_t (opaque) */
  uint16_t          _loopDepth;
  bool              _loopEnabled;
  volatile uint32_t _loopDrops;

  /* Computed-vitals store (A5). Written by the DSP sink, read by OpenView/instr
   * and the sketch via vitals(). */
  hpi_vitals_t      _vitals;
  bool              _vitalsEnabled;

  /* Host command/control plane (A6). */
  Stream           *_cmdPort;
  bool              _cmdEnabled;

  /* Persistent config (A7): load at boot, flush dirty changes from instr. */
  bool              _configEnabled;

  /* I2C sensor store (A9). */
  bool              _sensorsEnabled;
  int16_t           _temp_x100;
  bool              _tempPresent;
  uint8_t           _battSoc;
  uint16_t          _battMv;
  bool              _battPresent;
};

extern HealthyPi5Class HealthyPi5;

#endif /* PROTOCENTRAL_HEALTHYPI_5_H */
