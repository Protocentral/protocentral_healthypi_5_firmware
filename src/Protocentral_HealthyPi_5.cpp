/*
 * HealthyPi5 dual-core NEXT runtime — implementation.
 * ============================================================================
 * Owns both cores. core1 (setup1/loop1, at the bottom of this file) runs
 * acquisition only and pushes into the lock-free ring. core0 runs the broker +
 * per-sink tasks + telemetry + watchdog, all pinned to core0. See Protocentral_HealthyPi_5.h.
 *
 * FreeRTOS's queue.h is included here; the pico queue_t lives only in
 * HPIRing.cpp, so the two same-named headers never collide.
 */
#ifdef __FREERTOS   /* whole runtime is FreeRTOS-only; empty TU otherwise */

#include "Protocentral_HealthyPi_5.h"
#include "HPIOpenView.h"
#include "HPIDsp.h"
#include "hpi_internal.h"
#include "cmd.h"
#include "HPIConfig.h"
#include "HPISd.h"
#include "HPIBridge.h"
#include "HPII2c.h"
#include "HPISpi1.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <hardware/watchdog.h>
#include <hardware/uart.h>

/* ===== watchdog tuning (RP2040 HW WDT max ~8.3 s) ========================== */
#define HPI_WDT_TIMEOUT_MS   4000   /* reset if not fed within this            */
#define HPI_WDT_FEED_MS      200    /* supervisor poll/feed cadence            */
#define HPI_WDT_STALL_POLLS  5      /* 200 ms * 5 = 1 s of no progress -> reset */
#define HPI_WDT_INIT_POLLS   150    /* 200 ms * 150 = 30 s grace for core1 sensor
                                     * bring-up before the acquisition heartbeat
                                     * is policed. Generous: first boot stacks up
                                     * USB enumeration + sensor resets, and the
                                     * grace only delays fault detection at start,
                                     * never relaxes steady-state protection.    */

/* Task priorities. arduino-pico runs setup()/loop() (and setup1()/loop1()) at
 * configMAX_PRIORITIES/2. Our loop()-stream consumer busy-drains at that
 * priority, so any service task placed BELOW it gets starved — that is exactly
 * why only the APP line (loop itself) appeared, with no HPI_INSTR and no
 * OpenView output. Every service task therefore sits ABOVE the loop priority;
 * they are event-driven or periodic (they block/yield when idle), so loop()
 * still gets the leftover CPU as the lowest-priority sample consumer. */
#define HPI_PRIO_LOOP    (configMAX_PRIORITIES / 2)   /* == loop()/loop1()      */
#define HPI_PRIO_WDT     (HPI_PRIO_LOOP + 2)          /* must always run        */
#define HPI_PRIO_BROKER  (HPI_PRIO_LOOP + 1)          /* must drain the ring    */
#define HPI_PRIO_SINK    (HPI_PRIO_LOOP + 1)          /* OpenView, SD, ...       */
#define HPI_PRIO_INSTR   (HPI_PRIO_LOOP + 1)          /* 1 Hz telemetry         */

/* ===== shared inter-core / inter-task state (declared in hpi_internal.h) === */
volatile bool     g_ring_ready    = false;
volatile bool     g_acq_started   = false;
volatile uint32_t g_seq_core1     = 0;
volatile uint32_t g_ring_overflow = 0;
volatile uint32_t g_ring_level_max = 0;
volatile uint16_t g_last_hr       = 0;
volatile uint8_t  g_last_spo2     = 0;
volatile uint32_t g_hb_core1      = 0;
volatile uint32_t g_hb_broker     = 0;
volatile uint8_t  g_c1_stage      = 0;
volatile uint8_t  g_acq_substep   = 0;

/* Pin a task to core0 so it never migrates to core1 and steals acquisition
 * cycles. Guarded: only used when the core builds with core-affinity support. */
static void pin_core0(TaskHandle_t h)
{
#if (defined(configUSE_CORE_AFFINITY) && (configUSE_CORE_AFFINITY == 1))
  vTaskCoreAffinitySet(h, (UBaseType_t)(1u << 0));
#else
  (void)h;
#endif
}

/* ===== construction / registration ======================================== */
HealthyPi5Class::HealthyPi5Class()
  : _nsinks(0), _dbg(&Serial1), _engine(HPI_ENGINE_FREERTOS), _started(false),
    _loopQ(nullptr), _loopDepth(256), _loopEnabled(false), _loopDrops(0),
    _vitalsEnabled(false), _cmdPort(nullptr), _cmdEnabled(false),
    _configEnabled(false), _sensorsEnabled(false),
    _temp_x100(0), _tempPresent(false), _battSoc(0), _battMv(0), _battPresent(false)
{
  _vitals = (hpi_vitals_t){ 0, 0, 0, 0, false, false, false };
  for (uint8_t i = 0; i < HPI_MAX_SINKS; i++) {
    _slots[i].sink = nullptr;
    _slots[i].q    = nullptr;
    _slots[i].drops = 0;
    _slots[i].hb    = 0;
  }
}

void HealthyPi5Class::addSink(HPISink *sink)
{
  if (!sink || _nsinks >= HPI_MAX_SINKS) return;

  uint8_t idx = _nsinks;
  _slots[idx].sink  = sink;
  _slots[idx].q     = nullptr;
  _slots[idx].drops = 0;
  _slots[idx].hb    = 0;

  if (_started) {
    /* Registered after begin(): build its queue + task now, then publish. */
    startSinkTask(idx);
  }
  _nsinks = idx + 1;     /* publish last so the broker never sees a half-slot */
}

void HealthyPi5Class::streamOpenView(Stream &port)
{
  /* One built-in OpenView sink, stored in static storage (no heap). */
  static OpenViewSink ov(port);
  addSink(&ov);
}

/* Wrap a plain callback as a sink. Allocated once at setup time. */
namespace {
class CallbackSink : public HPISink {
public:
  explicit CallbackSink(HPISampleCb cb) : _cb(cb) {}
  void        consume(const hpi_sample_t &s) override { if (_cb) _cb(s); }
  const char *name() const override { return "cb"; }
private:
  HPISampleCb _cb;
};
}

void HealthyPi5Class::onSample(HPISampleCb cb)
{
  if (!cb) return;
  addSink(new CallbackSink(cb));
}

void HealthyPi5Class::computeVitals()
{
  static DspSink dsp(&_vitals);   /* single instance; static storage (~2 KB) */
  _vitalsEnabled = true;
  addSink(&dsp);
}

void HealthyPi5Class::enableCommands(Stream &port)
{
  _cmdPort    = &port;
  _cmdEnabled = true;
  if (_started) startCmdTask();   /* registered after begin(): start now */
}

bool        HealthyPi5Class::streaming()    const { return cmd_streaming_enabled(); }
const char *HealthyPi5Class::deviceName()   const { return cmd_device_name(); }
uint32_t    HealthyPi5Class::commandCount() const { return cmd_count(); }

void HealthyPi5Class::persistConfig()
{
  _configEnabled = true;
  if (_started) hpi_config_init();   /* enabled after begin(): load now */
}

void HealthyPi5Class::recordSD()
{
  static SdSink sd;
  addSink(&sd);
}

void HealthyPi5Class::recordStart()      { hpi_rec_start(); }
void HealthyPi5Class::recordStop()       { hpi_rec_stop(); }
bool HealthyPi5Class::recording() const  { return hpi_sd_recording(); }

void HealthyPi5Class::enableBridge()
{
  static BridgeSink bridge;
  addSink(&bridge);
}
uint32_t HealthyPi5Class::bridgeTxDrops() const { return hb_tx_drops(); }

void HealthyPi5Class::enableSensors()
{
  _sensorsEnabled = true;
  if (_started) hpi_i2c_init();
}

void HealthyPi5Class::setTemperature(int16_t x100, bool present)
{
  _temp_x100 = x100;
  _tempPresent = present;
}
void HealthyPi5Class::setBattery(uint8_t soc, uint16_t mv, bool present)
{
  _battSoc = soc;
  _battMv = mv;
  _battPresent = present;
}

void HealthyPi5Class::enableLoopStream(uint16_t depth)
{
  _loopEnabled = true;
  _loopDepth   = depth;
  if (_started && !_loopQ) _loopQ = xQueueCreate(depth, sizeof(hpi_sample_t));
}

bool HealthyPi5Class::read(hpi_sample_t &s)
{
  return _loopQ && xQueueReceive((QueueHandle_t)_loopQ, &s, 0) == pdTRUE;
}

uint32_t HealthyPi5Class::available() const
{
  return _loopQ ? (uint32_t)uxQueueMessagesWaiting((QueueHandle_t)_loopQ) : 0;
}

/* ===== bring-up =========================================================== */
void HealthyPi5Class::startSinkTask(uint8_t idx)
{
  SinkSlot &sl = _slots[idx];
  uint16_t depth = sl.sink->queueDepth();
  sl.q = xQueueCreate(depth, sizeof(hpi_sample_t));

  char nm[8];
  snprintf(nm, sizeof(nm), "snk%u", (unsigned)idx);
  TaskHandle_t h;
  xTaskCreate(sinkTramp, nm, sl.sink->stackWords(), &_slots[idx], HPI_PRIO_SINK, &h);
  pin_core0(h);
}

/* The command parser's response port (one command source for now: USB). */
static Stream *s_cmd_resp_port = nullptr;
static void cmd_respond(const uint8_t *buf, size_t len)
{
  if (s_cmd_resp_port) s_cmd_resp_port->write(buf, len);
}

void HealthyPi5Class::startCmdTask()
{
  s_cmd_resp_port = _cmdPort;
  TaskHandle_t h;
  xTaskCreate(cmdTramp, "cmd", 512, this, HPI_PRIO_SINK, &h);
  pin_core0(h);
}

bool HealthyPi5Class::begin(HPIEngine engine)
{
  _engine = engine;

  /* Debug/telemetry console. If left at the default Serial1, bring up UART0. */
  if (_dbg == &Serial1) {
    Serial1.setTX(HPI_PIN_UART0_TX);
    Serial1.setRX(HPI_PIN_UART0_RX);
    Serial1.begin(HPI_UART0_BAUD);
  }
  Serial.begin(115200);                 /* USB-CDC -> OpenView 2 (binary)      */

  if (engine == HPI_ENGINE_COOP) {
    _dbg->printf("HPI_WARN COOP engine not implemented; using FreeRTOS\r\n");
    _engine = HPI_ENGINE_FREERTOS;
  }

  /* Reset reason from the LAST boot (read before we re-arm the HW WDT below):
   * wdt_caused=1 -> the hardware watchdog fired (a >4 s core hang / CPU lockup,
   * which bypasses the fault handlers); wdt_caused=0 -> power-on / brownout /
   * external reset (a hardware/power event). This distinguishes the message-less
   * display reset between a lockup and a supply problem. */
  _dbg->printf("\r\nHPI_RESET wdt_caused=%d\r\n", (int)watchdog_caused_reboot());

  _dbg->printf("HPI_BOOT HealthyPi5 NEXT (Arduino dual-core) "
               "ring=%d ecg=%dSPS sinks=%u\r\n",
               HPI_RING_DEPTH, HPI_ECG_SPS, (unsigned)_nsinks);

  cmd_init();                            /* control state: streaming on, etc.   */
  if (_configEnabled) hpi_config_init(); /* override defaults from flash (A7)   */
  if (_sensorsEnabled) {
    hpi_i2c_init();                       /* probe temp/battery on Wire1 (A9)    */
    _dbg->printf("HPI_TEMP sensor=%s\r\n", hpi_temp_sensor_name()); /* QWIIC temp */
  }
  hpi_spi1_bus_init();                    /* SD SPI1 bus mutex                   */
  hpi_ring_init();                       /* MUST precede core1 touching it      */

  /* ---- bring the SPINE up first, before any (possibly slow) sink/peripheral
   * init, so core1 starts acquiring immediately and the watchdog + broker are
   * running independent of how long a sink's begin() takes. ---- */
  rp2040.wdt_begin(HPI_WDT_TIMEOUT_MS);  /* arm HW WDT before the supervisor    */
  g_ring_ready = true;                    /* release core1's setup1() spin-wait  */

  TaskHandle_t h;
  xTaskCreate(watchdogTramp, "wdt",   1024, this, HPI_PRIO_WDT,    &h); pin_core0(h);
  xTaskCreate(brokerTramp,   "brk",    512, this, HPI_PRIO_BROKER, &h); pin_core0(h);
  xTaskCreate(instrTramp,    "instr", 1024, this, HPI_PRIO_INSTR,  &h); pin_core0(h);

  /* ---- consumers attach after the spine is live. A slow sink begin() (e.g. an
   * SD mount) now only delays that sink, never core1 or the broker. The broker
   * skips any sink whose queue isn't created yet (sl.q == nullptr). ---- */
  for (uint8_t i = 0; i < _nsinks; i++) startSinkTask(i);
  if (_cmdEnabled) startCmdTask();
  if (_loopEnabled && !_loopQ) _loopQ = xQueueCreate(_loopDepth, sizeof(hpi_sample_t));

  _started = true;
  return true;
}

void HealthyPi5Class::update()
{
  /* FreeRTOS engine: all work runs in tasks; nothing to do here. */
}

uint32_t HealthyPi5Class::totalDrops() const
{
  uint32_t d = _loopDrops;
  for (uint8_t i = 0; i < _nsinks; i++) d += _slots[i].drops;
  return d;
}

uint32_t HealthyPi5Class::uptimeSeconds() const
{
  return millis() / 1000;   /* millis() resets on reboot -> uptime since boot */
}

/* ===== core0 tasks ======================================================== */
void HealthyPi5Class::brokerTramp(void *arg)
{
  HealthyPi5Class *self = (HealthyPi5Class *)arg;
  hpi_sample_t s;

  for (;;) {
    /* Drain everything currently in the ring, then sleep one tick. The 512-deep
     * ring absorbs the ~1 ms polling latency, so this is lossless without
     * busy-spinning a core. */
    bool got_any = false;
    while (hpi_ring_pop(&s)) {
      got_any = true;

      uint32_t lvl = hpi_ring_level() + 1;        /* post-pop level + this one */
      if (lvl > g_ring_level_max) g_ring_level_max = lvl;

      g_last_hr   = s.hr;                          /* vitals snapshot for instr */
      g_last_spo2 = s.spo2;

      /* Fan out to every sink's queue. Drop-newest (timeout 0) so a stalled
       * sink never blocks the broker; each drop is counted per-sink. */
      for (uint8_t i = 0; i < self->_nsinks; i++) {
        SinkSlot &sl = self->_slots[i];
        if (sl.q && xQueueSend((QueueHandle_t)sl.q, &s, 0) != pdTRUE) sl.drops++;
      }

      /* ...and to the sketch's loop() stream, if enabled (drop-newest too). */
      if (self->_loopQ &&
          xQueueSend((QueueHandle_t)self->_loopQ, &s, 0) != pdTRUE) {
        self->_loopDrops++;
      }
    }

    g_hb_broker++;                                 /* heartbeat for the watchdog */
    if (!got_any) vTaskDelay(1);                   /* nothing pending: yield     */
  }
}

void HealthyPi5Class::sinkTramp(void *arg)
{
  SinkSlot *slot = (SinkSlot *)arg;
  slot->sink->begin();

  hpi_sample_t s;
  for (;;) {
    if (xQueueReceive((QueueHandle_t)slot->q, &s, portMAX_DELAY) == pdTRUE) {
      slot->hb++;
      slot->sink->consume(s);
    }
  }
}

void HealthyPi5Class::cmdTramp(void *arg)
{
  HealthyPi5Class *self = (HealthyPi5Class *)arg;
  Stream *port = self->_cmdPort;

  cmd_parser_t parser;
  cmd_parser_init(&parser, cmd_respond);

  for (;;) {
    /* Feed every available RX byte through the bounded parser, then sleep. The
     * parser is best-effort (not watchdogged): a quiet control link is normal. */
    int c, fed = 0;
    while ((c = port->read()) >= 0) { cmd_parse_byte(&parser, (uint8_t)c); fed++; }
    if (fed == 0) vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void HealthyPi5Class::instrTramp(void *arg)
{
  HealthyPi5Class *self = (HealthyPi5Class *)arg;
  TickType_t last = xTaskGetTickCount();
  uint32_t   prev_seq = 0;

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));

    uint32_t seq = g_seq_core1;
    uint32_t sps = seq - prev_seq;                 /* effective samples/s        */
    prev_seq = seq;

    /* Emit on the debug stream (UART0), NOT the USB-CDC OpenView port. key=value.
     * FORMAT CAVEAT: reconcile keys with seqcheck.py before declaring host
     * parity. */
    /* HR/SpO2/RR from the DSP vitals when enabled, else the raw sample values. */
    unsigned hr   = self->_vitalsEnabled ? self->_vitals.hr        : g_last_hr;
    unsigned spo2 = self->_vitalsEnabled ? self->_vitals.spo2      : g_last_spo2;
    unsigned rr   = self->_vitalsEnabled ? self->_vitals.resp_rate : 0;

    uint32_t up = self->uptimeSeconds();   /* up= shown as <h>h<mm>m */

    self->_dbg->printf("HPI_INSTR seq=%lu sps=%lu c1stage=%u ring=%lu/%lu ringmax=%lu "
                       "ovf=%lu drops=%lu hr=%u spo2=%u rr=%u temp=%d batt=%u "
                       "stream=%u cmds=%lu hbtx=%lu hbdrop=%lu up=%luh%02lum "
                       "hb_c1=%lu hb_brk=%lu\r\n",
                       (unsigned long)seq,
                       (unsigned long)sps,
                       (unsigned)g_c1_stage,
                       (unsigned long)hpi_ring_level(),
                       (unsigned long)hpi_ring_capacity(),
                       (unsigned long)g_ring_level_max,
                       (unsigned long)g_ring_overflow,
                       (unsigned long)self->totalDrops(),
                       hr, spo2, rr,
                       (int)self->_temp_x100, (unsigned)self->_battSoc,
                       (unsigned)cmd_streaming_enabled(),
                       (unsigned long)cmd_count(),
                       (unsigned long)hb_tx_frames(), (unsigned long)hb_tx_drops(),
                       (unsigned long)(up / 3600), (unsigned long)((up / 60) % 60),
                       (unsigned long)g_hb_core1,
                       (unsigned long)g_hb_broker);

    /* Poll the I2C temp/battery sensors here at 1 Hz (A9). */
    if (self->_sensorsEnabled) hpi_i2c_poll();

    /* Flush a changed persisted setting to flash here, off the data hot path.
     * The commit briefly parks core1 — see HPIConfig. No-op unless dirty. */
    if (self->_configEnabled) hpi_config_flush_if_dirty();
  }
}

void HealthyPi5Class::forensicDump(const char *stalled)
{
  _dbg->printf("\r\nHPI_FAULT stalled=%s\r\n", stalled);
  _dbg->printf("  c1stage=%u acqstep=%u acq=%u hb_core1=%lu hb_broker=%lu\r\n",
               (unsigned)g_c1_stage, (unsigned)g_acq_substep,
               (unsigned)g_acq_started,
               (unsigned long)g_hb_core1, (unsigned long)g_hb_broker);
  _dbg->printf("  seq=%lu ring=%lu/%lu ovf=%lu drops=%lu\r\n",
               (unsigned long)g_seq_core1,
               (unsigned long)hpi_ring_level(),
               (unsigned long)hpi_ring_capacity(),
               (unsigned long)g_ring_overflow,
               (unsigned long)totalDrops());
  _dbg->flush();
}

void HealthyPi5Class::watchdogTramp(void *arg)
{
  HealthyPi5Class *self = (HealthyPi5Class *)arg;
  uint32_t prev_c1 = 0, prev_brk = 0;
  uint8_t  miss_c1 = 0, miss_brk = 0;
  uint16_t init_polls = 0;
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(HPI_WDT_FEED_MS));

    uint32_t c1 = g_hb_core1, brk = g_hb_broker;
    const char *stalled = nullptr;

    /* The broker is the always-on ring drainer; police it from boot. */
    miss_brk = (brk == prev_brk) ? (uint8_t)(miss_brk + 1) : 0;
    prev_brk = brk;
    if (miss_brk >= HPI_WDT_STALL_POLLS) stalled = "broker";

    if (!g_acq_started) {
      /* core1 is still bringing up the sensors (SPI + AFE4400 + MAX30001 resets,
       * which include blocking delays). Don't police its heartbeat yet — that
       * would reset the chip mid-init and boot-loop forever. Allow a grace; if
       * core1 hasn't reached streaming by then, it is genuinely hung (the dump's
       * `c1stage` says where). */
      if (++init_polls >= HPI_WDT_INIT_POLLS) stalled = "core1-init";
    } else {
      /* Acquisition is live: the pace loop must bump every period. */
      miss_c1 = (c1 == prev_c1) ? (uint8_t)(miss_c1 + 1) : 0;
      prev_c1 = c1;
      if (!stalled && miss_c1 >= HPI_WDT_STALL_POLLS) stalled = "core1";
    }

    /* The sink/instr tasks are best-effort and deliberately NOT watchdogged: a
     * sink legitimately blocks when no samples flow or a slow host back-presses,
     * and that must never reset the device. */
    if (stalled) {
      self->forensicDump(stalled);
      for (;;) { /* stop feeding -> HW WDT resets in <= HPI_WDT_TIMEOUT_MS */ }
    }

    rp2040.wdt_reset();                    /* all heartbeats healthy: feed       */
  }
}

/* ===== core1 — acquisition only =========================================== */
void HealthyPi5Class::_core1Setup()
{
  /* Busy-wait until core0 has initialised the ring, then bring up SPI0 + both
   * AFEs. No FreeRTOS API on core1. hpi_acq_init() advances g_c1_stage (3..6)
   * so a fault during sensor bring-up is pinpointed. */
  while (!g_ring_ready) { tight_loop_contents(); }
  g_c1_stage = 1;
  hpi_acq_init();
  g_acq_started = true;
}

void HealthyPi5Class::_core1Loop()
{
  static uint32_t next_us = 0;
  static bool     primed  = false;

  if (!g_acq_started) { primed = false; return; }

  /* Seed the pacing clock on the first cycle after acquisition starts. Without
   * this, next_us is still 0 (g_acq_started is set in setup1, before loop1 ever
   * runs) so core1 would "catch up" from t=0 and burst hundreds of samples at
   * boot — the ring absorbs it losslessly, but it pointlessly spikes occupancy. */
  if (!primed) { next_us = micros(); primed = true; }
  g_c1_stage = 7;

  /* Busy-pace to the next 128 SPS deadline (rollover-safe), then acquire. No
   * FreeRTOS call here — this is the inviolable acquisition path. */
  while ((int32_t)(micros() - next_us) < 0) { tight_loop_contents(); }
  next_us += HPI_ACQ_PERIOD_US;

  hpi_acq_step();
  g_hb_core1++;
}

/* ===== fault hooks — override the core's SILENT-reset weak defaults so a
 * stack overflow or heap exhaustion prints which task/why before resetting.
 * (This is what makes an otherwise message-less abrupt reset diagnosable.) ==== */
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  Serial1.printf("\r\nHPI_STACKOVF task=%s\r\n", pcTaskName ? pcTaskName : "?");
  Serial1.flush();
  watchdog_reboot(0, 0, 0);
}
extern "C" void vApplicationMallocFailedHook(void)
{
  Serial1.printf("\r\nHPI_MALLOCFAIL\r\n");
  Serial1.flush();
  watchdog_reboot(0, 0, 0);
}

/* Hard-fault handler: a CPU fault (bad pointer, unaligned access, etc.) is NOT a
 * FreeRTOS hook — the default handler just hangs until the HW WDT resets, with
 * no message. Override it to write a marker straight to UART0 (raw, ISR-safe)
 * then reboot, so a fault-driven reset is no longer silent. uart0 == Serial1. */
extern "C" void isr_hardfault(void)
{
  const char *m = "\r\nHPI_HARDFAULT\r\n";
  while (*m) {
    while (!uart_is_writable(uart0)) { }
    uart_putc_raw(uart0, *m++);
  }
  watchdog_reboot(0, 0, 0);
}

/* ===== singleton + the core1 entry points the arduino-pico core calls ====== */
HealthyPi5Class HealthyPi5;

/* The library OWNS core1. These strong definitions override arduino-pico's weak
 * setup1()/loop1(), so a sketch using HealthyPi5 must NOT define its own. */
void setup1() { HealthyPi5._core1Setup(); }
void loop1()  { HealthyPi5._core1Loop(); }

#endif /* __FREERTOS */
