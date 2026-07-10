/*
 * HealthyPi 5 NEXT — vitals + ECG on an ILI9488 display, driven from the sketch.
 * ============================================================================
 * The display is NOT part of the HealthyPi5 library. This sketch reads vitals
 * and raw samples through the ordinary public API and paints them itself with a
 * third-party graphics library. Nothing here reaches into the library's
 * internals, so the library keeps zero display dependencies:
 *
 *     HealthyPi5.vitals()            -> HR / SpO2 / respiration  (DSP sink, ~1 Hz)
 *     HealthyPi5.temperature_x100()  -> body temperature         (I2C, ~1 Hz)
 *     HealthyPi5.batterySoc()        -> fuel gauge               (I2C, ~1 Hz)
 *     HealthyPi5.read(s)             -> raw 128 SPS samples for the ECG trace
 *
 *   Board    : "Raspberry Pi Pico" (arduino-pico) — Tools > os: "FreeRTOS SMP"
 *   Display  : ILI9488 320x480 SPI, on SPI1 (shared with the microSD slot)
 *   Libraries: HealthyPi5 (this repo), protocentral_max30001,
 *              protocentral_afe44xx, "GFX Library for Arduino" (Arduino_GFX)
 *
 * WHY THIS IS SAFE TO DO IN loop()
 *   loop() is an ordinary, lowest-priority core0 task draining a drop-newest
 *   queue. An SPI display is slow; if painting can't keep up with 128 SPS, the
 *   broker drops the newest sample into `drops=` (visible in the 1 Hz HPI_INSTR
 *   line on UART0) and moves on. Acquisition on core1 NEVER stalls, and no other
 *   sink is affected. You cannot break lossless sampling from here.
 *
 * ILI9488 NOTE: over SPI this controller does not accept 16-bit RGB565 pixels —
 * it needs 18-bit (RGB666). That is why the panel class below is the *_18bit
 * variant. Colours are still written as RGB565 constants; the driver expands
 * them. Using the plain Arduino_ILI9488 class over SPI gives a blank/garbled
 * screen.
 *
 * SPI1 IS A SHARED BUS. The display and the microSD card sit on SPI1 behind
 * separate chip-selects. This sketch does not call HealthyPi5.recordSD(), so it
 * is the only user of the bus. If you add SD recording, every draw below must be
 * wrapped in hpi_spi1_lock()/hpi_spi1_unlock() — see the note near drawWave().
 */
#include <FreeRTOS.h>        // selects arduino-pico's FreeRTOS-SMP variant
#include <Protocentral_HealthyPi_5.h>
#include <Arduino_GFX_Library.h>

// ---- display wiring: from the library's board.h. Never hard-code a GPIO. -----
// Arduino_HWSPI drives the Arduino `SPI1` object (not the pico-sdk spi1 handle),
// which is the same object the library's SD sink uses — so the two can coexist.
static Arduino_DataBus *bus = new Arduino_HWSPI(HPI_PIN_LCD_DC, HPI_PIN_LCD_CS, &SPI1);
static Arduino_GFX     *gfx = new Arduino_ILI9488_18bit(bus, HPI_PIN_LCD_RST,
                                                        1 /* rotation: landscape */,
                                                        false /* not an IPS panel */);

// ---- layout (480x320 after rotation) ---------------------------------------
#define HDR_H        28                       // title bar
#define TILE_Y       (HDR_H + 6)              // vitals row
#define TILE_H       84
#define WAVE_TOP     (TILE_Y + TILE_H + 8)    // ECG trace
#define WAVE_BOT     314
#define WAVE_H       (WAVE_BOT - WAVE_TOP)
#define WAVE_MID     (WAVE_TOP + WAVE_H / 2)

#define COL_BG       RGB565_BLACK
#define COL_HDR      RGB565_DARKCYAN
#define COL_LABEL    RGB565_LIGHTGREY
#define COL_TRACE    RGB565_GREENYELLOW
#define COL_STALE    RGB565_DIMGREY          // a vital the DSP has not validated

// ---- ECG trace tuning -------------------------------------------------------
// Same 1-pole high-pass as 01_ECG_Plotter (baseline-wander removal), then a
// fixed vertical scale. ECG_FULLSCALE maps to half the trace height; 40000 LSB
// suits the R-peak amplitude that 09_RawProcessing's beat detector thresholds
// at 30000. Lower it to magnify a small signal.
static const float   HP_ALPHA      = 0.95f;
static const int32_t ECG_FULLSCALE = 40000;

static float   hp_in = 0.0f, hp_out = 0.0f;
static int16_t wave_x = 0;      // current column
static int16_t prev_y = -1;     // previous sample's pixel row (-1 = pen up)

// Cached vitals, so we only repaint a tile when its number actually changes
// (a full repaint every cycle would flicker and waste SPI bandwidth).
static uint16_t last_hr = 0xFFFF, last_spo2 = 0xFFFF, last_rr = 0xFFFF;
static int16_t  last_temp = INT16_MIN;
static uint8_t  last_batt = 0xFF;

// ---- drawing helpers --------------------------------------------------------

// One vitals tile: a label above a large value. `valid` greys out a reading the
// DSP has not yet qualified (no finger on the PPG, no lead contact, ...).
static void drawTile(int16_t x, int16_t w, const char *label,
                     const char *value, uint16_t colour, bool valid)
{
  gfx->fillRect(x, TILE_Y, w, TILE_H, COL_BG);

  gfx->setTextSize(1);
  gfx->setTextColor(COL_LABEL, COL_BG);
  gfx->setCursor(x + 4, TILE_Y + 4);
  gfx->print(label);

  gfx->setTextSize(3);
  gfx->setTextColor(valid ? colour : COL_STALE, COL_BG);
  gfx->setCursor(x + 4, TILE_Y + 26);
  gfx->print(value);
}

static void drawChrome()
{
  gfx->fillScreen(COL_BG);

  gfx->fillRect(0, 0, gfx->width(), HDR_H, COL_HDR);
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_WHITE, COL_HDR);
  gfx->setCursor(6, 6);
  gfx->print("HealthyPi 5");

  gfx->drawFastHLine(0, WAVE_TOP - 4, gfx->width(), RGB565_DARKGREY);
  gfx->setTextSize(1);
  gfx->setTextColor(COL_LABEL, COL_BG);
  gfx->setCursor(6, WAVE_TOP - 14);
  gfx->print("ECG  128 SPS");
}

// Repaint only the tiles whose value changed since the last call.
static void drawVitals()
{
  const hpi_vitals_t &v = HealthyPi5.vitals();
  const int16_t w = gfx->width() / 5;   // five tiles across
  char buf[8];

  if (v.hr != last_hr) {
    last_hr = v.hr;
    if (v.hr_valid && v.hr) snprintf(buf, sizeof(buf), "%u", v.hr);
    else                    snprintf(buf, sizeof(buf), "--");
    drawTile(0 * w, w, "HR bpm", buf, RGB565_RED, v.hr_valid && v.hr);
  }

  if (v.spo2 != last_spo2) {
    last_spo2 = v.spo2;
    if (v.spo2_valid && v.spo2) snprintf(buf, sizeof(buf), "%u", v.spo2);
    else                        snprintf(buf, sizeof(buf), "--");
    drawTile(1 * w, w, "SpO2 %", buf, RGB565_CYAN, v.spo2_valid && v.spo2);
  }

  if (v.resp_rate != last_rr) {
    last_rr = v.resp_rate;
    if (v.resp_valid && v.resp_rate) snprintf(buf, sizeof(buf), "%u", v.resp_rate);
    else                             snprintf(buf, sizeof(buf), "--");
    drawTile(2 * w, w, "RESP rpm", buf, RGB565_YELLOW, v.resp_valid && v.resp_rate);
  }

  // Temperature and battery come from the I2C poll, not the DSP. Both are
  // absent-sensor-safe: a board with no temp cable or no battery reports
  // "not present" rather than a bogus number.
  int16_t t = HealthyPi5.temperature_x100();
  if (t != last_temp) {
    last_temp = t;
    bool present = HealthyPi5.temperaturePresent();
    // Take the sign out before dividing: -50 (=-0.5 C) must not print as "0.5",
    // which is what plain integer division would give (it truncates toward zero).
    int32_t at = abs((int32_t)t);
    if (present) snprintf(buf, sizeof(buf), "%s%ld.%ld",
                          t < 0 ? "-" : "", at / 100, (at % 100) / 10);
    else         snprintf(buf, sizeof(buf), "--");
    drawTile(3 * w, w, "TEMP C", buf, RGB565_ORANGE, present);
  }

  uint8_t soc = HealthyPi5.batterySoc();
  if (soc != last_batt) {
    last_batt = soc;
    bool present = HealthyPi5.batteryPresent();
    if (present) snprintf(buf, sizeof(buf), "%u", soc);
    else         snprintf(buf, sizeof(buf), "--");
    drawTile(4 * w, w, "BATT %", buf, RGB565_GREENYELLOW, present);
  }
}

// Paint one ECG sample as one column, sweeping left to right and erasing a few
// columns ahead of the pen (the classic bedside-monitor sweep). We never redraw
// the whole trace area — at 24 MHz a full 480x320 18-bit repaint would take
// ~150 ms and could not keep up with 128 SPS.
//
// If you enable HealthyPi5.recordSD(), bracket this function's body with
// hpi_spi1_lock() / hpi_spi1_unlock() (from <HPISpi1.h>) so a card write can't
// interleave with a display transfer on the shared SPI1 bus.
static void drawWave(int32_t ecg_raw)
{
  float x = (float)ecg_raw;
  float y = HP_ALPHA * (hp_out + x - hp_in);   // 1-pole high-pass
  hp_in  = x;
  hp_out = y;

  int32_t px = ((int32_t)y * (WAVE_H / 2)) / ECG_FULLSCALE;
  if (px >  WAVE_H / 2 - 1) px =  WAVE_H / 2 - 1;   // clamp into the band
  if (px < -(WAVE_H / 2 - 1)) px = -(WAVE_H / 2 - 1);
  int16_t cur_y = WAVE_MID - (int16_t)px;

  // Erase an 8 px gap ahead of the pen so old trace is visibly consumed.
  for (int16_t i = 1; i <= 8; i++) {
    int16_t ex = (wave_x + i) % gfx->width();
    gfx->drawFastVLine(ex, WAVE_TOP, WAVE_H, COL_BG);
  }

  // Join to the previous sample so fast QRS edges stay continuous. `prev_y < 0`
  // means the pen just wrapped, so start a fresh stroke instead of drawing a
  // full-width line back across the screen.
  if (prev_y >= 0) gfx->drawLine(wave_x - 1, prev_y, wave_x, cur_y, COL_TRACE);
  else             gfx->drawPixel(wave_x, cur_y, COL_TRACE);

  prev_y = cur_y;
  if (++wave_x >= gfx->width()) { wave_x = 0; prev_y = -1; }
}

// ---- sketch -----------------------------------------------------------------
void setup()
{
  // Backlight is a plain PWM pin (GP16). Full brightness; lower for dimming.
  pinMode(HPI_PIN_LCD_BACKLIGHT, OUTPUT);
  analogWrite(HPI_PIN_LCD_BACKLIGHT, 255);

  // arduino-pico needs the SPI1 pins assigned before the bus is started.
  SPI1.setSCK(HPI_PIN_SPI1_SCK);
  SPI1.setTX(HPI_PIN_SPI1_MOSI);
  SPI1.setRX(HPI_PIN_SPI1_MISO);

  if (!gfx->begin(HPI_LCD_SPI_HZ)) {   // 24 MHz, from board.h
    Serial1.println("HPI_APP display init FAILED");
    // Carry on regardless: the runtime below is useful headless, and this way a
    // missing display never bricks acquisition.
  }
  drawChrome();

  // ---- the HealthyPi5 runtime. Register sinks BEFORE begin(). ----
  HealthyPi5.computeVitals();       // DSP sink -> HR / SpO2 / respiration
  HealthyPi5.enableSensors();       // I2C temperature + battery
  HealthyPi5.enableLoopStream(256); // raw samples -> loop(), for the ECG trace
  HealthyPi5.begin();               // ring + broker + telemetry + watchdog
}

void loop()
{
  // Drain every sample the broker has queued for us, then repaint the numbers.
  // Nothing here blocks acquisition: a full queue drops the newest sample.
  hpi_sample_t s;
  while (HealthyPi5.read(s)) drawWave(s.ecg);

  static uint32_t next_vitals = 0;
  if ((int32_t)(millis() - next_vitals) >= 0) {
    next_vitals = millis() + 500;   // the DSP updates at ~1 Hz; 2 Hz is plenty
    drawVitals();
  }
}
