#include <FreeRTOS.h>
#include <task.h>
#include <lvgl.h>
#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <Protocentral_HealthyPi_5.h>

#define DISP_W 480
#define DISP_H 320

Arduino_DataBus *bus = nullptr;
Arduino_GFX     *gfx  = nullptr;

static void init_gfx_bus()
{
  
  /* SPI1 is shared with the SD card, so EVERY touch of the peripheral — the
   * pin mux and SPI1.begin() included, not just transfers — has to happen
   * under the arbitration lock. SPI1.begin() calls beginTransaction(), which
   * can spi_deinit()/spi_init() the block out from under an in-flight SD
   * transaction if it runs unlocked. */
  HealthyPi5.hpiSpi1Lock();
  SPI1.setSCK(HPI_PIN_SPI1_SCK);
  SPI1.setTX(HPI_PIN_SPI1_MOSI);   // MOSI
  SPI1.setRX(HPI_PIN_SPI1_MISO);
  SPI1.begin();
 /* Explicit hardware reset, timed to match the known-working reference
   * driver. Do this ourselves rather than relying on Arduino_GFX's internal
   * reset pulse inside gfx->begin() — that pulse isn't guaranteed to fully
   * resync the controller on a warm restart. */
  pinMode(HPI_PIN_LCD_RST, OUTPUT);
  digitalWrite(HPI_PIN_LCD_RST, HIGH); delay(5);
  digitalWrite(HPI_PIN_LCD_RST, LOW);  delay(20);
  digitalWrite(HPI_PIN_LCD_RST, HIGH); delay(150);
  HealthyPi5.hpiSpi1Unlock();

  bus = new Arduino_HWSPI(HPI_PIN_LCD_DC, HPI_PIN_LCD_CS, &SPI1, true);

#ifdef HPI_DISPLAY_ST7796
  gfx = new Arduino_ST7796(bus, HPI_PIN_LCD_RST, 1);
#else
  gfx = new Arduino_ILI9488_18bit(bus, HPI_PIN_LCD_RST, 1);
#endif
}

static uint8_t s_draw_buf1[DISP_W * 12 * 2];
static uint8_t s_draw_buf2[DISP_W * 12 * 2];

#ifndef HPI_BL_PWM_HZ
#define HPI_BL_PWM_HZ   2000
#endif
#ifndef HPI_BL_PWM_DUTY
#define HPI_BL_PWM_DUTY 240
#endif

static void lcd_backlight(bool on)
{
  analogWriteFreq(HPI_BL_PWM_HZ);
  analogWriteRange(255);
  analogWrite(HPI_PIN_LCD_BACKLIGHT, on ? HPI_BL_PWM_DUTY : 0);
}

static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
  int32_t w = area->x2 - area->x1 + 1;
  int32_t h = area->y2 - area->y1 + 1;
  HealthyPi5.hpiSpi1Lock();
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
  HealthyPi5.hpiSpi1Unlock();
  lv_display_flush_ready(disp);
}

static uint32_t tick_cb(void) { return millis(); }

/* ---- UI (unchanged from the original build_ui/update_ui) ---- */
#define ACCENT_HR    lv_color_hex(0xEF5350)
#define ACCENT_SPO2  lv_color_hex(0x26C6DA)
#define ACCENT_RR    lv_color_hex(0xFFCA28)
#define ACCENT_TEMP  lv_color_hex(0xFFA726)

static lv_obj_t  *s_val[4];
static lv_group_t *s_group;
static lv_style_t s_focus_style;
static lv_obj_t  *s_rec_btn, *s_rec_btn_lbl, *s_rec_chip, *s_rec_chip_lbl;
static lv_obj_t  *s_sd_chip, *s_sd_chip_lbl;
static void refresh_status_ui(void);

static void make_card(int idx, int col, int row, const char *title, lv_color_t accent)
{
  lv_obj_t *card = lv_obj_create(lv_screen_active());
  lv_obj_set_size(card, 222, 96);
  lv_obj_set_pos(card, 12 + col * 234, 52 + row * 104);
  lv_obj_set_style_bg_color(card, lv_color_hex(0x1C222A), 0);
  lv_obj_set_style_border_width(card, 0, 0);
  lv_obj_set_style_radius(card, 10, 0);
  lv_obj_set_style_pad_all(card, 8, 0);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *lbl = lv_label_create(card);
  lv_label_set_text(lbl, title);
  lv_obj_set_style_text_color(lbl, accent, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t *val = lv_label_create(card);
  lv_label_set_text(val, "--");
  lv_obj_set_style_text_color(val, lv_color_white(), 0);
  lv_obj_set_style_text_font(val, &lv_font_montserrat_48, 0);
  lv_obj_set_width(val, 190);
  lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(val, LV_ALIGN_CENTER, 0, 6);
  s_val[idx] = val;
}

static void rec_btn_long_press_cb(lv_event_t *e)
{
  (void)e;
  if (HealthyPi5.recording()) HealthyPi5.recordStop();
  else                        HealthyPi5.recordStart();
}

static void build_ui(void)
{
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x0C1014), 0);

  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text(title, "HealthyPi 5");
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
  lv_obj_align(title, LV_ALIGN_TOP_RIGHT, -12, 8);

  lv_style_init(&s_focus_style);
  lv_style_set_border_color(&s_focus_style, lv_color_white());
  lv_style_set_border_width(&s_focus_style, 3);

  make_card(0, 0, 0, "HR  bpm",  ACCENT_HR);
  make_card(1, 1, 0, "SpO2  %",  ACCENT_SPO2);
  make_card(2, 0, 1, "RESP rpm", ACCENT_RR);
  make_card(3, 1, 1, "TEMP  C",  ACCENT_TEMP);

  s_rec_chip = lv_obj_create(scr);
  lv_obj_set_size(s_rec_chip, 96, 32);
  lv_obj_align(s_rec_chip, LV_ALIGN_TOP_LEFT, 12, 8);
  lv_obj_set_style_radius(s_rec_chip, 16, 0);
  lv_obj_set_style_border_width(s_rec_chip, 0, 0);
  lv_obj_set_style_pad_all(s_rec_chip, 0, 0);
  lv_obj_clear_flag(s_rec_chip, LV_OBJ_FLAG_SCROLLABLE);
  s_rec_chip_lbl = lv_label_create(s_rec_chip);
  lv_obj_set_style_text_font(s_rec_chip_lbl, &lv_font_montserrat_14, 0);
  lv_obj_center(s_rec_chip_lbl);

  s_sd_chip = lv_obj_create(scr);
  lv_obj_set_size(s_sd_chip, 96, 32);
  lv_obj_align(s_sd_chip, LV_ALIGN_TOP_LEFT, 116, 8);
  lv_obj_set_style_radius(s_sd_chip, 16, 0);
  lv_obj_set_style_border_width(s_sd_chip, 0, 0);
  lv_obj_set_style_pad_all(s_sd_chip, 0, 0);
  lv_obj_clear_flag(s_sd_chip, LV_OBJ_FLAG_SCROLLABLE);
  s_sd_chip_lbl = lv_label_create(s_sd_chip);
  lv_obj_set_style_text_font(s_sd_chip_lbl, &lv_font_montserrat_14, 0);
  lv_obj_center(s_sd_chip_lbl);

  s_rec_btn = lv_button_create(scr);
  lv_obj_set_size(s_rec_btn, 456, 54);
  lv_obj_set_pos(s_rec_btn, 12, 258);
  lv_obj_set_style_radius(s_rec_btn, 10, 0);
  lv_obj_set_style_border_width(s_rec_btn, 0, 0);
  lv_obj_set_style_pad_all(s_rec_btn, 6, 0);
  lv_obj_add_style(s_rec_btn, &s_focus_style, LV_STATE_FOCUSED);
  lv_obj_add_event_cb(s_rec_btn, rec_btn_long_press_cb, LV_EVENT_LONG_PRESSED, NULL);
  s_rec_btn_lbl = lv_label_create(s_rec_btn);
  lv_obj_set_style_text_font(s_rec_btn_lbl, &lv_font_montserrat_14, 0);
  lv_obj_center(s_rec_btn_lbl);
  lv_group_add_obj(s_group, s_rec_btn);

  refresh_status_ui();
}

static void keys_init(void)
{
  pinMode(HPI_PIN_KEY_UP,   INPUT_PULLUP);
  pinMode(HPI_PIN_KEY_DOWN, INPUT_PULLUP);
  pinMode(HPI_PIN_KEY_OK,   INPUT_PULLUP);
}

static void encoder_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  (void)indev;
  bool up   = (digitalRead(HPI_PIN_KEY_UP)   == LOW);
  bool down = (digitalRead(HPI_PIN_KEY_DOWN) == LOW);
  bool ok   = (digitalRead(HPI_PIN_KEY_OK)   == LOW);

  static bool up_p, down_p;
  int16_t diff = 0;
  if (up   && !up_p)   diff = -1;
  if (down && !down_p) diff = +1;
  up_p = up; down_p = down;

  data->enc_diff = diff;
  data->state = ok ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

static void set_val(int idx, int32_t val, bool is_temp)
{
  static int32_t last[4] = { INT32_MAX, INT32_MAX, INT32_MAX, INT32_MAX };
  if (val == last[idx]) return;
  last[idx] = val;

  if (val == INT32_MIN) {
    lv_label_set_text(s_val[idx], "--");
  } else if (is_temp) {
    /* val is temperature x100. Split sign off first: a plain "%d.%d" on a
     * negative value renders -1.5 C as "-1.-5". */
    int32_t t = (val < 0) ? -val : val;
    lv_label_set_text_fmt(s_val[idx], "%s%d.%d", (val < 0) ? "-" : "",
                          (int)(t / 100), (int)((t / 10) % 10));
  } else {
    lv_label_set_text_fmt(s_val[idx], "%d", (int)val);
  }
}

static int s_status_shown = -1;   // 0=idle, 1=recording, 2=no-sd

static void refresh_status_ui(void)
{
  bool card = HealthyPi5.sdCardPresent();
  bool rec  = HealthyPi5.recording();
  int state = !card ? 2 : (rec ? 1 : 0);
  if (state == s_status_shown) return;
  s_status_shown = state;

  if (!card) {
    lv_label_set_text(s_sd_chip_lbl, "NO SD");
    lv_obj_set_style_bg_color(s_sd_chip, ACCENT_TEMP, 0);
    lv_obj_set_style_text_color(s_sd_chip_lbl, lv_color_white(), 0);

    lv_label_set_text(s_rec_chip_lbl, "IDLE");
    lv_obj_set_style_bg_color(s_rec_chip, lv_color_hex(0x2A2F37), 0);
    lv_obj_set_style_text_color(s_rec_chip_lbl, lv_color_hex(0x9AA0A6), 0);

    lv_label_set_text(s_rec_btn_lbl, "No SD card - insert, then hold OK");
    lv_obj_set_style_bg_color(s_rec_btn, lv_color_hex(0x7A5A12), 0);
  } else if (rec) {
    lv_label_set_text(s_sd_chip_lbl, "SD OK");
    lv_obj_set_style_bg_color(s_sd_chip, lv_color_hex(0x2A2F37), 0);
    lv_obj_set_style_text_color(s_sd_chip_lbl, lv_color_hex(0x9AA0A6), 0);

    lv_label_set_text(s_rec_chip_lbl, "REC");
    lv_obj_set_style_bg_color(s_rec_chip, ACCENT_HR, 0);
    lv_obj_set_style_text_color(s_rec_chip_lbl, lv_color_white(), 0);

    lv_label_set_text(s_rec_btn_lbl, "Hold OK to STOP");
    lv_obj_set_style_bg_color(s_rec_btn, lv_color_hex(0x7F1D1D), 0);
  } else {
    lv_label_set_text(s_sd_chip_lbl, "SD OK");
    lv_obj_set_style_bg_color(s_sd_chip, lv_color_hex(0x2A2F37), 0);
    lv_obj_set_style_text_color(s_sd_chip_lbl, lv_color_hex(0x9AA0A6), 0);

    lv_label_set_text(s_rec_chip_lbl, "IDLE");
    lv_obj_set_style_bg_color(s_rec_chip, lv_color_hex(0x2A2F37), 0);
    lv_obj_set_style_text_color(s_rec_chip_lbl, lv_color_hex(0x9AA0A6), 0);

    lv_label_set_text(s_rec_btn_lbl, "Hold OK to RECORD");
    lv_obj_set_style_bg_color(s_rec_btn, lv_color_hex(0x1C222A), 0);
  }
}
static void update_ui(void)
{
  const hpi_vitals_t &v = HealthyPi5.vitals();
  set_val(0, v.hr_valid   ? (int32_t)v.hr        : INT32_MIN, false);
  set_val(1, v.spo2_valid ? (int32_t)v.spo2      : INT32_MIN, false);
  set_val(2, v.resp_valid ? (int32_t)v.resp_rate : INT32_MIN, false);
  set_val(3, HealthyPi5.temperaturePresent() ? (int32_t)HealthyPi5.temperature_x100()
                                             : INT32_MIN, true);
}

/* ---- display task ---- */
void display_task(void *arg)
{
  (void)arg;

  init_gfx_bus();

  pinMode(HPI_PIN_LCD_BACKLIGHT, OUTPUT);
  digitalWrite(HPI_PIN_LCD_BACKLIGHT, LOW);   // keep backlight OFF until GRAM is cleared
  Serial1.printf("HPI_DISP start (backlight off, clearing GRAM)\r\n");

  HealthyPi5.hpiSpi1Lock();
  /* Arduino_GFX picks the bus data mode itself, and on this platform
   * Arduino_HWSPI::begin() defaults to SPI_MODE2 (CPOL=1) — see the
   * `#elif defined(SPI_HAS_TRANSACTION)` branch. Arduino_TFT only forwards a
   * mode when the driver sets _override_datamode, which Arduino_ILI9488_18bit
   * never does. Both the ILI9488 and the ST7796 want MODE0, so bring the bus
   * up explicitly here and pass GFX_SKIP_DATABUS_BEGIN so Arduino_TFT::begin()
   * does not re-run the databus begin and reset the mode back to MODE2. */
  bus->begin(HPI_LCD_SPI_HZ, SPI_MODE0);
  gfx->begin(GFX_SKIP_DATABUS_BEGIN);
  gfx->invertDisplay(true);
  gfx->fillScreen(0x000000);
  HealthyPi5.hpiSpi1Unlock();
  lcd_backlight(true);
  Serial1.printf("HPI_DISP panel init done, backlight on\r\n");

  lv_init();
  lv_tick_set_cb(tick_cb);
  Serial1.printf("HPI_DISP lvgl up\r\n");

  lv_display_t *disp = lv_display_create(DISP_W, DISP_H);
  lv_display_set_flush_cb(disp, flush_cb);
  lv_display_set_buffers(disp, s_draw_buf1, s_draw_buf2, sizeof(s_draw_buf1),
                       LV_DISPLAY_RENDER_MODE_PARTIAL);
  Serial1.printf("HPI_DISP buffers set\r\n");

  s_group = lv_group_create();
  build_ui();
  keys_init();
  Serial1.printf("HPI_DISP ui ready\r\n");

  lv_indev_t *enc = lv_indev_create();
  lv_indev_set_type(enc, LV_INDEV_TYPE_ENCODER);
  lv_indev_set_read_cb(enc, encoder_read_cb);
  lv_indev_set_group(enc, s_group);
  lv_indev_set_long_press_time(enc, 700);

  uint32_t last_update = 0;
  for (;;) {
    refresh_status_ui();

    uint32_t now = tick_cb();
    if (now - last_update >= 1000) {
      last_update = now;
      update_ui();
    }

    lv_timer_handler();

    /* ~5 Hz is ample for 1 Hz vitals and keeps the SPI1 lock free for the SD
     * sink; the encoder is polled from lv_timer_handler at the same rate. */
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void setup()
{
  HealthyPi5.computeVitals();
  HealthyPi5.streamOpenView();
  HealthyPi5.enableCommands();
  HealthyPi5.persistConfig();
  HealthyPi5.recordSD();
  HealthyPi5.enableSensors();
  HealthyPi5.enableBridge();
  HealthyPi5.begin();

  TaskHandle_t h;
  xTaskCreate(display_task, "disp", 8192, nullptr,
              (configMAX_PRIORITIES / 2) + 1, &h);
#if (defined(configUSE_CORE_AFFINITY) && (configUSE_CORE_AFFINITY == 1))
  vTaskCoreAffinitySet(h, (UBaseType_t)(1u << 0));
#endif
}

void loop() {}
