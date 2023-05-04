#include "src/lvgl/lvgl.h"
#include <TFT_eSPI.h>

#include "healthypi_display.h"
#include "hpi_defines.h"

static const uint16_t screenWidth = 480;
static const uint16_t screenHeight = 320;

// SPI TFT driver
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);

// LVGL GUI Objects
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];
lv_indev_t *indev_keypad;

// GUI Charts
static lv_obj_t *chart1;
static lv_obj_t *chart2;
static lv_obj_t *chart3;
static lv_chart_series_t *ser1;
static lv_chart_series_t *ser2;
static lv_chart_series_t *ser3;

// GUI Labels
lv_obj_t *label_hr;
lv_obj_t *label_spo2;
lv_obj_t *label_rr;
lv_obj_t *label_temp;

lv_obj_t *label_co2_voc;

// LVGL GUI Screens
lv_obj_t *scr_main_menu;
lv_obj_t *scr_charts_all;

lv_obj_t *scr_charts_ecg;
lv_obj_t *scr_charts_ppg;
lv_obj_t *scr_charts_resp;

enum hpi_scr_t hpi_current_screen = SCR_MAIN_MENU;

int pos = 0;

int pointCounter = 0;

float y1_max = 0;
float y1_min = 10000;

float y2_max = 0;
float y2_min = 10000;

float y3_max = 0;
float y3_min = 10000;

static float gx = 0;

#define SAMPLE_RATE 125

bool chart1_update = false;
bool chart2_update = false;
bool chart3_update = false;

void HealthyPi_Display::init()
{
    Serial.println("Initializing display");

    tft.begin();

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

    tft.setRotation(3);

    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // lv_obj_set_style_bg_color(lv_scr_act(), LV_COLOR_MAKE(0, 0, 0), LV_STATE_DEFAULT);
    //  lv_obj_set_style_text_color(lv_scr_act(),LV_COLOR_MAKE(255, 255, 255), LV_STATE_DEFAULT);
    lv_indev_init();

    // draw_main_menu();
    draw_scr_charts_all();
    // draw_scr_charts_ppg_only();
    // draw_scr_charts_ecg_only();
    //    lv_scr_load(scr_all_charts);
    //    lv_scr_load(scr_main_menu);
    //  get_screen(SCR_CHARTS_ALL);
    //  get_screen(SCR_CHARTS_PPG);

    get_screen(SCR_CHARTS_ALL);
}

void HealthyPi_Display::do_set_scale()
{
    if (gx >= DISP_WINDOW_SIZE)
    {
        if (chart1_update == true)
            // lv_chart_set_range(chart1, LV_CHART_AXIS_PRIMARY_Y, y1_min, y1_max);
            if (chart2_update == true)
                // lv_chart_set_range(chart2, LV_CHART_AXIS_PRIMARY_Y, y2_min, y2_max);
                if (chart3_update == true)
                    // lv_chart_set_range(chart3, LV_CHART_AXIS_PRIMARY_Y, y3_min, y3_max);

                    gx = 0;
        y1_max = -100000;
        y1_min = 100000;

        y2_max = -100000;
        y2_min = 100000;

        y3_max = -100000;
        y3_min = 100000;
    }
}

void HealthyPi_Display::draw_plotECG(float *data_ecg, int num_samples)
{
    if (chart1_update == true)
    {
        for (int i = 0; i < num_samples; i++)
        {
            if (data_ecg[i] < y1_min)
            {
                y1_min = data_ecg[i];
            }

            if (data_ecg[i] > y1_max)
            {
                y1_max = data_ecg[i];
            }

            lv_chart_set_next_value(chart1, ser1, data_ecg[i]);
        }
    }
}

void HealthyPi_Display::draw_plotppg(float data_ppg)
{
    if (chart2_update == true)
    {
        if (data_ppg < y2_min)
        {
            y2_min = data_ppg;
        }

        if (data_ppg > y2_max)
        {
            y2_max = data_ppg;
        }
        lv_chart_set_next_value(chart2, ser2, data_ppg);
    }
}

void HealthyPi_Display::draw_plotresp(float *data_resp, int num_samples)
{
    if (chart3_update == true)
    {
        for (int i = 0; i < num_samples; i++)
        {
            if (data_resp[i] < y3_min)
            {
                y3_min = data_resp[i];
            }

            if (data_resp[i] > y3_max)
            {
                y3_max = data_resp[i];
            }

            lv_chart_set_next_value(chart3, ser3, data_resp[i]);
        }
    }
}

void HealthyPi_Display::updateHR(uint8_t hr)
{
  if (chart1_update == true)
  {
    lv_label_set_text_fmt(label_hr, "%d", hr);
  }
}

void HealthyPi_Display::updateSpO2(uint8_t spo2, bool spo2_ok)
{
  if (chart2_update == true)
  {
    if (spo2_ok == true)
    {
      lv_label_set_text_fmt(label_spo2, "%d", spo2);
    }
    else
    {
      lv_label_set_text_fmt(label_spo2, "--");
    }
  }
}

void HealthyPi_Display::updateRR(uint8_t rr)
{
  if (chart3_update == true)
  {
    lv_label_set_text_fmt(label_rr, "%d\n", rr);
  }
}

void HealthyPi_Display::updateTemp(float temp)
{
  char temp_str[10];
  sprintf(temp_str, "%.1f", temp);
  lv_label_set_text_fmt(label_temp, temp_str, temp);
}

void HealthyPi_Display::updateEnv(int co2, int voc)
{
  char co2_str[14];
  sprintf(co2_str, "CO2: %d VOC: %d", co2, voc);
  lv_label_set_text(label_co2_voc, co2_str);
}

/* LVGL Common Header */
void draw_header(lv_obj_t *parent)
{
    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_bg_color(&style, lv_color_black());
    lv_style_set_text_color(&style, lv_color_white());
    lv_style_set_border_width(&style, 0);
    lv_style_set_pad_all(&style, 0);
    lv_obj_add_style(parent, &style, 0);

    // Draw Header bar
    // ProtoCentral logo
    LV_IMG_DECLARE(logo_oneline);
    lv_obj_t *img1 = lv_img_create(parent);
    lv_img_set_src(img1, &logo_oneline);
    lv_obj_align(img1, LV_ALIGN_TOP_LEFT, 10, 7);
    lv_obj_set_size(img1, 104, 10);

    // HealthyPi label
    lv_obj_t *label_hpi = lv_label_create(parent);
    lv_label_set_text(label_hpi, "HealthyPi 5");
    lv_obj_align(label_hpi, LV_ALIGN_TOP_MID, 0, 5);

    // Label for Symbols
    lv_obj_t *label_symbols = lv_label_create(parent);
    lv_label_set_text(label_symbols, LV_SYMBOL_BATTERY_FULL " " LV_SYMBOL_BLUETOOTH);
    lv_obj_align(label_symbols, LV_ALIGN_TOP_RIGHT, -5, 5);
}

void draw_footer(lv_obj_t *parent)
{
    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_bg_color(&style, lv_color_black());
    lv_style_set_text_color(&style, lv_color_white());
    lv_style_set_border_width(&style, 0);
    lv_style_set_pad_all(&style, 0);
    lv_obj_add_style(parent, &style, 0);

    // Draw Header bar
    // ProtoCentral logo
    LV_IMG_DECLARE(logo_oneline);
    lv_obj_t *img1 = lv_img_create(parent);
    lv_img_set_src(img1, &logo_oneline);
    lv_obj_align(img1, LV_ALIGN_BOTTOM_MID, 10, -10);
    lv_obj_set_size(img1, 104, 10);

    // Label for Symbols
    lv_obj_t *label_symbols = lv_label_create(parent);
    lv_label_set_text(label_symbols, LV_SYMBOL_BATTERY_FULL " " LV_SYMBOL_BLUETOOTH);
    lv_obj_align(label_symbols, LV_ALIGN_BOTTOM_LEFT, 5, -10);

    // Label for CO2 / VOC
    label_co2_voc = lv_label_create(parent);
    lv_label_set_text(label_co2_voc, "CO2: 400 VOC: 0");
    lv_obj_align(label_co2_voc, LV_ALIGN_BOTTOM_LEFT, 45, -10);
}

void get_screen(enum hpi_scr_t get_scr)
{
    switch (get_scr)
    {
    case SCR_MAIN_MENU:
        hpi_current_screen = SCR_MAIN_MENU;
        lv_scr_load(scr_main_menu);
        break;
    case SCR_CHARTS_ALL:
        chart1_update = true;
        chart2_update = true;
        chart3_update = true;
        hpi_current_screen = SCR_CHARTS_ALL;
        lv_scr_load(scr_charts_all);
        break;
    case SCR_CHARTS_ECG:
        chart1_update = true;
        chart2_update = false;
        chart3_update = false;
        hpi_current_screen = SCR_CHARTS_ECG;
        lv_scr_load(scr_charts_ecg);

        break;
    case SCR_CHARTS_PPG:
        chart1_update = false;
        chart2_update = true;
        chart3_update = false;
        hpi_current_screen = SCR_CHARTS_PPG;
        lv_scr_load(scr_charts_ppg);

        break;
    default:
        break;
    }
}

void menu_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_VALUE_CHANGED)
    {
        pos = lv_roller_get_selected(obj);

        Serial.print("Pos changed to:");
        Serial.println(pos);
    }
    if (code == LV_EVENT_PRESSED)
    {
        Serial.println("Pressed");
        if (hpi_current_screen == SCR_CHARTS_ALL)
        {
            Serial.println("Scr charts all");
            get_screen(SCR_MAIN_MENU);
        }
        else if (hpi_current_screen == SCR_MAIN_MENU)
        {
            Serial.println("Scr main menu");
            if (pos == 0)
            {
                get_screen(SCR_CHARTS_ALL);
            }
            if (pos == 1)
            {
                get_screen(SCR_CHARTS_PPG);
            }
            else if (pos == 2)
            {
                get_screen(SCR_CHARTS_ECG);
            }
            else if (pos == 3)
            {
                get_screen(SCR_CHARTS_RESP);
            }
        }
        else if (hpi_current_screen == SCR_CHARTS_PPG)
        {
            Serial.println("Scr charts ppg");
            get_screen(SCR_MAIN_MENU);
        }
        else if (hpi_current_screen == SCR_CHARTS_ECG)
        {
            Serial.println("Scr charts ecg");
            get_screen(SCR_MAIN_MENU);
        }
        else if (hpi_current_screen == SCR_CHARTS_RESP)
        {
            Serial.println("Scr charts resp");
            get_screen(SCR_MAIN_MENU);
        }

        // lv_scr_load(scr_clock);
    }
}

static void btn1_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_VALUE_CHANGED)
    {

        Serial.println("Selected value:");

        // lv_scr_load(scr_clock);
    }
}

/* LVGL Screens */

// Draw all charts screen
void HealthyPi_Display::draw_scr_charts_all(void)
{

    static lv_style_t style;
    lv_style_init(&style);
    // lv_style_set_radius(&style, 5);

    /*Make a gradient*/
    lv_style_set_bg_opa(&style, LV_OPA_COVER);
    lv_style_set_border_width(&style, 0);
    static lv_grad_dsc_t grad;
    grad.dir = LV_GRAD_DIR_HOR;
    grad.stops_count = 2;
    grad.stops[0].color = lv_color_hex(0x003a57); // lv_palette_lighten(LV_PALETTE_GREY, 1);
    grad.stops[1].color = lv_color_black();       // lv_palette_main(LV_PALETTE_BLUE);

    /*Shift the gradient to the bottom*/
    grad.stops[0].frac = 128;
    grad.stops[1].frac = 192;

    lv_style_set_bg_grad(&style, &grad);

    scr_charts_all = lv_obj_create(NULL);

    lv_obj_add_style(scr_charts_all, &style, 0);

    // lv_obj_set_style_bg_color(scr_charts_all, lv_color_hex(0x134f5c), LV_STATE_DEFAULT);

    lv_group_t *g1 = lv_group_create();

    // Create Chart 1
    chart1 = lv_chart_create(scr_charts_all);
    lv_obj_set_size(chart1, 380, 90);
    lv_obj_set_style_bg_color(chart1, LV_COLOR_MAKE(0, 0, 0), LV_STATE_DEFAULT);

    lv_obj_set_style_size(chart1, 0, LV_PART_INDICATOR);
    lv_chart_set_point_count(chart1, DISP_WINDOW_SIZE);
    // lv_chart_set_type(chart1, LV_CHART_TYPE_LINE);   /*Show lines and points too*
    lv_chart_set_range(chart1, LV_CHART_AXIS_PRIMARY_Y, -200, 200);
    // lv_chart_set_range(chart1, LV_CHART_AXIS_SECONDARY_Y, 0, 1000);
    lv_chart_set_div_line_count(chart1, 0, 0);
    lv_chart_set_update_mode(chart1, LV_CHART_UPDATE_MODE_CIRCULAR);

    lv_obj_set_pos(chart1, 10, 4);

    // Create Chart 2
    chart2 = lv_chart_create(scr_charts_all);
    lv_obj_set_size(chart2, 380, 90);
    lv_obj_set_style_bg_color(chart2, LV_COLOR_MAKE(0, 0, 0), LV_STATE_DEFAULT);

    lv_obj_set_style_size(chart2, 0, LV_PART_INDICATOR);
    lv_chart_set_point_count(chart2, DISP_WINDOW_SIZE);
    // lv_chart_set_type(chart1, LV_CHART_TYPE_LINE);   /*Show lines and points too*
    lv_chart_set_range(chart2, LV_CHART_AXIS_PRIMARY_Y, -1000, 1000);
    // lv_chart_set_range(chart1, LV_CHART_AXIS_SECONDARY_Y, 0, 1000);
    lv_chart_set_div_line_count(chart2, 0, 0);
    lv_chart_set_update_mode(chart2, LV_CHART_UPDATE_MODE_CIRCULAR);

    lv_obj_align_to(chart2, chart1, LV_ALIGN_OUT_BOTTOM_MID, 0, 2);

    // Create Chart 3
    chart3 = lv_chart_create(scr_charts_all);
    lv_obj_set_size(chart3, 380, 90);
    lv_obj_set_style_bg_color(chart3, LV_COLOR_MAKE(0, 0, 0), LV_STATE_DEFAULT);

    lv_obj_set_style_size(chart3, 0, LV_PART_INDICATOR);
    lv_chart_set_point_count(chart3, DISP_WINDOW_SIZE);
    // lv_chart_set_type(chart1, LV_CHART_TYPE_LINE);   /*Show lines and points too*
    lv_chart_set_range(chart3, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    // lv_chart_set_range(chart1, LV_CHART_AXIS_SECONDARY_Y, 0, 1000);
    lv_chart_set_div_line_count(chart3, 0, 0);
    lv_chart_set_update_mode(chart3, LV_CHART_UPDATE_MODE_CIRCULAR);

    lv_obj_align_to(chart3, chart2, LV_ALIGN_OUT_BOTTOM_MID, 0, 2);

    /* Data Series for 3 plots*/
    ser1 = lv_chart_add_series(chart1, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    ser2 = lv_chart_add_series(chart2, lv_palette_main(LV_PALETTE_YELLOW), LV_CHART_AXIS_PRIMARY_Y);
    ser3 = lv_chart_add_series(chart3, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);

    /*
    // Button to switch to other screens
    lv_obj_t *btn1 = lv_btn_create(scr_charts_all);
    lv_obj_add_event_cb(btn1, btn1_cb, LV_EVENT_ALL, NULL);
    lv_obj_align_to(btn1, chart3, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_obj_t *label = lv_label_create(btn1);
    lv_label_set_text(label, "Push button for other charts");
    lv_obj_center(label);
    */

    // Subscript (Unit) label style
    static lv_style_t style_sub;
    lv_style_init(&style_sub);
    lv_style_set_text_color(&style_sub, lv_color_white());
    lv_style_set_text_font(&style_sub, &lv_font_montserrat_12);

    // HR Number label style
    static lv_style_t style_hr;
    lv_style_init(&style_hr);
    lv_style_set_text_color(&style_hr, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_text_font(&style_hr, &lv_font_montserrat_42);

    // HR Number label
    label_hr = lv_label_create(scr_charts_all);
    lv_label_set_text(label_hr, "--");
    lv_obj_align_to(label_hr, chart1, LV_ALIGN_OUT_RIGHT_MID, 8, -25);
    lv_obj_add_style(label_hr, &style_hr, LV_STATE_DEFAULT);

    // HR Title label
    static lv_obj_t *label_hr_title = lv_label_create(scr_charts_all);
    lv_label_set_text(label_hr_title, "HR");
    lv_obj_align_to(label_hr_title, label_hr, LV_ALIGN_TOP_MID, 0, -10);
    lv_obj_add_style(label_hr_title, &style_sub, LV_STATE_DEFAULT);

    // HR BPM Subscript label
    static lv_obj_t *label_hr_sub = lv_label_create(scr_charts_all);
    lv_label_set_text(label_hr_sub, "bpm");
    lv_obj_align_to(label_hr_sub, label_hr, LV_ALIGN_BOTTOM_MID, 0, 10);
    lv_obj_add_style(label_hr_sub, &style_sub, LV_STATE_DEFAULT);

    // SpO2 label style
    static lv_style_t style_spo2;
    lv_style_init(&style_spo2);
    lv_style_set_text_color(&style_spo2, lv_palette_main(LV_PALETTE_YELLOW));
    lv_style_set_text_font(&style_spo2, &lv_font_montserrat_42);

    // SPO2 Number label
    label_spo2 = lv_label_create(scr_charts_all);
    lv_label_set_text(label_spo2, "--");
    lv_obj_align_to(label_spo2, chart2, LV_ALIGN_OUT_RIGHT_MID, 8, -25);
    lv_obj_add_style(label_spo2, &style_spo2, LV_STATE_DEFAULT);

    // SpO2 Title label
    static lv_obj_t *label_spo2_title = lv_label_create(scr_charts_all);
    lv_label_set_text(label_spo2_title, "SpO2");
    lv_obj_align_to(label_spo2_title, label_spo2, LV_ALIGN_TOP_MID, 0, -10);
    lv_obj_add_style(label_spo2_title, &style_sub, LV_STATE_DEFAULT);

    // SpO2 % label
    static lv_obj_t *label_spo2_sub = lv_label_create(scr_charts_all);
    lv_label_set_text(label_spo2_sub, "%");
    lv_obj_align_to(label_spo2_sub, label_spo2, LV_ALIGN_BOTTOM_MID, 0, 10);
    lv_obj_add_style(label_spo2_sub, &style_sub, LV_STATE_DEFAULT);

    // RR label style
    static lv_style_t style_rr;
    lv_style_init(&style_rr);
    lv_style_set_text_color(&style_rr, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_text_font(&style_rr, &lv_font_montserrat_42);

    // RR Number label
    label_rr = lv_label_create(scr_charts_all);
    lv_label_set_text(label_rr, "--");
    lv_obj_align_to(label_rr, chart3, LV_ALIGN_OUT_RIGHT_MID, 8, -30);
    lv_obj_add_style(label_rr, &style_rr, LV_STATE_DEFAULT);

    // RR Sub BPM label
    static lv_obj_t *label_rr_title = lv_label_create(scr_charts_all);
    lv_label_set_text(label_rr_title, "Resp Rate");
    lv_obj_align_to(label_rr_title, label_rr, LV_ALIGN_TOP_MID, 0, -10);
    lv_obj_add_style(label_rr_title, &style_sub, LV_STATE_DEFAULT);

    // RR Sub BPM label
    static lv_obj_t *label_rr_sub = lv_label_create(scr_charts_all);
    lv_label_set_text(label_rr_sub, "bpm");
    lv_obj_align_to(label_rr_sub, label_rr, LV_ALIGN_BOTTOM_MID, 0, 10);
    lv_obj_add_style(label_rr_sub, &style_sub, LV_STATE_DEFAULT);

    // Temp label
    lv_obj_t *label_temp_title = lv_label_create(scr_charts_all);
    lv_label_set_text(label_temp_title, "Temp.");
    lv_obj_align_to(label_temp_title, label_rr_sub, LV_ALIGN_BOTTOM_MID, 5, 25);
    lv_obj_add_style(label_temp_title, &style_sub, LV_STATE_DEFAULT);

    // SpO2 label style
    static lv_style_t style_temp;
    lv_style_init(&style_temp);
    lv_style_set_text_color(&style_temp, lv_palette_main(LV_PALETTE_LIME));
    lv_style_set_text_font(&style_temp, &lv_font_montserrat_34);

    // Temp Number label
    label_temp = lv_label_create(scr_charts_all);
    lv_label_set_text(label_temp, "--");
    lv_obj_align_to(label_temp, label_temp_title, LV_ALIGN_BOTTOM_MID, -20, 10);
    lv_obj_add_style(label_temp, &style_temp, LV_STATE_DEFAULT);

    // Temp Sub deg C label
    static lv_obj_t *label_temp_sub = lv_label_create(scr_charts_all);
    lv_label_set_text(label_temp_sub, "°C");
    lv_obj_align_to(label_temp_sub, label_temp, LV_ALIGN_BOTTOM_MID, 0, 10);
    lv_obj_add_style(label_temp_sub, &style_sub, LV_STATE_DEFAULT);

    // draw_header(scr_charts_all);
    draw_footer(scr_charts_all);

    // lv_group_add_obj(g1, btn1);
    // lv_indev_set_group(indev_keypad, g1);
}

// Draw the main menu
void HealthyPi_Display::draw_main_menu(void)
{
    // Create main screen called scr_main_menu
    scr_main_menu = lv_obj_create(NULL);
    // lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x003a57), LV_PART_MAIN);

    // lv_obj_set_style_bg_color(scr_main_menu, lv_color_hex(0x003a57), LV_STATE_DEFAULT);

    lv_group_t *g = lv_group_create();

    draw_header(scr_main_menu);

    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_bg_color(&style, lv_color_black());
    lv_style_set_text_color(&style, lv_color_white());
    lv_style_set_border_width(&style, 0);
    lv_style_set_pad_all(&style, 0);
    lv_obj_add_style(lv_scr_act(), &style, 0);

    /*A style to make the selected option larger*/
    static lv_style_t style_sel;
    lv_style_init(&style_sel);

    // lv_style_set_text_font(&style_sel, &lv_font_montserrat_22);

    lv_obj_t *roller1 = lv_roller_create(scr_main_menu);
    lv_roller_set_options(roller1,
                          "Show All Charts (default)\n"
                          "Show ECG\n"
                          "Show PPG\n"
                          "Show Respiration",
                          LV_ROLLER_MODE_INFINITE);
    // lv_obj_add_style(roller1, &style_sel, LV_PART_SELECTED);
    lv_obj_add_style(roller1, &style, 0);
    lv_obj_add_event_cb(roller1, menu_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(roller1, LV_ALIGN_CENTER, 0, 0);

    lv_group_add_obj(g, roller1);
    lv_indev_set_group(indev_keypad, g);
}

// Draw ECG only screen
void draw_scr_charts_ecg_only(void)
{
    scr_charts_ecg = lv_obj_create(NULL);

    lv_group_t *g1 = lv_group_create();

    // Create Chart 1
    chart1 = lv_chart_create(scr_charts_ecg);
    lv_obj_set_size(chart1, 480, 90 * 3);
    lv_obj_set_style_bg_color(chart1, LV_COLOR_MAKE(0, 0, 0), LV_STATE_DEFAULT);

    lv_obj_set_style_size(chart1, 0, LV_PART_INDICATOR);
    lv_chart_set_point_count(chart1, DISP_WINDOW_SIZE);
    // lv_chart_set_type(chart1, LV_CHART_TYPE_LINE);   /*Show lines and points too*
    lv_chart_set_range(chart1, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    // lv_chart_set_range(chart1, LV_CHART_AXIS_SECONDARY_Y, 0, 1000);
    lv_chart_set_div_line_count(chart1, 0, 0);
    lv_chart_set_update_mode(chart1, LV_CHART_UPDATE_MODE_CIRCULAR);

    lv_obj_set_pos(chart1, 0, 30);

    /* Data Series for 3 plots*/
    ser1 = lv_chart_add_series(chart1, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);

    // HR label style
    static lv_style_t style_hr;
    lv_style_init(&style_hr);
    lv_style_set_text_color(&style_hr, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_text_font(&style_hr, &lv_font_montserrat_28);

    // HR label
    label_hr = lv_label_create(scr_charts_ecg);
    lv_label_set_text(label_hr, "--");
    lv_obj_align_to(label_hr, chart1, LV_ALIGN_TOP_RIGHT, -30, 0);
    lv_obj_add_style(label_hr, &style_hr, LV_STATE_DEFAULT);

    draw_header(scr_charts_ecg);
}

// Display flushing callback
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

/*Get the currently being pressed key.  0 if no key is pressed*/
static uint32_t keypad_get_key(void)
{
    /*Your code comes here*/
    if (digitalRead(BUTTON_DOWN) == LOW)
    {
        // Serial.println("Down");
        return (uint32_t)1;
    }
    else if (digitalRead(BUTTON_UP) == LOW)
    {
        return (uint32_t)2;
    }
    else if (digitalRead(BUTTON_CENTER) == LOW)
    {
        return (uint32_t)3;
    }
    return 0;
}

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/*Will be called by the library to read*/
static void keypad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static uint32_t last_key = 0;

    /*Get whether the a key is pressed and save the pressed key*/
    uint32_t act_key = keypad_get_key();
    if (act_key != 0)
    {
        data->state = LV_INDEV_STATE_PR;

        /*Translate the keys to LVGL control characters according to your key definitions*/
        switch (act_key)
        {
        case 1:
            act_key = LV_KEY_DOWN;
            break;
        case 2:
            act_key = LV_KEY_UP;
            break;
        case 3:
            act_key = LV_KEY_ENTER;
            break;
        case 4:
            act_key = LV_KEY_ENTER;
            break;
        }

        last_key = act_key;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }

    data->key = last_key;
}

// Draw PPG only screen
void draw_scr_charts_ppg_only()
{
    scr_charts_ppg = lv_obj_create(NULL);

    lv_group_t *g1 = lv_group_create();

    // Create Chart 1
    chart2 = lv_chart_create(scr_charts_ppg);
    lv_obj_set_size(chart2, 480, 90 * 3);
    lv_obj_set_style_bg_color(chart2, LV_COLOR_MAKE(0, 0, 0), LV_STATE_DEFAULT);

    lv_obj_set_style_size(chart2, 0, LV_PART_INDICATOR);
    lv_chart_set_point_count(chart2, DISP_WINDOW_SIZE);
    // lv_chart_set_type(chart1, LV_CHART_TYPE_LINE);   /*Show lines and points too*
    lv_chart_set_range(chart2, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    // lv_chart_set_range(chart1, LV_CHART_AXIS_SECONDARY_Y, 0, 1000);
    lv_chart_set_div_line_count(chart2, 0, 0);
    lv_chart_set_update_mode(chart2, LV_CHART_UPDATE_MODE_CIRCULAR);

    lv_obj_set_pos(chart2, 0, 30);

    /* Data Series for 3 plots*/
    ser2 = lv_chart_add_series(chart2, lv_palette_main(LV_PALETTE_YELLOW), LV_CHART_AXIS_PRIMARY_Y);

    // SpO2 label style
    static lv_style_t style_spo2;
    lv_style_init(&style_spo2);
    lv_style_set_text_color(&style_spo2, lv_palette_main(LV_PALETTE_YELLOW));
    lv_style_set_text_font(&style_spo2, &lv_font_montserrat_28);

    // SPO2 label
    label_spo2 = lv_label_create(scr_charts_ppg);
    lv_label_set_text(label_spo2, "--");
    lv_obj_align_to(label_spo2, chart2, LV_ALIGN_TOP_RIGHT, -40, -5);
    lv_obj_add_style(label_spo2, &style_spo2, LV_STATE_DEFAULT);

    draw_header(scr_charts_ppg);
}

void lv_indev_init(void)
{
    static lv_indev_drv_t indev_drv;

    pinMode(BUTTON_DOWN, INPUT);
    pinMode(BUTTON_UP, INPUT);
    pinMode(BUTTON_CENTER, INPUT);

    /*Register a keypad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_KEYPAD;
    indev_drv.read_cb = keypad_read;
    indev_keypad = lv_indev_drv_register(&indev_drv);
}
