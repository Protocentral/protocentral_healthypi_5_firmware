#ifndef __HPI_DISP_H__
#define __HPI_DISP_H__

#include "Arduino.h"
#include "hpi_defines.h"

void lv_indev_init(void);
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void menu_event_handler(lv_event_t *e);
void get_screen(enum hpi_scr_t get_scr);
void draw_header(lv_obj_t *parent);
void draw_footer(lv_obj_t *parent);

class HealthyPi_Display
{
private:
    float y1_max = 0;
    float y1_min = 10000;

    float y2_max = 0;
    float y2_min = 10000;

    float y3_max = 0;
    float y3_min = 10000;

    float gx = 0;

    void init_styles(void);

public:
    void init();
    void draw_plotECG(float *data_ecg, int num_samples);
    void draw_plotresp(float *data_resp, int num_samples);
    void draw_plotppg(float data_ppg);
    void draw_main_menu(void);
    void draw_scr_hrv(void);


    void draw_plotRRI(float data_rri);
    void draw_plot_poincare(float *data_ecg, int num_samples);

    void draw_scr_charts_all(void);

    void updateEnv(int co2, int voc);
    void updateTemp(float temp);
    void updateRR(uint8_t rr);
    void updateSpO2(uint8_t spo2, bool spo2_ok);
    void updateHR(uint8_t hr);

    void do_set_scale();
    void add_samples(int num_samples);
};

#endif // __HPI_DISP_H__