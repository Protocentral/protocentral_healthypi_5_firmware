/*
 * LVGL v9 configuration for the HealthyPi 5 display example.
 * ============================================================================
 * This file is NOT compiled from here. LVGL resolves its config as
 * "../../lv_conf.h" relative to lvgl/src/, i.e. it must sit NEXT TO the lvgl
 * library folder — a copy inside the sketch folder is not found. So install it
 * with:
 *
 *     cp extras/lv_conf.h "$(arduino-cli config get directories.user)/libraries/"
 *
 * extras/scripts/display-test.sh and the display CI job both do this for you.
 *
 * Minimal-override style: only the macros this UI depends on are set and
 * lv_conf_internal.h supplies every other default.
 *
 * A single task (the display task) owns LVGL, so no RTOS lock is needed:
 * LV_USE_OS stays at its default (none), ticks come from lv_tick_set_cb, and
 * that task calls lv_timer_handler itself.
 */
#ifndef LV_CONF_H
#define LV_CONF_H

/* 16-bit render buffer (RGB565); the panel flush converts/streams it. */
#define LV_COLOR_DEPTH 16

/* LVGL's builtin allocator over a fixed pool. 24 KB proved too tight for the
 * 480x320 UI (montserrat_48 + cards + button): lv_malloc failed mid-build_ui
 * and, with asserts/log off, the task hung on the NULL -> blank panel. 64 KB is
 * comfortable and the RP2040 has 264 KB of RAM. */
#define LV_USE_STDLIB_MALLOC  LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_STRING  LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_SPRINTF LV_STDLIB_BUILTIN
#define LV_MEM_SIZE (64U * 1024U)

/* Logging + malloc assert ON so an out-of-memory (or any LVGL error) is
 * reported instead of silently hanging. */
#define LV_USE_LOG 1
#define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
#define LV_USE_ASSERT_NULL    1
#define LV_USE_ASSERT_MALLOC  1
#define LV_USE_DEMO_WIDGETS   0

/* Fonts: small caption + big numerics for the vitals. */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_48 1
#define LV_FONT_DEFAULT &lv_font_montserrat_14

#endif /* LV_CONF_H */
