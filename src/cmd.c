/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 ProtoCentral Electronics — HealthyPi 5 NEXT firmware. */
/* Host command / control plane (P6). See cmd.h. Ported from the GF cmd parser. */
#include "cmd.h"

#include <string.h>

#define CMD_SYNC1        0xAA
#define CMD_SYNC2        0x55
#define HPI_FRAME_ACK    0x81

/* SD-recording hooks — weak no-ops until the SD sink (A8) overrides them. */
__attribute__((weak)) void hpi_rec_start(void) { }
__attribute__((weak)) void hpi_rec_stop(void)  { }

/* Command types (host → device). */
#define CMD_PING          0x01
#define CMD_STREAM_START  0x10
#define CMD_STREAM_STOP   0x11
#define CMD_SET_NAME      0x12   /* payload = ASCII device name */
#define CMD_SET_AUTOSTREAM 0x13  /* payload[0] = 0/1 (auto-stream on connect) */
#define CMD_REC_START     0x14   /* start SD recording (P8) */
#define CMD_REC_STOP      0x15   /* stop SD recording  (P8) */

enum { S_SYNC1, S_SYNC2, S_TYPE, S_LEN, S_PAYLOAD, S_CHK };

/* ---- control state ---- */
static volatile bool s_streaming  = true;   /* gates the OpenView stream */
static volatile bool s_auto_stream = true;  /* auto-start streaming on connect */
static char          s_name[CMD_NAME_LEN] = "HealthyPi 5";
static volatile uint32_t s_cmds;
static volatile bool s_config_dirty;        /* a persisted setting changed (P7) */

void cmd_init(void)
{
    s_streaming    = true;
    s_auto_stream  = true;
    s_cmds         = 0;
    s_config_dirty = false;
    cmd_set_name("HealthyPi 5");   /* default until config load overrides */
}

bool        cmd_streaming_enabled(void) { return s_streaming; }
bool        cmd_auto_stream(void)       { return s_auto_stream; }
const char *cmd_device_name(void)       { return s_name; }
uint32_t    cmd_count(void)             { return s_cmds; }

void cmd_set_name(const char *name)
{
    size_t i = 0;
    for (; i < CMD_NAME_LEN - 1 && name[i]; i++) { s_name[i] = name[i]; }
    s_name[i] = '\0';
}
void cmd_set_auto_stream(bool en) { s_auto_stream = en; }
bool cmd_config_dirty(void)       { return s_config_dirty; }
void cmd_config_clear_dirty(void) { s_config_dirty = false; }

void cmd_parser_init(cmd_parser_t *p, cmd_resp_fn respond)
{
    memset(p, 0, sizeof(*p));
    p->state   = S_SYNC1;
    p->respond = respond;
}

static void cmd_dispatch(cmd_parser_t *p)
{
    s_cmds++;
    switch (p->type) {
    case CMD_PING:
        if (p->respond) {
            const uint8_t ack[5] = { CMD_SYNC1, CMD_SYNC2, HPI_FRAME_ACK, 0x00, 0x00 };
            p->respond(ack, sizeof(ack));
        }
        break;
    case CMD_STREAM_START:
        s_streaming = true;
        break;
    case CMD_STREAM_STOP:
        s_streaming = false;
        break;
    case CMD_SET_NAME: {
        char tmp[CMD_NAME_LEN];
        uint8_t nl = (p->len < CMD_NAME_LEN - 1) ? p->len : (uint8_t)(CMD_NAME_LEN - 1);
        memcpy(tmp, p->payload, nl);
        tmp[nl] = '\0';
        cmd_set_name(tmp);
        s_config_dirty = true;   /* config module flushes to flash (P7) */
        /* BLE_SET_NAME is forwarded to the ESP32-C3 by bridge_task. */
        break;
    }
    case CMD_SET_AUTOSTREAM:
        if (p->len >= 1) {
            cmd_set_auto_stream(p->payload[0] != 0);
            s_config_dirty = true;
        }
        break;
    case CMD_REC_START:
        hpi_rec_start();
        break;
    case CMD_REC_STOP:
        hpi_rec_stop();
        break;
    default:
        break;
    }
}

void cmd_parse_byte(cmd_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case S_SYNC1:
        p->state = (b == CMD_SYNC1) ? S_SYNC2 : S_SYNC1;
        break;
    case S_SYNC2:
        p->state = (b == CMD_SYNC2) ? S_TYPE : S_SYNC1;
        break;
    case S_TYPE:
        p->type = b;
        p->state = S_LEN;
        break;
    case S_LEN:
        p->len = b;
        if (p->len > CMD_MAX_PAYLOAD) { p->state = S_SYNC1; break; }  /* reject, no store */
        p->idx = 0;
        p->chk = 0;
        p->state = (p->len == 0) ? S_CHK : S_PAYLOAD;
        break;
    case S_PAYLOAD:
        if (p->idx < CMD_MAX_PAYLOAD) { p->payload[p->idx++] = b; p->chk ^= b; }
        if (p->idx >= p->len) { p->state = S_CHK; }
        break;
    case S_CHK:
        if (b == p->chk) { cmd_dispatch(p); }
        p->state = S_SYNC1;
        break;
    }
}
