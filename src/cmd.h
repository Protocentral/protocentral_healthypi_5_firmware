/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 ProtoCentral Electronics — HealthyPi 5 NEXT firmware. */
/*
 * Host command / control plane (P6) — bounded `AA 55` parser.
 *
 * Wire format (host → device), byte-identical to app_gf/src/main.c:
 *   AA 55 | type | len | payload[len] | xor-checksum(payload)
 *
 * Strictly bounded by construction: a len > CMD_MAX_PAYLOAD is rejected before
 * any byte is stored, and the payload write is double-guarded by the index
 * check — a peer-controlled length can never write past the fixed buffer.
 *
 * One parser instance per source (one source for now: USB; P9 adds a second fed
 * from HealthyBridge HOST_CMD frames). Each instance carries a response sink so
 * a PING ack is routed back to the source it came from.
 */
#ifndef HPI_CMD_H
#define HPI_CMD_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CMD_MAX_PAYLOAD 32
#define CMD_NAME_LEN    32

/* Response sink: dispatch calls this to send bytes back toward the host. */
typedef void (*cmd_resp_fn)(const uint8_t *buf, size_t len);

typedef struct {
    uint8_t     state;
    uint8_t     type, len, idx, chk;
    uint8_t     payload[CMD_MAX_PAYLOAD];
    cmd_resp_fn respond;
} cmd_parser_t;

/* Reset control state to defaults (streaming + auto-stream on). Call once at boot. */
void cmd_init(void);

void cmd_parser_init(cmd_parser_t *p, cmd_resp_fn respond);
void cmd_parse_byte(cmd_parser_t *p, uint8_t b);

/* Control state, read by the USB sink. */
bool        cmd_streaming_enabled(void);
bool        cmd_auto_stream(void);
const char *cmd_device_name(void);
uint32_t    cmd_count(void);              /* commands successfully dispatched */

/* Persisted settings (P7). The setters apply a value WITHOUT marking dirty — used
 * by config load at boot. A SET_NAME/SET_AUTOSTREAM command applies via these AND
 * marks dirty, so the config module flushes the change to flash. */
void cmd_set_name(const char *name);
void cmd_set_auto_stream(bool en);
bool cmd_config_dirty(void);
void cmd_config_clear_dirty(void);

/* SD-recording hooks for REC_START/REC_STOP. Weak no-ops here; the SD sink (A8)
 * provides strong definitions when it lands. */
void hpi_rec_start(void);
void hpi_rec_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* HPI_CMD_H */
