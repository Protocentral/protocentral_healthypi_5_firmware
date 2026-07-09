/*
 * SD recording sink (A8) — see HPISd.h. Ported from the NEXT sd_logger.c; the
 * file-static state and the on-disk format are preserved, with FatFS swapped
 * for arduino-pico's SDFS (SPI1) and the sd_task loop folded into consume().
 */
#ifdef __FREERTOS   /* part of the FreeRTOS-only runtime; empty TU otherwise */

#include "HPISd.h"
#include "cmd.h"
#include "HPISpi1.h"

#include <SDFS.h>
#include <string.h>

#define SD_WRITE_BUF   4096      /* batch writes to whole flash pages (256 recs)  */
#define SD_SYNC_MS     2000      /* flush FAT this often to bound power-loss loss  */

/* File header (32 bytes) — byte-identical to decode_rec.py. */
#define SD_MAGIC    0x35504824u  /* '$HP5' LE */
#define SD_VERSION  1
typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t rec_size;       /* sizeof(hpi_sd_rec_t) = 16 */
  uint16_t sample_rate_hz; /* ~128 */
  uint16_t channels;       /* 4: ecg, bioz, ppg_red, ppg_ir */
  uint32_t start_uptime_ms;
  uint8_t  reserved[16];
} sd_file_hdr_t;

/* Control -> sink one-shot request (set from the A6 hooks below). */
enum { SD_REQ_NONE = 0, SD_REQ_START, SD_REQ_STOP };
static volatile int s_req;

/* Owned by the SD sink task only. */
static File     s_file;
static uint8_t  s_wbuf[SD_WRITE_BUF];
static size_t   s_wlen;
static bool     s_mounted;
static uint32_t s_last_sync;

/* Published to readers. */
static volatile bool     s_card;
static volatile bool     s_recording;
static volatile uint32_t s_drops;
static volatile uint32_t s_rec_start_ms;

bool     hpi_sd_recording(void)    { return s_recording; }
bool     hpi_sd_card_present(void) { return s_card; }
uint32_t hpi_sd_drops(void)        { return s_drops; }
uint32_t hpi_sd_record_secs(void)
{
  return s_recording ? (millis() - s_rec_start_ms) / 1000u : 0;
}

/* A6 REC_START/REC_STOP land here (override the weak no-ops in cmd.c). */
extern "C" void hpi_rec_start(void) { s_req = SD_REQ_START; }
extern "C" void hpi_rec_stop(void)  { s_req = SD_REQ_STOP; }

static bool sd_mount(void)
{
  if (s_mounted) return true;

  SPI1.setSCK(HPI_PIN_SPI1_SCK);
  SPI1.setTX(HPI_PIN_SPI1_MOSI);
  SPI1.setRX(HPI_PIN_SPI1_MISO);

  SDFSConfig cfg(HPI_PIN_SD_CS, HPI_SD_SPI_HZ, SPI1);
  SDFS.setConfig(cfg);
  if (!SDFS.begin()) {
    s_card = false;
    return false;
  }
  s_card = s_mounted = true;
  return true;
}

static void flush_buf(void)
{
  if (s_wlen == 0) return;
  size_t bw = s_file.write(s_wbuf, s_wlen);
  if (bw != s_wlen) s_drops++;   /* short write -> card error, counted */
  s_wlen = 0;
}

/* All SD I/O is bracketed by hpi_spi1_lock()/unlock() — the card shares SPI1
 * with the LCD (HPISpi1). */
static void do_start(void)
{
  if (s_recording) return;

  hpi_spi1_lock();
  if (!sd_mount()) { hpi_spi1_unlock(); return; }

  char path[20];
  for (int i = 1; i <= 99999; i++) {
    snprintf(path, sizeof(path), "/REC%05d.BIN", i);
    if (!SDFS.exists(path)) break;
    if (i == 99999) { hpi_spi1_unlock(); return; }   /* card full of REC files */
  }

  s_file = SDFS.open(path, "w");
  if (!s_file) { hpi_spi1_unlock(); return; }

  sd_file_hdr_t hdr;
  memset(&hdr, 0, sizeof(hdr));
  hdr.magic          = SD_MAGIC;
  hdr.version        = SD_VERSION;
  hdr.rec_size       = sizeof(hpi_sd_rec_t);
  hdr.sample_rate_hz = HPI_ECG_SPS;
  hdr.channels       = 4;
  hdr.start_uptime_ms = millis();
  s_file.write((const uint8_t *)&hdr, sizeof(hdr));
  hpi_spi1_unlock();

  s_wlen         = 0;
  s_last_sync    = millis();
  s_rec_start_ms = millis();
  s_recording    = true;
}

static void do_stop(void)
{
  if (!s_recording) return;
  s_recording = false;
  hpi_spi1_lock();
  flush_buf();
  s_file.flush();
  s_file.close();
  hpi_spi1_unlock();
}

void SdSink::begin()
{
  /* Deliberately do NOT mount here: a missing/slow card makes SdFat retry for
   * seconds, which would lengthen first-boot bring-up. The card is mounted
   * lazily on the first REC_START (do_start -> sd_mount). */
}

void SdSink::consume(const hpi_sample_t &s)
{
  /* Service a one-shot REC_START/STOP request (even with no card / not recording). */
  int req = s_req;
  if (req != SD_REQ_NONE) {
    s_req = SD_REQ_NONE;
    if (req == SD_REQ_START) do_start();
    else                     do_stop();
  }

  if (!s_recording) return;

  hpi_sd_rec_t rec = { s.ecg, s.bioz, s.ppg_red, s.ppg_ir };
  memcpy(&s_wbuf[s_wlen], &rec, sizeof(rec));
  s_wlen += sizeof(rec);
  if (s_wlen >= SD_WRITE_BUF) {
    hpi_spi1_lock();
    flush_buf();
    hpi_spi1_unlock();
  }

  uint32_t now = millis();
  if ((now - s_last_sync) >= SD_SYNC_MS) {
    s_last_sync = now;
    hpi_spi1_lock();
    flush_buf();
    s_file.flush();
    hpi_spi1_unlock();
  }
}

#endif /* __FREERTOS */
