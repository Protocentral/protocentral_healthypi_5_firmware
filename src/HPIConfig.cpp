/*
 * Persistent config (A7) — see HPIConfig.h. Blob format mirrors the NEXT
 * hpi_config.c so the on-flash layout is recognisable, but storage goes through
 * arduino-pico's EEPROM (last flash sector, multicore-safe commit).
 */
#ifdef __FREERTOS   /* part of the FreeRTOS-only runtime; empty TU otherwise */

#include "HPIConfig.h"
#include "cmd.h"

#include <EEPROM.h>
#include <string.h>
#include <stddef.h>

#define HPI_CONFIG_MAGIC   0x48503543u   /* 'H''P''5''C' */
#define HPI_CONFIG_VERSION 1u

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  char     name[CMD_NAME_LEN];   /* 32 */
  uint8_t  auto_stream;
  uint8_t  pad[3];
  uint32_t checksum;             /* additive sum of all preceding bytes */
} hpi_config_blob_t;

static uint32_t blob_checksum(const hpi_config_blob_t *b)
{
  const uint8_t *p = (const uint8_t *)b;
  uint32_t sum = 0;
  for (size_t i = 0; i < offsetof(hpi_config_blob_t, checksum); i++) sum += p[i];
  return sum;
}

void hpi_config_init(void)
{
  EEPROM.begin(256);                 /* maps the last flash sector into RAM */

  hpi_config_blob_t b;
  EEPROM.get(0, b);

  if (b.magic == HPI_CONFIG_MAGIC &&
      b.version == HPI_CONFIG_VERSION &&
      b.checksum == blob_checksum(&b)) {
    char name[CMD_NAME_LEN];
    memcpy(name, b.name, CMD_NAME_LEN);
    name[CMD_NAME_LEN - 1] = '\0';
    cmd_set_name(name);
    cmd_set_auto_stream(b.auto_stream != 0);
  }
  /* else: no valid blob — keep the cmd_init() defaults. */
}

void hpi_config_flush_if_dirty(void)
{
  if (!cmd_config_dirty()) return;

  hpi_config_blob_t b;
  memset(&b, 0, sizeof(b));
  b.magic   = HPI_CONFIG_MAGIC;
  b.version = HPI_CONFIG_VERSION;
  strncpy(b.name, cmd_device_name(), CMD_NAME_LEN - 1);
  b.auto_stream = cmd_auto_stream() ? 1 : 0;
  b.checksum = blob_checksum(&b);

  EEPROM.put(0, b);
  EEPROM.commit();                   /* parks core1, erases+programs the sector */
  cmd_config_clear_dirty();
}

#endif /* __FREERTOS */
