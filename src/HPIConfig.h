/*
 * HealthyPi 5 NEXT (Arduino) — persistent config (A7).
 * ============================================================================
 * device_name + auto_stream persisted in the last flash sector, mirroring the
 * NEXT hpi_config.c blob (magic + version + checksum). Uses arduino-pico's
 * EEPROM library, whose commit() is multicore-safe under FreeRTOS: it parks
 * core1 via rp2040.idleOtherCore() before the flash erase/program, so a config
 * write can't corrupt or crash acquisition (it does pause it for ~50-100 ms).
 *
 * The persisted values back the cmd control state (cmd.h): load overrides the
 * cmd_init() defaults at boot; a SET_NAME/SET_AUTOSTREAM command marks
 * cmd_config_dirty(), which the flush writes back.
 */
#ifndef HPI_CONFIG_H
#define HPI_CONFIG_H

/* Load the persisted blob and apply it to the cmd state. Call once at boot,
 * AFTER cmd_init(). Falls back to the cmd defaults if no valid blob is present. */
void hpi_config_init(void);

/* If a persisted setting changed (cmd_config_dirty()), write it to flash and
 * clear the dirty flag. Call from a low-priority periodic context (the instr
 * task) so the flash write — which briefly parks core1 — is off the hot path. */
void hpi_config_flush_if_dirty(void);

#endif /* HPI_CONFIG_H */
