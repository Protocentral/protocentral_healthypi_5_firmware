/*
 * HealthyPi 5 NEXT (Arduino) — I2C sensors (A9): temperature + battery.
 * ============================================================================
 * Body temperature (MAX30205 @0x49 on older kits, or AS6221 @0x48 on newer
 * QWIIC kits — auto-detected at boot) and MAX17048 fuel gauge (0x36) on Wire1
 * (I2C1, GP6/7). Absent-sensor-safe: each device is probed once at init and
 * only polled if it ACKs, so a board without a temp cable or battery just
 * reports "not present" instead of stalling the bus. Values are pushed into the
 * HealthyPi5 store (temperature_x100 / battery*), where OpenView and the
 * HealthyBridge vitals/battery frames pick them up.
 */
#ifndef HPI_I2C_H
#define HPI_I2C_H

void        hpi_i2c_init(void);          /* Wire1 begin + probe/detect devices */
void        hpi_i2c_poll(void);          /* read present sensors (~1 Hz)        */
const char *hpi_temp_sensor_name(void);  /* "MAX30205" | "AS6221" | "none"      */

#endif /* HPI_I2C_H */
