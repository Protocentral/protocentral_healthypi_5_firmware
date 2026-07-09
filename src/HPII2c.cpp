/*
 * I2C sensors (A9) — see HPII2c.h.
 */
#ifdef __FREERTOS   /* part of the FreeRTOS-only runtime; empty TU otherwise */

#include "Protocentral_HealthyPi_5.h"
#include "HPII2c.h"
#include <Wire.h>

static bool s_temp_present;
static bool s_batt_present;

/* Which temperature sensor is on the QWIIC connector, detected at boot. Newer
 * kits ship the AS6221 (addr 0x48, 1/128 C/LSB) in place of the MAX30205
 * (addr 0x49, 1/256 C/LSB). Both expose the temperature in register 0x00 as a
 * signed 16-bit big-endian value; only the address and scaling differ, so we
 * store the detected address plus the per-LSB divisor (raw*100/div = temp x100). */
enum hpi_temp_kind { HPI_TEMP_NONE, HPI_TEMP_MAX30205, HPI_TEMP_AS6221 };
static hpi_temp_kind s_temp_kind = HPI_TEMP_NONE;
static uint8_t       s_temp_addr;
static int32_t       s_temp_lsb_div;

static bool i2c_probe(uint8_t addr)
{
  Wire1.beginTransmission(addr);
  return (Wire1.endTransmission() == 0);
}

const char *hpi_temp_sensor_name(void)
{
  switch (s_temp_kind) {
    case HPI_TEMP_MAX30205: return "MAX30205";
    case HPI_TEMP_AS6221:   return "AS6221";
    default:                return "none";
  }
}

void hpi_i2c_init(void)
{
  Wire1.setSDA(HPI_PIN_I2C1_SDA);
  Wire1.setSCL(HPI_PIN_I2C1_SCL);
  Wire1.setClock(HPI_I2C1_HZ);
  Wire1.begin();

  /* Temperature: probe the MAX30205 first, then the AS6221 (QWIIC). */
  if (i2c_probe(HPI_ADDR_MAX30205)) {
    s_temp_kind = HPI_TEMP_MAX30205; s_temp_addr = HPI_ADDR_MAX30205; s_temp_lsb_div = 256;
  } else if (i2c_probe(HPI_ADDR_AS6221)) {
    s_temp_kind = HPI_TEMP_AS6221;   s_temp_addr = HPI_ADDR_AS6221;   s_temp_lsb_div = 128;
  } else {
    s_temp_kind = HPI_TEMP_NONE;
  }
  s_temp_present = (s_temp_kind != HPI_TEMP_NONE);

  s_batt_present = i2c_probe(HPI_ADDR_MAX17048);
}

/* Read a 16-bit big-endian register; false on any bus error. */
static bool read_reg16(uint8_t addr, uint8_t reg, uint16_t *out)
{
  Wire1.beginTransmission(addr);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(addr, (uint8_t)2) < 2) return false;
  uint8_t hi = Wire1.read();
  uint8_t lo = Wire1.read();
  *out = ((uint16_t)hi << 8) | lo;
  return true;
}

void hpi_i2c_poll(void)
{
  if (s_temp_present) {
    uint16_t raw;
    if (read_reg16(s_temp_addr, HPI_TEMP_REG, &raw)) {
      /* Signed 16-bit; scale by the detected sensor's per-LSB resolution. */
      int16_t t_x100 = (int16_t)(((int32_t)(int16_t)raw * 100) / s_temp_lsb_div);
      HealthyPi5.setTemperature(t_x100, true);
    }
  }

  if (s_batt_present) {
    uint16_t soc_raw, vcell_raw;
    if (read_reg16(HPI_ADDR_MAX17048, 0x04, &soc_raw) &&    /* SOC  */
        read_reg16(HPI_ADDR_MAX17048, 0x02, &vcell_raw)) {  /* VCELL */
      uint8_t  soc = (uint8_t)(soc_raw >> 8);               /* integer %        */
      uint16_t mv  = (uint16_t)(((uint32_t)vcell_raw * 78125u) / 1000000u); /* 78.125 uV/LSB */
      HealthyPi5.setBattery(soc, mv, true);
    }
  }
}

#endif /* __FREERTOS */
