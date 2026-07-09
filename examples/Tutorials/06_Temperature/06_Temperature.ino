/*
 * HealthyPi 5 — Tutorial 06: Body temperature (MAX30205 or AS6221, I2C)
 * ============================================================================
 * Read body temperature from the digital temperature sensor on the QWIIC
 * connector and print it in degrees Celsius. Two sensors ship across kits — the
 * MAX30205 on older kits and the AS6221 on newer ones — so this example also
 * shows "which sensor is here?" auto-detection AND "absent-sensor-safe" design:
 * we probe the bus once at boot, pick whichever answers, and simply report if
 * neither is connected — instead of hanging or printing junk.
 *
 *   Board    : "Raspberry Pi Pico"  (arduino-pico core)
 *   Sensor   : MAX30205 (addr 0x49) OR AS6221 (addr 0x48), on I2C1 (Wire1/QWIIC)
 *   Library  : none beyond the core Wire library
 *   Also needs: the in-repo HealthyPi5 library, for board.h
 *   Output   : Arduino IDE  ->  Tools > Serial Monitor  @ 115200 baud
 *
 * Both sensors put the temperature in register 0x00 as a signed 16-bit
 * big-endian value; only the I2C address and the per-LSB resolution differ:
 *   MAX30205: 1 LSB = 1/256 C (0.00390625 C)
 *   AS6221:   1 LSB = 1/128 C (0.0078125 C)
 * So Celsius = raw * lsb, where we set `lsb` from whichever sensor we found.
 */
#include <Wire.h>
#include <board.h>

uint8_t     temp_addr = 0;              // detected I2C address (0 = none found)
float       temp_lsb  = 0.0f;           // degrees C per LSB for that sensor
const char *temp_name = "none";

// Probe the two known temperature sensors on the QWIIC bus; first to ACK wins.
bool detect_temp_sensor() {
  Wire1.beginTransmission(HPI_ADDR_MAX30205);
  if (Wire1.endTransmission() == 0) {
    temp_addr = HPI_ADDR_MAX30205; temp_lsb = 1.0f / 256; temp_name = "MAX30205";
    return true;
  }
  Wire1.beginTransmission(HPI_ADDR_AS6221);
  if (Wire1.endTransmission() == 0) {
    temp_addr = HPI_ADDR_AS6221;   temp_lsb = 1.0f / 128; temp_name = "AS6221";
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);

  // ---- I2C1 (Wire1) — the HealthyPi 5's sensor / QWIIC I2C bus ----
  Wire1.setSDA(HPI_PIN_I2C1_SDA);
  Wire1.setSCL(HPI_PIN_I2C1_SCL);
  Wire1.setClock(HPI_I2C1_HZ);
  Wire1.begin();

  if (detect_temp_sensor()) {
    Serial.print(temp_name);
    Serial.println(" found — reading temperature once per second...");
  } else {
    Serial.println("No temperature sensor found on the QWIIC connector "
                   "(MAX30205 @0x49 / AS6221 @0x48). Connect one and reset.");
  }
}

void loop() {
  if (!temp_addr) { delay(1000); return; }

  static uint32_t last_ms = 0;
  if (millis() - last_ms < 1000) return;   // temperature changes slowly
  last_ms = millis();

  // Point at the temperature register, then read 2 bytes (big-endian).
  Wire1.beginTransmission(temp_addr);
  Wire1.write(HPI_TEMP_REG);               // 0x00 on both sensors
  if (Wire1.endTransmission(false) != 0) return;          // repeated-start read
  if (Wire1.requestFrom(temp_addr, (uint8_t)2) < 2) return;

  int16_t raw = ((int16_t)Wire1.read() << 8) | Wire1.read();
  float celsius = raw * temp_lsb;          // scale by the detected sensor's LSB

  Serial.print(temp_name);
  Serial.print("  Temp: ");
  Serial.print(celsius, 2);
  Serial.println(" C");
}
