/*
 * HealthyPi 5 NEXT — full dual-core parity firmware.
 * ============================================================================
 * The ADVANCED example: the production HealthyPi 5 NEXT dual-core AMP
 * architecture (Protocentral/healthypi5_next_rp2040), realised on arduino-pico
 * and encapsulated in the HealthyPi5 library. The sketch just declares which
 * features to run; the library owns both cores and every task.
 *
 *   Board    : "Raspberry Pi Pico"  (arduino-pico)  — Tools > os: "FreeRTOS SMP"
 *   Sensors  : MAX30001 (ECG/BioZ/HR), AFE4400 (PPG/SpO2), MAX30205, MAX17048
 *   Output   : USB-CDC (Serial)  -> OpenView 2 binary stream + command plane
 *              UART0  (Serial1)  -> 1 Hz HPI_INSTR telemetry / HPI_FAULT
 *              UART1  (Serial2)  -> HealthyBridge link to the ESP32-C3
 *   Libraries: HealthyPi5 (this repo), protocentral_max30001,
 *              protocentral_afe44xx (+ bundled SpO2 algorithm)
 *
 * WHAT THE LIBRARY DOES (see the Protocentral_HealthyPi_5 library source):
 *   - core1 acquires at 128 SPS into a lock-free 512-deep ring; no FreeRTOS API.
 *   - core0 runs a broker that fans each sample out to one drop-newest queue per
 *     sink (OpenView, DSP/vitals, SD, HealthyBridge), plus the command plane,
 *     1 Hz telemetry and a hardware watchdog. Every core0 task is pinned to
 *     core0 so nothing steals acquisition cycles. A slow/absent sink only drops
 *     its own samples (counted) — it can never back-pressure core1.
 *
 * Because the library defines setup1()/loop1(), this sketch must NOT.
 * To run your own DSP on the raw samples in loop(), see ../../Tutorials/09_RawProcessing
 * (HealthyPi5.enableLoopStream() + read()).
 */
#include <FreeRTOS.h>        // selects arduino-pico's FreeRTOS-SMP variant
#include <Protocentral_HealthyPi_5.h>

void setup()
{
  HealthyPi5.computeVitals();      // DSP sink: HR + SpO2 + respiration (A5)
  HealthyPi5.streamOpenView();     // OpenView 2 over USB-CDC (carries the vitals)
  HealthyPi5.enableCommands();     // host control plane on USB RX (A6)
  HealthyPi5.persistConfig();      // device name + auto-stream saved to flash (A7)
  HealthyPi5.recordSD();           // REC*.BIN to a FAT SD card on SPI1 (A8)
  HealthyPi5.enableSensors();      // I2C temp (MAX30205) + battery (MAX17048) (A9)
  HealthyPi5.enableBridge();       // HealthyBridge link to the ESP32-C3 (A9)
  HealthyPi5.begin();              // ring + broker + telemetry + watchdog
}

void loop()
{
  // Everything runs in the library's pinned core0 tasks. Watch the single 1 Hz
  // HPI_INSTR line on UART0 (Serial1) for health/telemetry.
}
