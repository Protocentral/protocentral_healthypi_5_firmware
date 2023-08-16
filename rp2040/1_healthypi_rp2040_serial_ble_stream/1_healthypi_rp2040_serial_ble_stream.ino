//////////////////////////////////////////////////////////////////////////////////////////
//   Arduino program for HealthyPi 5 to Work in Streaming mode with Raspberry Pi RP2040
//
//   Copyright (c) 2022 ProtoCentral
//
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,  INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR   OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <Wire.h>

#include "protocentral_afe44xx.h"
#include "Protocentral_spo2_algorithm.h"

#include <protocentral_max30001.h>

#define CES_CMDIF_PKT_START_1 0x0A
#define CES_CMDIF_PKT_START_2 0xFA
#define CES_CMDIF_TYPE_DATA 0x02
#define CES_CMDIF_PKT_STOP_1 0x00
#define CES_CMDIF_PKT_STOP 0x0B

#define CES_CMDIF_DATA_LEN_LSB 20
#define CES_CMDIF_DATA_LEN_MSB 0

#define PPG_DATA 0X00

void send_data_serial_port(void);

uint8_t ppg_data_buff[20];
uint8_t lead_flag = 0x04;
uint8_t data_len = 20;
uint8_t spo2;
uint16_t ppg_stream_cnt = 1;
int16_t ppg_wave_ir;

bool spo2_calc_done = false;
bool ppg_buf_ready = false;
bool BioZSkipSample = false;

char DataPacket[30];

#define SPI_SCK_PIN 2
#define SPI_MOSI_PIN 3
#define SPI_MISO_PIN 4

#define MAX30001_CS_PIN 5

#define AFE44XX_CS_PIN 27
#define AFE44XX_DRDY_PIN 17
#define AFE44XX_PWDN_PIN 18

#define PIN_RP2040_TX_ESP32_RX 24
#define PIN_RP2040_RX_ESP32_TX 25

#define RP2040_ESP32_UART_BAUD 230400

#define ZERO 0
// volatile char DataPacket[DATA_LEN];
const char DataPacketHeader[] = { CES_CMDIF_PKT_START_1, CES_CMDIF_PKT_START_2, CES_CMDIF_DATA_LEN_LSB, CES_CMDIF_DATA_LEN_MSB, CES_CMDIF_TYPE_DATA };
const char DataPacketFooter[] = { ZERO, CES_CMDIF_PKT_STOP };

signed long ecg_data;
signed long bioz_data;

MAX30001 max30001(MAX30001_CS_PIN);
AFE44XX afe44xx(AFE44XX_CS_PIN, AFE44XX_PWDN_PIN);

spo2_algorithm spo2;
afe44xx_data afe44xx_raw_data;

const bool hpi_ble_enabled = true;

void send_data_serial_port(void) {

  for (int i = 0; i < 5; i++) {
    Serial.write(DataPacketHeader[i]);  // transmit the data over USB
    if (hpi_ble_enabled == true)
      Serial2.write(DataPacketHeader[i]);
  }

  for (int i = 0; i < 20; i++) {
    Serial.write(DataPacket[i]);  // transmit the data over USB
    if (hpi_ble_enabled == true)
      Serial2.write(DataPacket[i]);
  }

  for (int i = 0; i < 2; i++) {
    Serial.write(DataPacketFooter[i]);  // transmit the data over USB
    if (hpi_ble_enabled == true)
      Serial2.write(DataPacketFooter[i]);
  }
}

void setup() {
  delay(2000);
  Serial.begin(115200);  // Baudrate for serial communication
  Serial.println("Setting up Healthy PI 5...");

  pinMode(MAX30001_CS_PIN, OUTPUT);
  digitalWrite(MAX30001_CS_PIN, HIGH);

  SPI.setRX(SPI_MISO_PIN);
  SPI.setTX(SPI_MOSI_PIN);
  SPI.setSCK(SPI_SCK_PIN);
  SPI.begin();

  if (hpi_ble_enabled) {
    // Serial 1 -> UART0 connected to Raspberry Pi 40-pin header
    Serial2.setTX(PIN_RP2040_TX_ESP32_RX);
    Serial2.setRX(PIN_RP2040_RX_ESP32_TX);
    Serial2.begin(RP2040_ESP32_UART_BAUD);
  }

  delay(100);
  afe44xx.afe44xx_init();
  delay(100);

  max30001.BeginECGBioZ();

  Serial.println("Initialization is complete");
}

void setData(signed long ecg_sample, signed long bioz_sample, bool _bioZSkipSample) {

  DataPacket[0] = ecg_sample;
  DataPacket[1] = ecg_sample >> 8;
  DataPacket[2] = ecg_sample >> 16;
  DataPacket[3] = ecg_sample >> 24;

  DataPacket[4] = bioz_sample;
  DataPacket[5] = bioz_sample >> 8;
  DataPacket[6] = bioz_sample >> 16;
  DataPacket[7] = bioz_sample >> 24;

  if (_bioZSkipSample == false) {
    DataPacket[8] = 0x00;
  } else {
    DataPacket[8] = 0xFF;
  }
}

void loop() {
  afe44xx.get_AFE44XX_Data(&afe44xx_raw_data);
  ppg_wave_ir = (int16_t)(afe44xx_raw_data.IR_data >> 8);
  ppg_wave_ir = ppg_wave_ir;

  ppg_data_buff[ppg_stream_cnt++] = (uint8_t)ppg_wave_ir;
  ppg_data_buff[ppg_stream_cnt++] = (ppg_wave_ir >> 8);

  if (ppg_stream_cnt >= 19) {
    ppg_buf_ready = true;
    ppg_stream_cnt = 1;
  }

  memcpy(&DataPacket[9], &afe44xx_raw_data.IR_data, sizeof(signed long));
  memcpy(&DataPacket[13], &afe44xx_raw_data.RED_data, sizeof(signed long));

  if (afe44xx_raw_data.buffer_count_overflow) {

    if (afe44xx_raw_data.spo2 == -999) {
      DataPacket[19] = 0;
      spo2 = 0;
    } else {
      DataPacket[19] = afe44xx_raw_data.spo2;
      spo2 = (uint8_t)afe44xx_raw_data.spo2;
    }

    spo2_calc_done = true;
    afe44xx_raw_data.buffer_count_overflow = false;
  }

  ecg_data = max30001.getECGSamples();

  if (BioZSkipSample == false) {
    bioz_data = max30001.getBioZSamples();
    setData(ecg_data, bioz_data, BioZSkipSample);
    BioZSkipSample = true;
  } else {
    bioz_data = 0x00;
    setData(ecg_data, bioz_data, BioZSkipSample);
    BioZSkipSample = false;
  }

  send_data_serial_port();
  delay(8);
}
