//////////////////////////////////////////////////////////////////////////////////////////
//   Arduino program for HealthyPi 5 with standalone display
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

/*

Building this application

* Required Libraries (install from Arduino Library Manager)
* ProtoCentral AFE4490 by ProtoCentral Electronics
* ProtoCentral MAX30001 by ProtoCentral Electronics
* TFT_eSPI by Bodmer

Copy "User_Setup_hpi5_2.h" into the TFT_eSPI library and set this file 
as the default display (see here for more info: https://github.com/Bodmer/TFT_eSPI#tips )

* **Important**: Change the "Optimize" setting in the "Tools" menu to "Optimize More (-O2)" or higher, 
due to limitations with the LVGL library

*/

#include <SPI.h>
#include <Wire.h>
// #include <SD.h>

#include "src/lvgl/lvgl.h"
#include <TFT_eSPI.h>

#include "protocentral_afe44xx.h"
#include "Protocentral_spo2_algorithm.h"
#include <protocentral_max30001.h>

#include "images.h"

// Contains pin definitions and global constants
#include "hpi_defines.h"

#include "healthypi_display.h"

void send_data_serial_port(void);

uint8_t ppg_data_buff[20];
uint8_t lead_flag = 0x04;
uint8_t data_len = 20;
uint8_t sp02;
uint16_t ppg_stream_cnt = 1;
int16_t ppg_wave_ir;

bool spo2_calc_done = false;
bool ppg_buf_ready = false;
bool BioZSkipSample = false;

char DataPacket[30];
static uint32_t plotTime = millis();

// Packet format for communication with OpenView
const char DataPacketHeader[] = { CES_CMDIF_PKT_START_1, CES_CMDIF_PKT_START_2, CES_CMDIF_DATA_LEN_LSB, CES_CMDIF_DATA_LEN_MSB, CES_CMDIF_TYPE_DATA };
const char DataPacketFooter[] = { 0x00, CES_CMDIF_PKT_STOP };

signed long ecg_data;
signed long bioz_data;

uint8_t global_spo2 = 96;

float ecg_mult = 0.00004768;  //VREF=1000 mV, GAIN=160 V/V
float resp_mult = 1.49011612;

MAX30001 max30001(MAX30001_CS_PIN);
AFE44XX afe44xx(AFE44XX_CS_PIN, AFE44XX_PWDN_PIN);
HealthyPi_Display hpi_display;

spo2_algorithm spo2;
afe44xx_data afe44xx_raw_data;

const bool hpi_ble_enabled = true;

float respPlot = 0;
float redPlot = 0;
float ecgPlot = 0;

float fltECGSamples[128];
float fltBIOZSamples[128];

#define PPG_READ_INTERVAL 8
#define ECG_READ_INTERVAL 8
#define TEMP_READ_INTERVAL 1000
#define RP2040_ESP32_UART_BAUD 230400

#define TEMP_SENS_ADDRESS MAX30205_ADDRESS1

unsigned long prevCountTime = 0;
unsigned long prevCountTime1 = 0;

unsigned long prevTempCountTime = 0;
unsigned long prevECGCountTime = 0;

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
  // delay(2000);
  Serial.begin(115200);  // Baudrate for serial communication
  Serial.println("Setting up Healthy PI 5...");

  // Set up MAX30001 pins
  pinMode(MAX30001_CS_PIN, OUTPUT);
  pinMode(MAX30001_INTB_PIN, INPUT);

  // Set up AFE44XX pins
  pinMode(AFE44XX_CS_PIN, OUTPUT);
  pinMode(AFE44XX_DRDY_PIN, INPUT);

  // Set up LED pins
  pinMode(HPI_LED_BLUE, OUTPUT);
  pinMode(HPI_LED_GREEN, OUTPUT);

  // Set Initial LED states
  digitalWrite(HPI_LED_BLUE, HIGH);
  digitalWrite(HPI_LED_GREEN, HIGH);

  // Configure SPI pins for AFE44XX & MAX30001
  SPI.setRX(SPI_MISO_PIN);
  SPI.setTX(SPI_MOSI_PIN);
  SPI.setSCK(SPI_SCK_PIN);

  SPI.begin();

  hpi_display.init();
  delay(1000);
  // SD.begin(SD_CS_PIN,40000000, SPI1);

  // Configure I2C pins for Temperature sensor/QWIIC ports
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();

  i2c_write_byte(TEMP_SENS_ADDRESS, MAX30205_CONFIGURATION, 0x00);  // mode config
  i2c_write_byte(TEMP_SENS_ADDRESS, MAX30205_THYST, 0x00);          // set threshold
  i2c_write_byte(TEMP_SENS_ADDRESS, MAX30205_TOS, 0x00);            //

  if (hpi_ble_enabled) {
    // Serial 1 -> UART0 connected to Raspberry Pi 40-pin header
    Serial2.setTX(PIN_RP2040_TX_ESP32_RX);
    Serial2.setRX(PIN_RP2040_RX_ESP32_TX);
    Serial2.begin(RP2040_ESP32_UART_BAUD);
  }

  delay(500);
  afe44xx.afe44xx_init();
  delay(500);
  max30001.BeginECGBioZ();
  delay(500);
  
  Serial.println("Init complete");
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
  unsigned long currentTime = millis();

  afe44xx.get_AFE44XX_Data(&afe44xx_raw_data);
  ppg_wave_ir = (int16_t)(afe44xx_raw_data.IR_data >> 8);
  ppg_wave_ir = ppg_wave_ir;

  signed long ppgVal1 = (afe44xx_raw_data.IR_data >> 8);

  ppg_data_buff[ppg_stream_cnt++] = (uint8_t)ppg_wave_ir;
  ppg_data_buff[ppg_stream_cnt++] = (ppg_wave_ir >> 8);

  //redPlot = (float)(ppg_wave_ir/10000);  // = (float) map(ppgVal1, (float)1000, (float)2500.0, (float)0.0, (float)100.0);
  float redPlot = (float)(ppg_wave_ir/10.000000);

  hpi_display.draw_plotECG(redPlot);
  hpi_display.add_samples(1);
  hpi_display.do_set_scale();

  if (ppg_stream_cnt >= 19) {
    ppg_buf_ready = true;
    ppg_stream_cnt = 1;
  }

  memcpy(&DataPacket[9], &afe44xx_raw_data.IR_data, sizeof(signed long));
  memcpy(&DataPacket[13], &afe44xx_raw_data.RED_data, sizeof(signed long));

  //hpi_display.draw_plotECG(ecg_data);
  //hpi_display.draw_plotppg(redPlot);

  if (afe44xx_raw_data.buffer_count_overflow) {

    if (afe44xx_raw_data.spo2 == -999) {
      DataPacket[19] = 0;
      sp02 = 0;
      hpi_display.updateSpO2((uint8_t)afe44xx_raw_data.spo2, false);
    } else {
      DataPacket[19] = afe44xx_raw_data.spo2;
      sp02 = (uint8_t)afe44xx_raw_data.spo2;
      hpi_display.updateSpO2((uint8_t)afe44xx_raw_data.spo2, true);
      hpi_display.updateHR((uint8_t)80);
      hpi_display.updateRR((uint8_t)20);
    }

    spo2_calc_done = true;
    afe44xx_raw_data.buffer_count_overflow = false;
  }

  max30001.max30001ServiceAllInterrupts();
  if (max30001.ecgSamplesAvailable > 0) {
    for (int i = 0; i < max30001.ecgSamplesAvailable; i++) {
      fltECGSamples[i] = (float)(max30001.s32ECGData[i] * ecg_mult);
      setData(max30001.s32ECGData[i], 0, false);
      // send_data_serial_port();
      //hpi_display.draw_plotECG(fltECGSamples[i]);
    }
    //hpi_display.add_samples(max30001.ecgSamplesAvailable);
    //hpi_display.add_samples();
    // /hpi_display.do_set_scale();
    // Serial.print("ECG:");
    // Serial.println(max30001.ecgSamplesAvailable);
    // draw_plotECG(fltECGSamples, max30001.ecgSamplesAvailable);

    // draw_plotECG(fltECGSamples, max30001.ecgSamplesAvailable);
    //hpi_display.draw_plotECG(fltECGSamples, max30001.ecgSamplesAvailable);

    //hpi_display.add_samples(max30001.ecgSamplesAvailable);
    // gx += max30001.ecgSamplesAvailable;
    max30001.ecgSamplesAvailable = 0;

    //hpi_display.add_samples(1);
  }

  if (max30001.biozSamplesAvailable > 0) {
    for (int i = 0; i < max30001.biozSamplesAvailable; i++) {
      fltBIOZSamples[i] = (float)(max30001.s32BIOZData[i] * resp_mult);
      setData(max30001.s32BIOZData[i], 0, false);
      //hpi_display.draw_plotresp(fltBIOZSamples[i]);  //, max30001.biozSamplesAvailable);
      // send_data_serial_port();
    }
    // Serial.print("ECG:");
    // Serial.println(max30001.ecgSamplesAvailable);
    // draw_plotECG(fltECGSamples, max30001.ecgSamplesAvailable);

    // draw_plotresp(fltBIOZSamples, max30001.biozSamplesAvailable);
    //hpi_display.draw_plotresp(fltBIOZSamples, max30001.biozSamplesAvailable);

    max30001.biozSamplesAvailable = 0;
  }


  /*  ecg_data = max30001.getECGSamples();

  float fl_ecg_data = ecg_data * ecg_mult;

  hpi_display.draw_plotECG(fl_ecg_data);

  hpi_display.add_samples(1);
  hpi_display.do_set_scale();

  if (BioZSkipSample == false) {
    bioz_data = max30001.getBioZSamples();
    setData(ecg_data, bioz_data, BioZSkipSample);
    BioZSkipSample = true;
  } else {
    bioz_data = 0x00;
    setData(ecg_data, bioz_data, BioZSkipSample);
    BioZSkipSample = false;
  }*/

  /*if (currentTime - prevTempCountTime >= TEMP_READ_INTERVAL) {
    float tread = getTemperature();
    hpi_display.updateTemp(tread);
    prevTempCountTime = currentTime;
  }*/

  send_data_serial_port();

  lv_timer_handler();
  delay(8);
}

float mapValue(float ip, float ipmin, float ipmax, float tomin, float tomax) {
  return tomin + (((tomax - tomin) * (ip - ipmin)) / (ipmax - ipmin));
}

float getTemperature(void) {
  uint8_t readRaw[2] = { 0 };
  i2c_read_bytes(TEMP_SENS_ADDRESS, MAX30205_TEMPERATURE, &readRaw[0], 2);  // read two bytes
  int16_t raw = readRaw[0] << 8 | readRaw[1];                               // combine two bytes
  float temperature = raw * 0.00390625;                                     // convert to temperature
  return temperature;
}

void i2c_read_bytes(uint8_t address, uint8_t subAddress, uint8_t *dest, uint8_t count) {
  Wire1.beginTransmission(address);  // Initialize the Tx buffer
  // Next send the register to be read. OR with 0x80 to indicate multi-read.
  Wire1.write(subAddress);
  Wire1.endTransmission(false);
  uint8_t i = 0;
  Wire1.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire1.available()) {
    dest[i++] = Wire1.read();
  }
}

// Wire.h read and write protocols
void i2c_write_byte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire1.beginTransmission(address);  // Initialize the Tx buffer
  Wire1.write(subAddress);           // Put slave register address in Tx buffer
  Wire1.write(data);                 // Put data in Tx buffer
  Wire1.endTransmission();           // Send the Tx buffer
}
