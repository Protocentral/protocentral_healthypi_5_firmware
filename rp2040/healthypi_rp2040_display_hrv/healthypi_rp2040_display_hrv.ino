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

#include <SPI.h>
#include <Wire.h>
// #include <SD.h>

#include "src/lvgl/lvgl.h"
#include <TFT_eSPI.h>

#include "protocentral_afe44xx.h"
#include "Protocentral_spo2_algorithm.h"
#include <protocentral_max30001.h>

// Qwiic Sensor includes
#include <protocentral_TLA20xx.h>
#include "SparkFunCCS811.h"

#include "images.h"
#include "hpi_defines.h"

#include "healthypi_display.h"

// Qwiic Sensors
#define TLA20XX_I2C_ADDR 0x49
#define CCS811_ADDR 0x5B // Default I2C Address

TLA20XX tla2022(TLA20XX_I2C_ADDR);
CCS811 ccsSensor(CCS811_ADDR);

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
const char DataPacketHeader[] = {CES_CMDIF_PKT_START_1, CES_CMDIF_PKT_START_2, CES_CMDIF_DATA_LEN_LSB, CES_CMDIF_DATA_LEN_MSB, CES_CMDIF_TYPE_DATA};
const char DataPacketFooter[] = {0x00, CES_CMDIF_PKT_STOP};

signed long ecg_data;
signed long bioz_data;

uint8_t global_spo2 = 96;

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
#define TEMP_READ_INTERVAL 1000

#define TEMP_SENS_ADDRESS MAX30205_ADDRESS1

unsigned long prevCountTime = 0;
unsigned long prevTempCountTime = 0;

void send_data_serial_port(void)
{

  for (int i = 0; i < 5; i++)
  {
    Serial.write(DataPacketHeader[i]); // transmit the data over USB
    if (hpi_ble_enabled == true)
      Serial2.write(DataPacketHeader[i]);
  }

  for (int i = 0; i < 20; i++)
  {
    Serial.write(DataPacket[i]); // transmit the data over USB
    if (hpi_ble_enabled == true)
      Serial2.write(DataPacket[i]);
  }

  for (int i = 0; i < 2; i++)
  {
    Serial.write(DataPacketFooter[i]); // transmit the data over USB
    if (hpi_ble_enabled == true)
      Serial2.write(DataPacketFooter[i]);
  }
}

void setup()
{
  // delay(2000);
  Serial.begin(115200); // Baudrate for serial communication
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
  digitalWrite(HPI_LED_GREEN, LOW);

  // Configure SPI pins for AFE44XX & MAX30001
  SPI.setRX(SPI_MISO_PIN);
  SPI.setTX(SPI_MOSI_PIN);
  SPI.setSCK(SPI_SCK_PIN);

  SPI.begin();

  // Configure I2C pins for Temperature sensor/QWIIC ports
  Wire1.setSDA(6);
  Wire1.setSCL(7);

  Wire1.begin();

  i2c_write_byte(TEMP_SENS_ADDRESS, MAX30205_CONFIGURATION, 0x00); // mode config
  i2c_write_byte(TEMP_SENS_ADDRESS, MAX30205_THYST, 0x00);         // set threshold
  i2c_write_byte(TEMP_SENS_ADDRESS, MAX30205_TOS, 0x00);           //

  /*tla2022.begin();

  tla2022.setMode(TLA20XX::OP_CONTINUOUS);
  tla2022.setDR(TLA20XX::DR_128SPS);
  tla2022.setFSR(TLA20XX::FSR_2_048V);
  */

  if (ccsSensor.begin(Wire1) == false)
  {
    Serial.print("CCS811 error. Please check wiring.");
    // while (1)
    //;
  }

  if (hpi_ble_enabled)
  {
    // Serial 1 -> UART0 connected to Raspberry Pi 40-pin header
    Serial2.setTX(PIN_RP2040_TX_ESP32_RX);
    Serial2.setRX(PIN_RP2040_RX_ESP32_TX);
    Serial2.begin(115200);
  }

  delay(100);
  afe44xx.afe44xx_init();
  delay(100);

  max30001.BeginECGBioZ();

  hpi_display.init();
  delay(1000);
  // SD.begin(SD_CS_PIN,40000000, SPI1);

  Serial.println("Init complete");
}

void setData(signed long ecg_sample, signed long bioz_sample, bool _bioZSkipSample)
{
  DataPacket[0] = ecg_sample;
  DataPacket[1] = ecg_sample >> 8;
  DataPacket[2] = ecg_sample >> 16;
  DataPacket[3] = ecg_sample >> 24;

  DataPacket[4] = bioz_sample;
  DataPacket[5] = bioz_sample >> 8;
  DataPacket[6] = bioz_sample >> 16;
  DataPacket[7] = bioz_sample >> 24;

  if (_bioZSkipSample == false)
  {
    DataPacket[8] = 0x00;
  }
  else
  {
    DataPacket[8] = 0xFF;
  }
}

void loop()
{
  unsigned long currentTime = millis();

  max30001.max30001ServiceAllInterrupts();
  if(true) //(max30001.ecgSamplesAvailable > 0)
  {
    for (int i = 0; i < max30001.ecgSamplesAvailable; i++)
    {
      fltECGSamples[i] = (float)(max30001.s32ECGData[i] / 100);
      // setData(max30001.s32ECGData[i], 0, false);
      // send_data_serial_port();
    }
   
    hpi_display.draw_plotECG(fltECGSamples, max30001.ecgSamplesAvailable);

    hpi_display.add_samples(max30001.ecgSamplesAvailable);
    max30001.ecgSamplesAvailable = 0;
  }

  if (max30001.biozSamplesAvailable > 0)
  {
    for (int i = 0; i < max30001.biozSamplesAvailable; i++)
    {
      fltBIOZSamples[i] = (float)(max30001.s32BIOZData[i] / 100);

      // setData(max30001.s32BIOZData[i], 0, false);
      // send_data_serial_port();
    }

    //hpi_display.draw_plotresp(fltBIOZSamples, max30001.biozSamplesAvailable);

    max30001.biozSamplesAvailable = 0;
  }

  // gx++;

  // send_data_serial_port();

  // Sample PPG every 8 ms
  if (currentTime - prevCountTime >= PPG_READ_INTERVAL)
  {

    afe44xx.get_AFE44XX_Data(&afe44xx_raw_data);
    ppg_wave_ir = (int16_t)(afe44xx_raw_data.IR_data >> 8);
    ppg_wave_ir = ppg_wave_ir;

    signed long ppgVal1 = (afe44xx_raw_data.RED_data >> 8);

    redPlot = (float)(ppgVal1); // = (float) map(ppgVal1, (float)1000, (float)2500.0, (float)0.0, (float)100.0);
    // float redPlot1 = ppgVal1/100000;

    ppg_data_buff[ppg_stream_cnt++] = (uint8_t)ppg_wave_ir;
    ppg_data_buff[ppg_stream_cnt++] = (ppg_wave_ir >> 8);

    if (ppg_stream_cnt >= 19)
    {
      ppg_buf_ready = true;
      ppg_stream_cnt = 1;
    }

    memcpy(&DataPacket[9], &afe44xx_raw_data.IR_data, sizeof(signed long));
    memcpy(&DataPacket[13], &afe44xx_raw_data.RED_data, sizeof(signed long));

    // draw_plotppg(redPlot);
    //hpi_display.draw_plotppg(redPlot);

    if (afe44xx_raw_data.buffer_count_overflow)
    {

      if (afe44xx_raw_data.spo2 == -999)
      {
        DataPacket[19] = 0;
        sp02 = 0;
        //hpi_display.updateSpO2((uint8_t)afe44xx_raw_data.spo2, false);
      }
      else
      {
        DataPacket[19] = afe44xx_raw_data.spo2;
        // sp02 = (uint8_t)afe44xx_raw_data.spo2;
        //hpi_display.updateSpO2((uint8_t)afe44xx_raw_data.spo2, true);
        //hpi_display.updateHR((uint8_t)80);
        //hpi_display.updateRR((uint8_t)20);
      }

      spo2_calc_done = true;
      afe44xx_raw_data.buffer_count_overflow = false;
    }

    prevCountTime = currentTime;
  }

  if (currentTime - prevTempCountTime >= TEMP_READ_INTERVAL)
  {
    float tread = getTemperature();
    //hpi_display.updateTemp(tread);
    prevTempCountTime = currentTime;
  }

  // float val = tla2022.read_adc(); // +/- 2.048 V FSR, 1 LSB = 1 mV
  // Serial.println(val);

  // Check to see if data is ready with .dataAvailable()
  if (ccsSensor.dataAvailable())
  {
    ccsSensor.readAlgorithmResults();

    //hpi_display.updateEnv(ccsSensor.getCO2(), ccsSensor.getTVOC());
  }

  hpi_display.do_set_scale();

  lv_timer_handler(); /* let the GUI do its work */

  // delay(1);
}

float mapValue(float ip, float ipmin, float ipmax, float tomin, float tomax)
{
  return tomin + (((tomax - tomin) * (ip - ipmin)) / (ipmax - ipmin));
}

float getTemperature(void)
{
  uint8_t readRaw[2] = {0};
  i2c_read_bytes(TEMP_SENS_ADDRESS, MAX30205_TEMPERATURE, &readRaw[0], 2); // read two bytes
  int16_t raw = readRaw[0] << 8 | readRaw[1];                              // combine two bytes
  float temperature = raw * 0.00390625;                                    // convert to temperature
  return temperature;
}

void i2c_read_bytes(uint8_t address, uint8_t subAddress, uint8_t *dest, uint8_t count)
{
  Wire1.beginTransmission(address); // Initialize the Tx buffer
  // Next send the register to be read. OR with 0x80 to indicate multi-read.
  Wire1.write(subAddress);
  Wire1.endTransmission(false);
  uint8_t i = 0;
  Wire1.requestFrom(address, count); // Read bytes from slave register address
  while (Wire1.available())
  {
    dest[i++] = Wire1.read();
  }
}

// Wire.h read and write protocols
void i2c_write_byte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire1.beginTransmission(address); // Initialize the Tx buffer
  Wire1.write(subAddress);          // Put slave register address in Tx buffer
  Wire1.write(data);                // Put data in Tx buffer
  Wire1.endTransmission();          // Send the Tx buffer
}
