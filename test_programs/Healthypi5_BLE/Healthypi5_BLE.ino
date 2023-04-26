//////////////////////////////////////////////////////////////////////////////////////////
//   Arduino program for HealthyPi 5 to work in BLE mode 
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
#include "SPIFFS.h"
#include <Update.h>
#include <FS.h> //Include File System Headers

#include "protocentral_afe44xx.h"
#include "Protocentral_MAX30205.h"
#include "Protocentral_spo2_algorithm.h"
//#include "Protocentral_MLX90632.h"

#include <protocentral_max30001.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define Heartrate_SERVICE_UUID (uint16_t(0x180D))
#define Heartrate_CHARACTERISTIC_UUID (uint16_t(0x2A37))
#define sp02_SERVICE_UUID (uint16_t(0x1822))
#define sp02_CHARACTERISTIC_UUID (uint16_t(0x2A5E))
#define DATASTREAM_SERVICE_UUID (uint16_t(0x1122))
#define DATASTREAM_CHARACTERISTIC_UUID (uint16_t(0x1424))
#define RESP_CHARACTERISTIC_UUID "babe4a4c-7789-11ed-a1eb-0242ac120002"
#define TEMP_SERVICE_UUID (uint16_t(0x1809))
#define TEMP_CHARACTERISTIC_UUID (uint16_t(0x2a6e))
#define BATTERY_SERVICE_UUID (uint16_t(0x180F))
#define BATTERY_CHARACTERISTIC_UUID (uint16_t(0x2a19))
#define HRV_SERVICE_UUID "cd5c7491-4448-7db8-ae4c-d1da8cba36d0"
#define HRV_CHARACTERISTIC_UUID "cd5ca86f-4448-7db8-ae4c-d1da8cba36d0"
#define HIST_CHARACTERISTIC_UUID "cd5c1525-4448-7db8-ae4c-d1da8cba36d0"

#define CES_CMDIF_PKT_START_1 0x0A
#define CES_CMDIF_PKT_START_2 0xFA
#define CES_CMDIF_TYPE_DATA 0x02
#define CES_CMDIF_PKT_STOP_1 0x00
#define CES_CMDIF_PKT_STOP 0x0B

#define CES_CMDIF_DATA_LEN_LSB 20
#define CES_CMDIF_DATA_LEN_MSB 0


#define MAX30205_READ_INTERVAL 10000
#define PPG_DATA 0X00
#define SENSOR_NOTFOUND 0xffff
#define TMP_MLX90632  2
#define TMP_MAX30205  0x01


int temperature;
float temp;
volatile long time_count = 0;

void send_data_serial_port(void);

uint8_t ecg_data_buff[40];
uint8_t resp_data_buff[40];
uint8_t ppg_data_buff[20];
uint8_t lead_flag = 0x04;
uint8_t data_len = 20;
uint8_t sp02;
uint32_t ecg_stream_cnt = 0;
uint32_t resp_stream_cnt = 0;
uint16_t ppg_stream_cnt = 0;
int16_t ppg_wave_ir;

int temperatureSensor = SENSOR_NOTFOUND;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool temp_data_ready = false;
bool spo2_calc_done = false;
bool ecg_buf_ready = false;
bool resp_buf_ready = false;
bool ppg_buf_ready = false;
bool BioZSkipSample = false;

char DataPacket[30];
String strValue = "";

const int MAX30001_CS_PIN = 13;
const int ADS1292_PWDN_PIN = 27;

//const int AFE4490_CS_PIN = 21;
//const int AFE4490_DRDY_PIN = 39;
const int AFE4490_PWDN_PIN = 4;

#define AFE44XX_CS_PIN   7
#define AFE44XX_PWDN_PIN 4

#define ZERO 0
//volatile char DataPacket[DATA_LEN];
const char DataPacketHeader[] = {CES_CMDIF_PKT_START_1, CES_CMDIF_PKT_START_2, CES_CMDIF_DATA_LEN_LSB, CES_CMDIF_DATA_LEN_MSB, CES_CMDIF_TYPE_DATA};
const char DataPacketFooter[] = {ZERO, CES_CMDIF_PKT_STOP};

BLEServer *pServer = NULL;
BLECharacteristic *Heartrate_Characteristic = NULL;
BLECharacteristic *sp02_Characteristic = NULL;
BLECharacteristic *datastream_Characteristic = NULL;
BLECharacteristic *battery_Characteristic = NULL;
BLECharacteristic *temperature_Characteristic = NULL;
BLECharacteristic *hist_Characteristic = NULL;
BLECharacteristic *hrv_Characteristic = NULL;
BLECharacteristic *resp_Characteristic = NULL;

MAX30001 max30001(MAX30001_CS_PIN);

signed long ecg_data;
signed long bioz_data;

AFE44XX afe44xx(AFE44XX_CS_PIN, AFE44XX_PWDN_PIN);
MAX30205 tempSensor;
//Protocentral_MLX90632 mlx90632;
spo2_algorithm spo2;
afe44xx_data afe44xx_raw_data;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("connected");
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("disconnected");
  }
};

class MyCallbackHandler : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *datastream_Characteristic)
  {
    std::string value = datastream_Characteristic->getValue();
    //int len = value.length();
    strValue = "0";

    if (value.length() > 0)
    {
      Serial.print("New value: ");

      for (int i = 0; i < value.length(); i++)
      {
        Serial.print(String(value[i]));
        strValue += value[i];
      }

      Serial.println();
    }
  }
};

float readTempData(void){

  /*if(temperatureSensor == TMP_MAX30205){
    return (tempSensor.getTemperature());
  }else if(temperatureSensor == TMP_MLX90632){
   return ( mlx90632.getSensorTemp());
  }*/

  Serial.println("unable to read temperature ");
  return 0xffff;
}

/*bool scanAvailableTempSensors(void){

  Wire.beginTransmission (MAX30205_ADDRESS1);
  if (Wire.endTransmission () == 0){
    tempSensor.max30205Address = MAX30205_ADDRESS1;
    temperatureSensor = TMP_MAX30205;

    return true;
  }

  Wire.beginTransmission (MAX30205_ADDRESS2);
  if(Wire.endTransmission () == 0){
    tempSensor.max30205Address = MAX30205_ADDRESS2;
    temperatureSensor = TMP_MAX30205;

    return true;
  }

  Wire.beginTransmission (MLX90632_ADDRESS);
  if(Wire.endTransmission () == 0){

    temperatureSensor = TMP_MLX90632;

    return true;
  }

  return false;
}*/

void HealthyPi5_BLE_Init()
{
  BLEDevice::init("Healthypi 5");     // Create the BLE Device
  pServer = BLEDevice::createServer(); // Create the BLE Server
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *HeartrateService = pServer->createService(Heartrate_SERVICE_UUID); // Create the BLE Service
  BLEService *sp02Service = pServer->createService(sp02_SERVICE_UUID);           // Create the BLE Service
  BLEService *TemperatureService = pServer->createService(TEMP_SERVICE_UUID);
  BLEService *batteryService = pServer->createService(BATTERY_SERVICE_UUID);
  BLEService *hrvService = pServer->createService(HRV_SERVICE_UUID);
  BLEService *datastreamService = pServer->createService(DATASTREAM_SERVICE_UUID);

  Heartrate_Characteristic = HeartrateService->createCharacteristic(
      Heartrate_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  sp02_Characteristic = sp02Service->createCharacteristic(
      sp02_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  temperature_Characteristic = TemperatureService->createCharacteristic(
      TEMP_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  battery_Characteristic = batteryService->createCharacteristic(
      BATTERY_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  hrv_Characteristic = hrvService->createCharacteristic(
      HRV_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  hist_Characteristic = hrvService->createCharacteristic(
      HIST_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  datastream_Characteristic = datastreamService->createCharacteristic(
      DATASTREAM_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  resp_Characteristic = datastreamService->createCharacteristic(
      RESP_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  Heartrate_Characteristic->addDescriptor(new BLE2902());
  sp02_Characteristic->addDescriptor(new BLE2902());
  temperature_Characteristic->addDescriptor(new BLE2902());
  battery_Characteristic->addDescriptor(new BLE2902());
  hist_Characteristic->addDescriptor(new BLE2902());
  hrv_Characteristic->addDescriptor(new BLE2902());
  resp_Characteristic->addDescriptor(new BLE2902());
  datastream_Characteristic->addDescriptor(new BLE2902());
  datastream_Characteristic->setCallbacks(new MyCallbackHandler());

  // Start the service
  HeartrateService->start();
  sp02Service->start();
  TemperatureService->start();
  batteryService->start();
  hrvService->start();
  datastreamService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(Heartrate_SERVICE_UUID);
  pAdvertising->addServiceUUID(sp02_SERVICE_UUID);
  pAdvertising->addServiceUUID(TEMP_SERVICE_UUID);
  pAdvertising->addServiceUUID(BATTERY_SERVICE_UUID);
  pAdvertising->addServiceUUID(HRV_SERVICE_UUID);
  pAdvertising->addServiceUUID(DATASTREAM_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x00); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void handle_ble_stack()
{

 if (ecg_buf_ready)
  {
    ecg_buf_ready = false;
    datastream_Characteristic->setValue(ecg_data_buff, 38);
    datastream_Characteristic->notify();
  }
  
  if (ppg_buf_ready)
  {
    ppg_buf_ready = false;
    hist_Characteristic->setValue(ppg_data_buff, 19);
    hist_Characteristic->notify();
  }

   if (resp_buf_ready)
  {
    resp_buf_ready = false;
    resp_Characteristic->setValue(resp_data_buff, 40);
    resp_Characteristic->notify();
  }

  if (spo2_calc_done)
  {
    // afe44xx_raw_data.buffer_count_overflow = false;
    uint8_t spo2_att_ble[5];
    spo2_att_ble[0] = 0x00;
    spo2_att_ble[1] = (uint8_t)sp02;
    spo2_att_ble[2] = (uint8_t)(sp02 >> 8);
    spo2_att_ble[3] = 0;
    spo2_att_ble[4] = 0;
    sp02_Characteristic->setValue(spo2_att_ble, 5);
    sp02_Characteristic->notify();
    spo2_calc_done = false;
  }


  if (temp_data_ready)
  {
    temperature_Characteristic->setValue((uint8_t *)&temperature, 2);
    temperature_Characteristic->notify();
    temp_data_ready = false;
  }

    if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }

   // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
  }


}



void send_data_serial_port(void)
{

  for (int i = 0; i < 5; i++)
  {
    Serial.write(DataPacketHeader[i]); // transmit the data over USB
  }

  for (int i = 0; i < 20; i++)
  {
    Serial.write(DataPacket[i]); // transmit the data over USB
  }

  for (int i = 0; i < 2; i++)
  {
    Serial.write(DataPacketFooter[i]); // transmit the data over USB
  }
}

void setup()
{
  delay(2000);
  Serial.begin(115200); // Baudrate for serial communication
  Serial.println("Setting up Healthy PI 5...");

  // initalize the  data ready and chip select pins:
  //pinMode(MAX30001_CS_PIN, OUTPUT);
  //pinMode(AFE4490_PWDN_PIN, OUTPUT);
  //pinMode(AFE4490_CS_PIN, OUTPUT);  //Slave Select
  //pinMode(AFE4490_DRDY_PIN, INPUT); // data ready

  HealthyPi5_BLE_Init();

  SPI.begin();
  //Wire.begin(25, 22);
  //SPI.setClockDivider(SPI_CLOCK_DIV16);
  //SPI.setBitOrder(MSBFIRST);
  //SPI.setDataMode(SPI_MODE0);
  //delay(10);
   
  afe44xx.afe44xx_init();
  
  delay(10);
  max30001.BeginECGBioZ();
  delay(10);
  
  /*if(scanAvailableTempSensors()){
    if(temperatureSensor == TMP_MLX90632){
      mlx90632.begin();
    }else if(temperatureSensor = TMP_MAX30205){
      tempSensor.begin();
    }
  }else{
    Serial.println("Couldn't find temperature sensor !!");
  }
  */
   
  Serial.println("Initialization is complete");

}


void setData(signed long ecg_sample, signed long bioz_sample, bool _bioZSkipSample) {

    ecg_data_buff[ecg_stream_cnt++] = (uint8_t)ecg_sample; 
    ecg_data_buff[ecg_stream_cnt++] = (uint8_t)(ecg_sample >> 8);   
    ecg_data_buff[ecg_stream_cnt++] = (uint8_t)(ecg_sample >> 16);
    ecg_data_buff[ecg_stream_cnt++] = (uint8_t)(ecg_sample >> 24);

    resp_data_buff[resp_stream_cnt++] = (uint8_t)bioz_sample;
    resp_data_buff[resp_stream_cnt++] = (uint8_t)(bioz_sample >> 8);
    resp_data_buff[resp_stream_cnt++] = (uint8_t)(bioz_sample >> 16);
    resp_data_buff[resp_stream_cnt++] = (uint8_t)(bioz_sample >> 24);
    

   if (_bioZSkipSample == false) {
    resp_data_buff[resp_stream_cnt++] = 0x00;
  } else {
    resp_data_buff[resp_stream_cnt++] = 0xFF;
  }
    
    if (ecg_stream_cnt >= 38)
    {
      ecg_buf_ready = true;
      ecg_stream_cnt = 0;
    }

    if (resp_stream_cnt >= 40)
    {
      resp_buf_ready = true;
      resp_stream_cnt = 0;
    }

}

void loop()
{
   ecg_data = max30001.getECGSamples();

   if (BioZSkipSample == false) {
    bioz_data = max30001.getBioZSamples();
    setData(ecg_data, bioz_data, BioZSkipSample);
    BioZSkipSample = true;
  } else
  {
    bioz_data = 0x00;
    setData(ecg_data, bioz_data, BioZSkipSample);
    BioZSkipSample=false;
  }
  delay(8);
  
    afe44xx.get_AFE44XX_Data(&afe44xx_raw_data);
    ppg_wave_ir = (int16_t)(afe44xx_raw_data.IR_data >> 8);
    ppg_wave_ir = ppg_wave_ir;

    ppg_data_buff[ppg_stream_cnt++] = (uint8_t)ppg_wave_ir;
    ppg_data_buff[ppg_stream_cnt++] = (ppg_wave_ir >> 8);

    if (ppg_stream_cnt >= 19)
    {
      ppg_buf_ready = true;
      ppg_stream_cnt = 0;
    }

    memcpy(&DataPacket[4], &afe44xx_raw_data.IR_data, sizeof(signed long));
    memcpy(&DataPacket[8], &afe44xx_raw_data.RED_data, sizeof(signed long));

    if (afe44xx_raw_data.buffer_count_overflow)
    {

      if (afe44xx_raw_data.spo2 == -999)
      {
        DataPacket[15] = 0;
        sp02 = 0;
      }
      else
      {
        DataPacket[15] = afe44xx_raw_data.spo2;
        sp02 = (uint8_t)afe44xx_raw_data.spo2;
        spo2_calc_done = true;
      }

      
      afe44xx_raw_data.buffer_count_overflow = false;
    }



    if ((time_count++ * (1000 / 128)) > MAX30205_READ_INTERVAL)
    {
      /*if(temperatureSensor != SENSOR_NOTFOUND ){
        temp = readTempData() * 100; // read temperature for every 100ms
        temperature = (uint16_t)temp;
        DataPacket[12] = (uint8_t)temperature;
        DataPacket[13] = (uint8_t)(temperature >> 8);
        temp_data_ready = true;
      }else scanAvailableTempSensors();
      */

      time_count = 0;
     
    }

    handle_ble_stack();
  }
