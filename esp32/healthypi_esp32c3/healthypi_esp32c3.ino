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

/************** Packet Validation  **********************/
#define CESState_Init 0
#define CESState_SOF1_Found 1
#define CESState_SOF2_Found 2
#define CESState_PktLen_Found 3

/*CES CMD IF Packet Format*/
#define CES_CMDIF_PKT_START_1 0x0A
#define CES_CMDIF_PKT_START_2 0xFA
#define CES_CMDIF_PKT_STOP 0x0B

/*CES CMD IF Packet Indices*/
#define CES_CMDIF_IND_LEN 2
#define CES_CMDIF_IND_LEN_MSB 3
#define CES_CMDIF_IND_PKTTYPE 4

#define CES_CMDIF_PKT_OVERHEAD 5

/************** Packet Related Variables **********************/

int ecs_rx_state = 0;                      // To check the state of the packet
int CES_Pkt_Len;                           // To store the Packet Length Deatils
int CES_Pkt_Pos_Counter, CES_Data_Counter; // Packet and data counter
int CES_Pkt_PktType;                       // To store the Packet Type
char CES_Pkt_Data_Counter[1000];           // Buffer to store the data from the packet
char CES_Pkt_ECG_Counter[4];               // Buffer to hold ECG data
char CES_Pkt_Resp_Counter[4];              // Respiration Buffer
char CES_Pkt_SpO2_Counter_RED[4];          // Buffer for SpO2 RED
char CES_Pkt_SpO2_Counter_IR[4];           // Buffer for SpO2 IR

char respdataTag = 0;
signed long ecg, resp;
int16_t spo2_ir, spo2_red;

uint8_t ecg_data_buff[40];
uint8_t resp_data_buff[40];
uint8_t ppg_data_buff[20];
int sp02;
uint32_t ecg_stream_cnt = 0;
uint32_t resp_stream_cnt = 0;
uint16_t ppg_stream_cnt = 0;
float temp;

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool temp_data_ready = false;
bool spo2_calc_done = false;
bool ecg_buf_ready = false;
bool resp_buf_ready = false;
bool ppg_buf_ready = false;

String strValue = "";

BLEServer *pServer = NULL;
BLECharacteristic *Heartrate_Characteristic = NULL;
BLECharacteristic *sp02_Characteristic = NULL;
BLECharacteristic *datastream_Characteristic = NULL;
BLECharacteristic *battery_Characteristic = NULL;
BLECharacteristic *temperature_Characteristic = NULL;
BLECharacteristic *hist_Characteristic = NULL;
BLECharacteristic *hrv_Characteristic = NULL;
BLECharacteristic *resp_Characteristic = NULL;

#define PIN_RP2040_TX_ESP32_RX 7
#define PIN_RP2040_RX_ESP32_TX 6

#define RP2040_ESP32_UART_BAUD 230400

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
    // int len = value.length();
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

void HealthyPi5_BLE_Init()
{
  BLEDevice::init("Healthypi 5");      // Create the BLE Device
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
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  sp02_Characteristic = sp02Service->createCharacteristic(
      sp02_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  temperature_Characteristic = TemperatureService->createCharacteristic(
      TEMP_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  battery_Characteristic = batteryService->createCharacteristic(
      BATTERY_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  hrv_Characteristic = hrvService->createCharacteristic(
      HRV_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  hist_Characteristic = hrvService->createCharacteristic(
      HIST_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  datastream_Characteristic = datastreamService->createCharacteristic(
      DATASTREAM_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  resp_Characteristic = datastreamService->createCharacteristic(
      RESP_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

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
  pAdvertising->setMinPreferred(0x00);
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

  if (1) //(spo2_calc_done)
  {
    sp02 = 96;

    uint8_t spo2_att_ble[5];
    spo2_att_ble[0] = 0x00;
    spo2_att_ble[1] = (uint8_t)sp02;
    spo2_att_ble[2] = (uint8_t)(sp02 >> 8);
    spo2_att_ble[3] = 0;
    spo2_att_ble[4] = 0;
    sp02_Characteristic->setValue(spo2_att_ble, 5);
    sp02_Characteristic->notify();
    spo2_calc_done = false;

    sp02 = 72;

    uint8_t hr_att_ble[5];
    hr_att_ble[0] = 0x00;
    hr_att_ble[1] = (uint8_t)sp02;
    hr_att_ble[2] = (uint8_t)(sp02 >> 8);
    hr_att_ble[3] = 0;
    hr_att_ble[4] = 0;

    Heartrate_Characteristic->setValue(hr_att_ble, 5);
    Heartrate_Characteristic->notify();

    uint8_t resp = 21;
    uint8_t resp_att_ble[11];
    resp_att_ble[0] = 0x00;

    resp_att_ble[1] = (uint8_t)resp;
    resp_att_ble[2] = (uint8_t)(resp >> 8);
    resp_att_ble[3] = 0;
    resp_att_ble[4] = 0;
    resp_att_ble[5] = 0;
    resp_att_ble[6] = 0;
    resp_att_ble[7] = 0;
    resp_att_ble[8] = 0;
    resp_att_ble[9] = 0;
    resp_att_ble[10] = resp;

    hrv_Characteristic->setValue(resp_att_ble, 11);
    hrv_Characteristic->notify();
  }

  if (1) //(temp_data_ready)
  {
    temp = 37.5;
    uint16_t temp_uint = (uint16_t)(temp * 100);
    uint8_t temp_data[2];
    temp_data[0] = (uint8_t)temp_uint;
    temp_data[1] = (uint8_t)(temp_uint >> 8);

    temperature_Characteristic->setValue(temp_data, 2);
    temperature_Characteristic->notify();
    temp_data_ready = false;
  }

  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
  }

  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
}

void hPiProcessData(char rxch)
{
  switch (ecs_rx_state)
  {
  case CESState_Init:
    if (rxch == CES_CMDIF_PKT_START_1)
      ecs_rx_state = CESState_SOF1_Found;
    break;

  case CESState_SOF1_Found:
    if (rxch == CES_CMDIF_PKT_START_2)
      ecs_rx_state = CESState_SOF2_Found;
    else
      ecs_rx_state = CESState_Init; // Invalid Packet, reset state to init
    break;

  case CESState_SOF2_Found:
    //    println("inside 3");
    ecs_rx_state = CESState_PktLen_Found;
    CES_Pkt_Len = (int)rxch;
    CES_Pkt_Pos_Counter = CES_CMDIF_IND_LEN;
    CES_Data_Counter = 0;
    break;

  case CESState_PktLen_Found:
    //    println("inside 4");
    CES_Pkt_Pos_Counter++;
    if (CES_Pkt_Pos_Counter < CES_CMDIF_PKT_OVERHEAD) // Read Header
    {
      if (CES_Pkt_Pos_Counter == CES_CMDIF_IND_LEN_MSB)
        CES_Pkt_Len = (int)((rxch << 8) | CES_Pkt_Len);
      else if (CES_Pkt_Pos_Counter == CES_CMDIF_IND_PKTTYPE)
        CES_Pkt_PktType = (int)rxch;
    }
    else if ((CES_Pkt_Pos_Counter >= CES_CMDIF_PKT_OVERHEAD) && (CES_Pkt_Pos_Counter < CES_CMDIF_PKT_OVERHEAD + CES_Pkt_Len + 1)) // Read Data
    {
      if (CES_Pkt_PktType == 2)
      {
        CES_Pkt_Data_Counter[CES_Data_Counter++] = (char)(rxch); // Buffer that assigns the data separated from the packet
      }
    }
    else // All  and data received
    {
      if (rxch == CES_CMDIF_PKT_STOP)
      {
        // Serial.println(CES_Pkt_Len);

        CES_Pkt_ECG_Counter[0] = CES_Pkt_Data_Counter[0];
        CES_Pkt_ECG_Counter[1] = CES_Pkt_Data_Counter[1];
        CES_Pkt_ECG_Counter[2] = CES_Pkt_Data_Counter[2];
        CES_Pkt_ECG_Counter[3] = CES_Pkt_Data_Counter[3];

        CES_Pkt_Resp_Counter[0] = CES_Pkt_Data_Counter[4];
        CES_Pkt_Resp_Counter[1] = CES_Pkt_Data_Counter[5];
        CES_Pkt_Resp_Counter[2] = CES_Pkt_Data_Counter[6];
        CES_Pkt_Resp_Counter[3] = CES_Pkt_Data_Counter[7];

        respdataTag = CES_Pkt_Data_Counter[8];

        CES_Pkt_SpO2_Counter_IR[0] = CES_Pkt_Data_Counter[9];
        CES_Pkt_SpO2_Counter_IR[1] = CES_Pkt_Data_Counter[10];
        CES_Pkt_SpO2_Counter_IR[2] = CES_Pkt_Data_Counter[11];
        CES_Pkt_SpO2_Counter_IR[3] = CES_Pkt_Data_Counter[12];

        CES_Pkt_SpO2_Counter_RED[0] = CES_Pkt_Data_Counter[13];
        CES_Pkt_SpO2_Counter_RED[1] = CES_Pkt_Data_Counter[14];
        CES_Pkt_SpO2_Counter_RED[2] = CES_Pkt_Data_Counter[15];
        CES_Pkt_SpO2_Counter_RED[3] = CES_Pkt_Data_Counter[16];

        // temp = (float) ((int) CES_Pkt_Data_Counter[17]| CES_Pkt_Data_Counter[18]<<8)/100;

        int data1 = CES_Pkt_ECG_Counter[0] | CES_Pkt_ECG_Counter[1] << 8 | CES_Pkt_ECG_Counter[2] << 16 | CES_Pkt_ECG_Counter[3] << 24; // reversePacket(CES_Pkt_ECG_Counter, CES_Pkt_ECG_Counter.length-1);
        ecg = (int32_t)data1;

        int data2 = CES_Pkt_Resp_Counter[0] | CES_Pkt_Resp_Counter[1] << 8 | CES_Pkt_Resp_Counter[2] << 16 | CES_Pkt_Resp_Counter[3] << 24; // reversePacket(CES_Pkt_ECG_Counter, CES_Pkt_ECG_Counter.length-1);
        resp = (int32_t)data2;

        int data3 = CES_Pkt_SpO2_Counter_IR[0] | CES_Pkt_SpO2_Counter_IR[1] << 8 | CES_Pkt_SpO2_Counter_IR[2] << 16 | CES_Pkt_SpO2_Counter_IR[3] << 24; // reversePacket(CES_Pkt_SpO2_Counter_IR, CES_Pkt_SpO2_Counter_IR.length-1);
        spo2_ir = (int16_t)(data3 >> 8);
        spo2_ir = spo2_ir;

        int data4 = CES_Pkt_SpO2_Counter_RED[0] | CES_Pkt_SpO2_Counter_RED[1] << 8 | CES_Pkt_SpO2_Counter_RED[2] << 16 | CES_Pkt_SpO2_Counter_RED[3] << 24; // reversePacket(CES_Pkt_SpO2_Counter_RED, CES_Pkt_SpO2_Counter_RED.length-1);
        spo2_red = (uint16_t)data4;

        setData(ecg, resp, respdataTag, spo2_ir);

        sp02 = (int)(CES_Pkt_Data_Counter[19]);
        spo2_calc_done = true;

        ecs_rx_state = CESState_Init;
      }
      else
      {
        ecs_rx_state = CESState_Init;
      }
    }
    break;

  default:
    break;
  }
}

void setData(uint32_t ecg_sample, uint32_t bioz_sample, char resp_tag, uint16_t ppg_wave_ir)
{

  ecg_data_buff[ecg_stream_cnt++] = (uint8_t)ecg_sample;
  ecg_data_buff[ecg_stream_cnt++] = (uint8_t)(ecg_sample >> 8);
  ecg_data_buff[ecg_stream_cnt++] = (uint8_t)(ecg_sample >> 16);
  ecg_data_buff[ecg_stream_cnt++] = (uint8_t)(ecg_sample >> 24);

  resp_data_buff[resp_stream_cnt++] = (uint8_t)bioz_sample;
  resp_data_buff[resp_stream_cnt++] = (uint8_t)(bioz_sample >> 8);
  resp_data_buff[resp_stream_cnt++] = (uint8_t)(bioz_sample >> 16);
  resp_data_buff[resp_stream_cnt++] = (uint8_t)(bioz_sample >> 24);

  resp_data_buff[resp_stream_cnt++] = resp_tag;

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

  ppg_data_buff[ppg_stream_cnt++] = (uint8_t)ppg_wave_ir;
  ppg_data_buff[ppg_stream_cnt++] = (uint8_t)(ppg_wave_ir >> 8);

  if (ppg_stream_cnt >= 19)
  {
    ppg_buf_ready = true;
    ppg_stream_cnt = 0;
  }
}

void setup()
{
  delay(2000);
  Serial.begin(115200); // Baudrate for serial communication
  Serial.println("Setting up Healthy PI 5 with ESP32...");

  Serial1.begin(RP2040_ESP32_UART_BAUD, SERIAL_8N1, PIN_RP2040_TX_ESP32_RX, PIN_RP2040_RX_ESP32_TX, false, 20000UL, 112);

  HealthyPi5_BLE_Init();

  Serial.println("Initialization is complete");
}

unsigned long prevCountTime = 0;

void loop()
{
  unsigned long currentTime = millis();

  if (Serial1.available())
  {
    hPiProcessData(Serial1.read());
    // Serial.write(Serial1.read());
  }

  if (currentTime - prevCountTime >= 8)
  {
    handle_ble_stack();
    prevCountTime = currentTime;
  }
}
