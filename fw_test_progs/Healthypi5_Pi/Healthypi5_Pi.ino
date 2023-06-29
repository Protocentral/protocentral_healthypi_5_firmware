
//////////////////////////////////////////////////////////////////////////////////////////
//   Arduino program for HealthyPi 5 to Work in PI mode 
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
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>

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

uint8_t ppg_data_buff[20];
uint8_t lead_flag = 0x04;
uint8_t data_len = 20;
uint8_t sp02;
uint16_t ppg_stream_cnt = 1;
int16_t ppg_wave_ir;

int temperatureSensor = SENSOR_NOTFOUND;

bool temp_data_ready = false;
bool spo2_calc_done = false;
bool ppg_buf_ready = false;
bool BioZSkipSample = false;

char DataPacket[30];

const int MAX30001_CS_PIN = 13;
const int ADS1292_PWDN_PIN = 27;
const int AFE4490_CS_PIN = 21;
const int AFE4490_DRDY_PIN = 39;
const int AFE4490_PWDN_PIN = 4;

#define ZERO 0
//volatile char DataPacket[DATA_LEN];
const char DataPacketHeader[] = {CES_CMDIF_PKT_START_1, CES_CMDIF_PKT_START_2, CES_CMDIF_DATA_LEN_LSB, CES_CMDIF_DATA_LEN_MSB, CES_CMDIF_TYPE_DATA};
const char DataPacketFooter[] = {ZERO, CES_CMDIF_PKT_STOP};

MAX30001 max30001(MAX30001_CS_PIN);

TFT_eSPI tft = TFT_eSPI(); 

long ecg_data;
signed long bioz_data;
long count = 0;

#define AFE44XX_CS_PIN   7
#define AFE44XX_PWDN_PIN 4

AFE44XX afe44xx(AFE44XX_CS_PIN, AFE44XX_PWDN_PIN);
MAX30205 tempSensor;
//Protocentral_MLX90632 mlx90632;
spo2_algorithm spo2;
afe44xx_data afe44xx_raw_data;

GraphWidget gr = GraphWidget(&tft);    
TraceWidget tr1 = TraceWidget(&gr);

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
  
  /*pinMode(AFE4490_PWDN_PIN, OUTPUT);
  pinMode(AFE4490_CS_PIN, OUTPUT);  //Slave Select
  pinMode(AFE4490_DRDY_PIN, INPUT); // data ready

  SPI.begin();
  Wire.begin(25, 22);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  */

  delay(10);
  afe44xx.afe44xx_init();

  delay(10);
  max30001.BeginECGBioZ();
  delay(10);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  
 // tft.pushImage(0, 0, iconWidth, iconHeight, icon);
 // tft.pushImage(390, 0, logoWidth, logoHeight, logoX);

  gr.createGraph(460, 200, tft.color565(5, 5, 5));
  gr.setGraphScale(0.0, 100.0, -50.0, 50.0);
  gr.setGraphGrid(0.0, 10.0, -50.0, 25, TFT_BLUE);   
  gr.drawGraph(10, 60);

  
tr1.addPoint(0.0, 0.0);
  tr1.addPoint(100.0, 0.0);

  
  tr1.startTrace(TFT_WHITE);
  
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
/*
void display_plots( long c)
{
  static uint32_t scanTime = millis();
  static float gx = 0.0, gy = 25.0;
  static bool cool = false;
  static uint16_t cnt = 0;

  // Sample periodically
  if (millis() - scanTime >= 1000) {
    tft.drawNumber(c, 10, 60);
    tr1.addPoint(gx, c);
    gx += 1.0;

    if (gy > 225.0) {
      gy = 224.0;
      cool = true;
      cnt = 0;
    }

    if (cool) {
      if (cnt > 20) gy -= 1.5;
      cnt++;
      if (gy < 25.0) {
        cool = false;
        gr.drawGraph(10, 60);
        tr1.startTrace(TFT_RED);
        tr1.addPoint(0.0, 183.0);
        tr1.addPoint(300.0, 183.0);

        tr1.startTrace(tft.color565(100, 100, 100));
        tr1.addPoint(0.0, 25.0);
        //tr1.addPoint(50.0, 120.0);
       // tr1.addPoint(140.0, 170.0);
       // tr1.addPoint(200.0, 225.0);
       // tr1.addPoint(220.0, 225.0);
       // tr1.addPoint(260.0, 150.0);

        tr1.startTrace(TFT_GREEN);
        gx = 0.0;
      }
    }
    else {
      if (gy < 120 || gy > 170.0) gy += 1.5;
      else gy += 0.40;
    }

    scanTime = millis();
  }  
}

void display_plot(float gx1, float gy1)
{
  static uint32_t plotTime = millis();
  static float gx = gx1, gy = gy1;
  static float delta = 7.0;

  if (millis() - plotTime >= 100) {
    plotTime = millis();

   tr1.addPoint(gx, gy);
 
    gx += 1.0;
    gy += delta;
    if (gy >  70.0) { delta = -7.0; gy =  70.0; }
    if (gy < -70.0) { delta =  7.0; gy = -70.0; }

    if (gx > 400.0) {
      gx = 0.0;
      gy = 0.0;

     // Draw empty graph at 40,10 on display
      gr.drawGraph(10, 60);
      // Start new trace
      tr1.startTrace(TFT_GREEN);

      tft.setTextSize(2);
  tft.setCursor(2, 290);
  tft.setTextColor(TFT_RED);  
  tft.println("HR:");
  tft.setCursor(50, 290);
  tft.setTextColor(TFT_YELLOW);  
  tft.println("65bpm");

  tft.setCursor(195, 290);
  tft.setTextColor(TFT_RED);  
  tft.println("SPO2:");
  tft.setCursor(257, 290);
  tft.setTextColor(TFT_YELLOW);  
  tft.println("96");

  tft.setCursor(350, 290);
  tft.setTextColor(TFT_RED);  
  tft.println("Resp:");
  tft.setCursor(410, 290);
  tft.setTextColor(TFT_YELLOW);  
  tft.println("20rpm");
      }

  }

}*/
/*
void display_plot1(long ecg)
{
  static uint32_t plotTime = millis();
  static float gx = 0.0, gy = 0.0;
  static float delta = 7.0;

  if (millis() - plotTime >= 500) {
    plotTime = millis();

    // Draw the x axis scale
  tft.setTextDatum(TC_DATUM); // Top centre text datum
  tft.drawNumber(0, gr.getPointX(0.0), gr.getPointY(-10.0));
  tft.drawNumber(50, gr.getPointX(50.0), gr.getPointY(-10.0));
  tft.drawNumber(100, gr.getPointX(100.0), gr.getPointY(-10.0));

  // Draw the y axis scale
  tft.setTextDatum(MR_DATUM); // Middle right text datum
  tft.drawNumber(-10, gr.getPointX(0.0), gr.getPointY(-10.0));
  tft.drawNumber(0, gr.getPointX(0.0), gr.getPointY(0.0));
  tft.drawNumber(5, gr.getPointX(0.0), gr.getPointY(5.0));
 
 //tft.drawNumber(ecg, 10, 60);
 tft.drawFloat(ecg, TC_DATUM, 10, 60);
tr1.addPoint(gx, ecg);
    
    
    
    gx += 1.0;    
 
  

     // Draw empty graph at 40,10 on display
      gr.drawGraph(10, 60);

  //    tr1.startTrace(TFT_RED);

  //tr1.addPoint(-10.0, 0.0);
  //tr1.addPoint(0.0, 5.0);
      // Start new trace
      tr1.startTrace(TFT_GREEN);
      gx = 0.0;
      tft.setTextSize(2);
  tft.setCursor(2, 290);
  tft.setTextColor(TFT_RED);  
  tft.println("HR:");

      tft.setTextSize(2);
  tft.setCursor(2, 290);
  tft.setTextColor(TFT_RED);  
  tft.println("HR:");
  tft.setCursor(50, 290);
  tft.setTextColor(TFT_YELLOW);  
  tft.println("65bpm");

  tft.setCursor(195, 290);
  tft.setTextColor(TFT_RED);  
  tft.println("SPO2:");
  tft.setCursor(257, 290);
  tft.setTextColor(TFT_YELLOW);  
  tft.println("96");

  tft.setCursor(350, 290);
  tft.setTextColor(TFT_RED);  
  tft.println("Resp:");
  tft.setCursor(410, 290);
  tft.setTextColor(TFT_YELLOW);  
  tft.println("20rpm");
      }

  

}*/

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
 ecg_data = max30001.getECGSamples();
     float delta = ecg_data/100000; 
     
static uint32_t plotTime = millis();
  static float gx = 0.0, gy = 0.0;
static float ppgVal = 0;  
 

  if (millis() - plotTime >= 50) {
    plotTime = millis();
    
   tr1.addPoint(gx, delta);
   gx += 1.0;

   if (gx > 100.0) {
      gx = 0.0;
      gy = 0.0;

    
      gr.drawGraph(10, 60);
     
      tr1.startTrace(TFT_GREEN);}}      
  
    afe44xx.get_AFE44XX_Data(&afe44xx_raw_data);
    ppg_wave_ir = (int16_t)(afe44xx_raw_data.IR_data >> 8);
    ppg_wave_ir = ppg_wave_ir;
      
    ppgVal = mapValue(ppg_wave_ir, (float)50000.0, (float)65535.0, (float)0.0, (float)100.0);
    Serial.println(afe44xx_raw_data.IR_data);              
      
    ppg_data_buff[ppg_stream_cnt++] = (uint8_t)ppg_wave_ir;
    ppg_data_buff[ppg_stream_cnt++] = (ppg_wave_ir >> 8);

    if (ppg_stream_cnt >= 19)
    {
      ppg_buf_ready = true;
      ppg_stream_cnt = 1;
    }

    memcpy(&DataPacket[9], &afe44xx_raw_data.IR_data, sizeof(signed long));
    memcpy(&DataPacket[13], &afe44xx_raw_data.RED_data, sizeof(signed long));

  

    if (afe44xx_raw_data.buffer_count_overflow)
    {

      if (afe44xx_raw_data.spo2 == -999)
      {
        DataPacket[19] = 0;
        sp02 = 0;
      }
      else
      {
        DataPacket[19] = afe44xx_raw_data.spo2;
        sp02 = (uint8_t)afe44xx_raw_data.spo2;
      }

      spo2_calc_done = true;
      afe44xx_raw_data.buffer_count_overflow = false;
    }
//send_data_serial_port();


    /*if ((time_count++ * (1000 / 128)) > MAX30205_READ_INTERVAL)
    {
      /*if(temperatureSensor != SENSOR_NOTFOUND ){
        temp = readTempData() * 100; // read temperature for every 100ms
        temperature = (uint16_t)temp;
        DataPacket[17] = (uint8_t)temperature;
        DataPacket[18] = (uint8_t)(temperature >> 8);
        temp_data_ready = true;
      }else scanAvailableTempSensors();*/

      time_count = 0;
     
    }*/

 
  }

  float mapValue(float ip, float ipmin, float ipmax, float tomin, float tomax) {
  return tomin + (((tomax - tomin) * (ip - ipmin)) / (ipmax - ipmin));
}

