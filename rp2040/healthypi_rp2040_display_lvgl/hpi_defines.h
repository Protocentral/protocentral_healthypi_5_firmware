// HealthyPi 5.2 Pin definitions for RP2040
#ifndef __HPI_DEFS_H__
#define __HPI_DEFS_H__

#define SPI_SCK_PIN 2
#define SPI_MOSI_PIN 3
#define SPI_MISO_PIN 4

#define MAX30001_CS_PIN 5
#define MAX30001_INTB_PIN 20
#define MAX30001_INT2B_PIN 19

#define AFE44XX_CS_PIN 27
#define AFE44XX_DRDY_PIN 17
#define AFE44XX_PWDN_PIN 18

#define PIN_RP2040_TX_ESP32_RX 24
#define PIN_RP2040_RX_ESP32_TX 25

#define HPI_SW1 12
#define HPI_SW2 14
#define HPI_SW3 1

#define BUTTON_DOWN HPI_SW1
#define BUTTON_UP HPI_SW3
#define BUTTON_CENTER HPI_SW2

#define HPI_LED_BLUE 22
#define HPI_LED_GREEN 21

#define SD_CS_PIN 16

#define   TFT_BKL_LITE    16  // Backlight control pin
#define   SD_CS_DISPLAY   TFT_BKL_LITE  // Backlight PWM pin

// Packet format for communication with OpenView

#define CES_CMDIF_PKT_START_1 0x0A
#define CES_CMDIF_PKT_START_2 0xFA
#define CES_CMDIF_TYPE_DATA 0x02
#define CES_CMDIF_PKT_STOP_1 0x00
#define CES_CMDIF_PKT_STOP 0x0B

#define CES_CMDIF_DATA_LEN_LSB 20
#define CES_CMDIF_DATA_LEN_MSB 0

#define PPG_DATA 0X00

#define MAX30205_ADDRESS1        0x49  // 8bit address converted to 7bit
#define MAX30205_ADDRESS2        0x48  // 8bit address converted to 7bit

// Registers
#define MAX30205_TEMPERATURE    0x00  //  get temperature ,Read only
#define MAX30205_CONFIGURATION  0x01  //
#define MAX30205_THYST          0x02  //
#define MAX30205_TOS            0x03  //

#define DISP_WINDOW_SIZE 512 // SAMPLE_RATE * 4

//Following pins are used for the display and defined in the User_Setup.h file
//#define TFT_MISO -1
//#define TFT_MOSI 11
//#define TFT_SCLK 10
//#define TFT_CS   29  // Chip select control pin
//#define TFT_DC   23  // Data Command control pin
//#define TFT_RST  8  // Reset pin (could connect to RST pin)



enum hpi_scr_t
{
  SCR_MAIN_MENU,
  SCR_CHARTS_ALL,
  SCR_CHARTS_ECG,
  SCR_CHARTS_PPG,
  SCR_CHARTS_RESP
};

#endif // __HPI_DEFS_H__