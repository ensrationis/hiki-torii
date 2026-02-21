#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>

// Data types
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

// Pin definitions (ESP32-C6 Insight board)
#define EPD_SCK_PIN  21
#define EPD_MOSI_PIN 20
#define EPD_CS_PIN   22
#define EPD_RST_PIN  15
#define EPD_DC_PIN   23
#define EPD_BUSY_PIN 4

#define GPIO_PIN_SET   1
#define GPIO_PIN_RESET 0

// GPIO read and write
#define DEV_Digital_Write(_pin, _value) digitalWrite(_pin, _value == 0? LOW:HIGH)
#define DEV_Digital_Read(_pin) digitalRead(_pin)

// Delay
#define DEV_Delay_ms(__xms) delay(__xms)

UBYTE DEV_Module_Init(void);
void DEV_SPI_WriteByte(UBYTE data);
UBYTE DEV_SPI_ReadByte();
void DEV_SPI_Write_nByte(UBYTE *pData, UDOUBLE len);

#endif
