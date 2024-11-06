/*
 * hx8357_stm32.h
 *
 *  Created on: Nov 5, 2024
 *      Author: madhavss
 */

#ifndef INC_HX8357_STM32_H_
#define INC_HX8357_STM32_H_

#include "main.h"

// Display dimensions
#define HX8357_TFTWIDTH  320
#define HX8357_TFTHEIGHT 480

// Display commands
#define HX8357_NOP     0x00
#define HX8357_SWRESET 0x01
#define HX8357_RDDID   0x04
#define HX8357_RDDST   0x09
#define HX8357_RDPOWMODE  0x0A
#define HX8357_RDMADCTL  0x0B
#define HX8357_RDCOLMOD  0x0C
#define HX8357_RDDIM  0x0D
#define HX8357_RDDSDR  0x0F
#define HX8357_SLPIN   0x10
#define HX8357_SLPOUT  0x11

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *dc_port;
    uint16_t dc_pin;
} HX8357_HandleTypeDef;

void HX8357_Init(HX8357_HandleTypeDef *display);
void HX8357_WriteCommand(HX8357_HandleTypeDef *display, uint8_t cmd);
void HX8357_WriteData(HX8357_HandleTypeDef *display, uint8_t data);

#endif /* INC_HX8357_STM32_H_ */
