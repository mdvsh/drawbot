/*
 * hx8357_stm32.h
 *
 *  Created on: Nov 5, 2024
 *      Author: madhavss
 */

#ifndef INC_HX8357_STM32_H_
#define INC_HX8357_STM32_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define LANDSCAPE

#ifdef LANDSCAPE
#define DISP_HEIGHT HX8357_WIDTH
#define DISP_WIDTH  HX8357_HEIGHT
#else
#define DISP_HEIGHT HX8357_HEIGHT
#define DISP_WIDTH  HX8357_WIDTH
#endif

#define CHAR_HEIGHT  7
#define CHAR_WIDTH   5
#define CHAR_PADDING 1

// From Adafruit Library
#define HX8357_WIDTH  320
#define HX8357_HEIGHT 480

#define HX8357_NOP       0x00
#define HX8357_SWRESET   0x01
#define HX8357_RDDID     0x04
#define HX8357_RDDST     0x09
#define HX8357_RDPOWMODE 0x0A
#define HX8357_RDMADCTL  0x0B
#define HX8357_RDCOLMOD  0x0C
#define HX8357_RDDIM     0x0D
#define HX8357_RDDSDR    0x0F
#define HX8357_SLPIN     0x10
#define HX8357_SLPOUT    0x11
#define HX8357_INVOFF    0x20
#define HX8357_INVON     0x21
#define HX8357_DISPOFF   0x28
#define HX8357_DISPON    0x29
#define HX8357_CASET     0x2A
#define HX8357_PASET     0x2B
#define HX8357_RAMWR     0x2C
#define HX8357_RAMRD     0x2E
#define HX8357_TEON      0x35
#define HX8357_TEARLINE  0x44
#define HX8357_MADCTL    0x36
#define HX8357_COLMOD    0x3A
#define HX8357_SETOSC    0xB0
#define HX8357_SETPWR1   0xB1
#define HX8357_SETRGB    0xB3
#define HX8357D_SETCOM   0xB6
#define HX8357D_SETCYC   0xB4
#define HX8357D_SETC     0xB9
#define HX8357D_SETSTBA  0xC0
#define HX8357_SETPANEL  0xCC
#define HX8357D_SETGAMMA 0xE0

// Color definitions
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

typedef enum { HX8357_PORTRAIT = 0, HX8357_LANDSCAPE = 1 } HX8357_Orientation;

// Error codes
typedef enum { HX8357_OK = 0, HX8357_ERROR = 1, HX8357_PARAM_ERROR = 3 } HX8357_Status;

typedef struct {
    SPI_HandleTypeDef *hspi; // SPI handle
    GPIO_TypeDef *cs_port;   // Chip Select port
    uint16_t cs_pin;         // Chip Select pin
    GPIO_TypeDef *dc_port;   // Data/Command port
    uint16_t dc_pin;         // Data/Command pin
    GPIO_TypeDef *rst_port;  // Reset port
    uint16_t rst_pin;        // Reset pin
    HX8357_Orientation orientation;
    uint16_t width;  // Current width based on orientation
    uint16_t height; // Current height based on orientation
} HX8357_HandleTypeDef;

void HX8357_Reset(HX8357_HandleTypeDef *display);
HX8357_Status HX8357_Init(HX8357_HandleTypeDef *display);
HX8357_Status HX8357_SetOrientation(HX8357_HandleTypeDef *display, HX8357_Orientation orientation);
HX8357_Status HX8357_FillScreen(HX8357_HandleTypeDef *display, uint16_t color);
HX8357_Status HX8357_Grid(HX8357_HandleTypeDef *display);
HX8357_Status HX8357_DrawPixel(HX8357_HandleTypeDef *display, uint16_t x, uint16_t y,
                               uint16_t color);
HX8357_Status HX8357_DrawThickPixel(HX8357_HandleTypeDef *display, uint16_t x, uint16_t y,
                                    uint16_t color, uint8_t thickness);
HX8357_Status HX8357_DrawLine(HX8357_HandleTypeDef *display, uint16_t x1, uint16_t y1, uint16_t x2,
                              uint16_t y2, uint16_t color, uint8_t thickness);
HX8357_Status HX8357_FillRect(HX8357_HandleTypeDef *display, uint16_t x, uint16_t y, uint16_t w,
                              uint16_t h, uint16_t color);
HX8357_Status HX8357_DrawText(HX8357_HandleTypeDef *display, const char *text, uint16_t x,
                              uint16_t y, uint16_t fg_color, uint16_t bg_color, uint8_t size);
void HX8357_BeginTouch(HX8357_HandleTypeDef *display);
void HX8357_EndTouch(HX8357_HandleTypeDef *display);
#endif /* INC_HX8357_STM32_H_ */
