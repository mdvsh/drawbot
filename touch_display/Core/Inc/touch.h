/*
 * touch.h
 *
 *  Created on: Nov 7, 2024
 *      Author: madhavss
 */

#ifndef INC_TOUCH_H_
#define INC_TOUCH_H_

#include "main.h"
#include "xbee.h"
#include "hx8357_stm32.h"

#define X_POS_PORT GPIOC
#define X_POS_PIN GPIO_PIN_0
#define X_POS_ADC_CHANNEL ADC_CHANNEL_1
#define Y_POS_PORT GPIOA
#define Y_POS_PIN GPIO_PIN_3
#define Y_POS_ADC_CHANNEL ADC_CHANNEL_8
#define X_NEG_PORT GPIOC
#define X_NEG_PIN GPIO_PIN_1
#define Y_NEG_PORT GPIOC
#define Y_NEG_PIN GPIO_PIN_3

#define MAX_GC_SAMPLES 11
#define PRESSURE_THRESHOLD 0.9f
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 480
#define SETTLE_TIME_MS 2

// Touch event structure
typedef struct
{
    uint16_t x;
    uint16_t y;
    float pressure;
    uint8_t touched;
} TouchEvent;

typedef struct
{
    struct
    {
        float x;
        float y;
    } cal_points[4];

    uint16_t display_width;
    uint16_t display_height;
    uint16_t last_x;
    uint16_t last_y;
    uint8_t first_point;
} Touch_Calibration;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t size;
    uint16_t color;
    uint16_t border_color;
} ClearButton;

// Function prototypes
void Touch_Init(ADC_HandleTypeDef *hadc);
TouchEvent Touch_Read(void);

void Touch_CalibrateInit(Touch_Calibration *cal);
void Touch_MapCoordinates(Touch_Calibration *cal, uint16_t touch_x, uint16_t touch_y,
                          uint16_t *display_x, uint16_t *display_y);
void Touch_DrawHandler(HX8357_HandleTypeDef *display, Touch_Calibration *cal, Zumo_Calibration *zum, UART_HandleTypeDef *huart, TouchEvent event);
void Touch_ResetLine(Touch_Calibration *cal);
void Draw_ClearButton(HX8357_HandleTypeDef *display, ClearButton *btn);
bool Is_ClearButtonPressed(ClearButton *btn, uint16_t x, uint16_t y);
bool Phantom_point(uint16_t display_x, uint16_t display_y);
#endif /* INC_TOUCH_H_ */
