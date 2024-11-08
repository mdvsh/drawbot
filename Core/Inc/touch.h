/*
 * touch2.h
 *
 *  Created on: Nov 7, 2024
 *      Author: madhavss
 */

#ifndef INC_TOUCH_H_
#define INC_TOUCH_H_

#include "stm32l4xx_hal.h"
#include <stdbool.h>

// Pin definitions
#define X_POS_PORT      GPIOC
#define X_POS_PIN       GPIO_PIN_0
#define X_POS_ADC_CHANNEL ADC_CHANNEL_1

#define Y_POS_PORT      GPIOA
#define Y_POS_PIN       GPIO_PIN_3
#define Y_POS_ADC_CHANNEL ADC_CHANNEL_8

#define X_NEG_PORT      GPIOC
#define X_NEG_PIN       GPIO_PIN_1

#define Y_NEG_PORT      GPIOC
#define Y_NEG_PIN       GPIO_PIN_3

// ADC and filtering configuration
#define ADC_BUFFER_SIZE     2
#define TOUCH_SAMPLES       10
#define TOUCH_VAR_THRESHOLD 50
#define DMA_TIMEOUT        10

// Screen dimensions
#define SCREEN_WIDTH     480
#define SCREEN_HEIGHT    320

// Touch calibration values, need to tune smh
#define X_MIN           300
#define X_MAX           3500
#define Y_MIN           900
#define Y_MAX           1300

typedef struct {
    float x_raw;
    float y_raw;
    uint16_t x;
    uint16_t y;
    bool valid;
} TouchPoint;

void Touch_Init(ADC_HandleTypeDef* hadc);
TouchPoint Touch_ReadPoint(void);
void Touch_InitX(void);
void Touch_InitY(void);

#endif /* INC_TOUCH_H_ */
