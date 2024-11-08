/*
 * touch.c
 *
 *  Created on: Nov 7, 2024
 *      Author: madhavss
 */


#include "touch.h"
#include <stdlib.h>
#include <stdio.h>

// Global ADC & DMA variables
static ADC_HandleTypeDef* touch_adc;
static volatile bool adc_complete = false;
static uint32_t adc_buffer[ADC_BUFFER_SIZE];
static ADC_ChannelConfTypeDef sConfig = {0};

// ADC conversion complete callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc == touch_adc) {
        adc_complete = true;
    }
}

// Helper function for median calculation
static int compare(const void* a, const void* b) {
    return (*(uint32_t*)a - *(uint32_t*)b);
}

// Calculate median with noise rejection
static float median(uint32_t arr[], int n) {
    qsort(arr, n, sizeof(uint32_t), compare);

    if (arr[n-1] - arr[0] > TOUCH_VAR_THRESHOLD) {
        return 0;
    }

    if (n % 2 == 0) {
        return ((float)arr[n/2-1] + (float)arr[n/2]) / 2.0;
    }
    return (float)arr[n/2];
}

void Touch_Init(ADC_HandleTypeDef* hadc) {
    touch_adc = hadc;

    // Setup ADC config common parameters
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
}

static float Touch_ReadAxisDMA(uint32_t channel) {
    uint32_t samples[TOUCH_SAMPLES] = {0};

    sConfig.Channel = channel;
    if (HAL_ADC_ConfigChannel(touch_adc, &sConfig) != HAL_OK) {
        return 0;
    }

    for (int i = 0; i < TOUCH_SAMPLES; i++) {
        adc_complete = false;

        if (HAL_ADC_Start_DMA(touch_adc, adc_buffer, ADC_BUFFER_SIZE) != HAL_OK) {
            return 0;
        }

        uint32_t start = HAL_GetTick();
        while (!adc_complete && (HAL_GetTick() - start) < DMA_TIMEOUT);

        if (adc_complete) {
            samples[i] = adc_buffer[0];
        }

        HAL_ADC_Stop_DMA(touch_adc);
        HAL_Delay(1);
    }

    return median(samples, TOUCH_SAMPLES);
}

void Touch_InitX(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Setup Y+ (HIGH) and Y- (LOW) for voltage gradient
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin = Y_POS_PIN;
    HAL_GPIO_Init(Y_POS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(Y_POS_PORT, Y_POS_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Pin = Y_NEG_PIN;
    HAL_GPIO_Init(Y_NEG_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(Y_NEG_PORT, Y_NEG_PIN, GPIO_PIN_RESET);

    // X- as floating input
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = X_NEG_PIN;
    HAL_GPIO_Init(X_NEG_PORT, &GPIO_InitStruct);

    // X+ as ADC input
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pin = X_POS_PIN;
    HAL_GPIO_Init(X_POS_PORT, &GPIO_InitStruct);
}

void Touch_InitY(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Setup X+ (HIGH) and X- (LOW) for voltage gradient
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin = X_POS_PIN;
    HAL_GPIO_Init(X_POS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(X_POS_PORT, X_POS_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Pin = X_NEG_PIN;
    HAL_GPIO_Init(X_NEG_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(X_NEG_PORT, X_NEG_PIN, GPIO_PIN_RESET);

    // Y- as floating input
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = Y_NEG_PIN;
    HAL_GPIO_Init(Y_NEG_PORT, &GPIO_InitStruct);

    // Y+ as ADC input
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pin = Y_POS_PIN;
    HAL_GPIO_Init(Y_POS_PORT, &GPIO_InitStruct);
}

// TOFIX
TouchPoint Touch_ReadPoint(void) {
    TouchPoint point = {0};

    Touch_InitX();
    HAL_Delay(1);
    point.x_raw = Touch_ReadAxisDMA(X_POS_ADC_CHANNEL);

    Touch_InitY();
    HAL_Delay(1);
    point.y_raw = Touch_ReadAxisDMA(Y_POS_ADC_CHANNEL);

    printf("\r\nRaw X: %.1f, Y: %.1f", point.x_raw, point.y_raw);

    // Validate readings within expected range
    if (point.x_raw >= X_MIN && point.x_raw <= X_MAX &&
        point.y_raw >= Y_MIN && point.y_raw <= Y_MAX) {

        point.x = (uint16_t)((point.x_raw - X_MIN) * (SCREEN_WIDTH - 1) / (X_MAX - X_MIN));
        point.y = (uint16_t)((point.y_raw - Y_MIN) * (SCREEN_HEIGHT - 1) / (Y_MAX - Y_MIN));

        // Optional axis inversion if needed
        // point.x = SCREEN_WIDTH - 1 - point.x;
        // point.y = SCREEN_HEIGHT - 1 - point.y;

        printf("\r\nMapped X: %d, Y: %d", point.x, point.y);
        point.valid = true;
    }

    return point;
}
