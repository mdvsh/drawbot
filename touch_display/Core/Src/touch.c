/*
 * touch.c
 *
 *  Created on: Nov 7, 2024
 *      Author: madhavss
 */

#include "touch.h"
#include "xbee.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#define ADC_MAX_VALUE ((1 << 12) - 1)

static ADC_HandleTypeDef *touch_adc;

// Private function prototypes
static uint16_t Touch_ReadX(void);
static uint16_t Touch_ReadY(void);
static float Touch_ReadZ(void);
static uint16_t Get_Median(uint16_t arr[], uint8_t size);
static void Configure_ADC_Channel(uint32_t channel);

void Touch_Init(ADC_HandleTypeDef *hadc)
{
    touch_adc = hadc;

    // Initial pin configuration
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO Clocks if not already done in CubeMX
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Configure all pins as analog initially
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = X_POS_PIN;
    HAL_GPIO_Init(X_POS_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = Y_POS_PIN;
    HAL_GPIO_Init(Y_POS_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = X_NEG_PIN;
    HAL_GPIO_Init(X_NEG_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = Y_NEG_PIN;
    HAL_GPIO_Init(Y_NEG_PORT, &GPIO_InitStruct);
}

TouchEvent Touch_Read(void)
{
    TouchEvent event = {0};
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();

    // Reduce settle time for more frequent sampling
    if (current_time - last_time < 5)
    {
        return event;
    }
    last_time = current_time;

    // Check pressure
    event.pressure = Touch_ReadZ();

    if (event.pressure < PRESSURE_THRESHOLD)
    {
        uint16_t x_samples[MAX_GC_SAMPLES];
        uint16_t y_samples[MAX_GC_SAMPLES];

        // Take samples
        for (uint8_t i = 0; i < MAX_GC_SAMPLES; i++)
        {
            x_samples[i] = Touch_ReadX();
            y_samples[i] = Touch_ReadY();
        }

        // Get median values
        event.x = Get_Median(x_samples, MAX_GC_SAMPLES);
        event.y = Get_Median(y_samples, MAX_GC_SAMPLES);
        event.touched = 1;
    }

    return event;
}
static uint16_t Touch_ReadX(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // X+ and X- as outputs to create voltage gradient
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = X_POS_PIN;
    HAL_GPIO_Init(X_POS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = X_NEG_PIN;
    HAL_GPIO_Init(X_NEG_PORT, &GPIO_InitStruct);

    // Create voltage gradient
    HAL_GPIO_WritePin(X_POS_PORT, X_POS_PIN, GPIO_PIN_SET);   // X+ HIGH
    HAL_GPIO_WritePin(X_NEG_PORT, X_NEG_PIN, GPIO_PIN_RESET); // X- LOW

    // Y- as floating input
    GPIO_InitStruct.Pin = Y_NEG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(Y_NEG_PORT, &GPIO_InitStruct);

    // Y+ as ADC input
    GPIO_InitStruct.Pin = Y_POS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(Y_POS_PORT, &GPIO_InitStruct);

    Configure_ADC_Channel(Y_POS_ADC_CHANNEL);

    HAL_Delay(SETTLE_TIME_MS);

    HAL_ADC_Start(touch_adc);
    HAL_ADC_PollForConversion(touch_adc, 100);
    uint32_t adc_val = ADC_MAX_VALUE - HAL_ADC_GetValue(touch_adc);

    return (uint16_t)((float)adc_val / ADC_MAX_VALUE * SCREEN_WIDTH);
}

static uint16_t Touch_ReadY(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Y+ and Y- as outputs to create voltage gradient
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = Y_POS_PIN;
    HAL_GPIO_Init(Y_POS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Y_NEG_PIN;
    HAL_GPIO_Init(Y_NEG_PORT, &GPIO_InitStruct);

    // Create voltage gradient
    HAL_GPIO_WritePin(Y_POS_PORT, Y_POS_PIN, GPIO_PIN_SET);   // Y+ HIGH
    HAL_GPIO_WritePin(Y_NEG_PORT, Y_NEG_PIN, GPIO_PIN_RESET); // Y- LOW

    // X- as floating input
    GPIO_InitStruct.Pin = X_NEG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(X_NEG_PORT, &GPIO_InitStruct);

    // X+ as ADC input
    GPIO_InitStruct.Pin = X_POS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(X_POS_PORT, &GPIO_InitStruct);

    Configure_ADC_Channel(X_POS_ADC_CHANNEL);

    HAL_Delay(SETTLE_TIME_MS);

    HAL_ADC_Start(touch_adc);
    HAL_ADC_PollForConversion(touch_adc, 100);
    uint32_t adc_val = ADC_MAX_VALUE - HAL_ADC_GetValue(touch_adc);

    return (uint16_t)((float)adc_val / ADC_MAX_VALUE * SCREEN_HEIGHT);
}

static float Touch_ReadZ(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // X+ as ADC input
    GPIO_InitStruct.Pin = X_POS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(X_POS_PORT, &GPIO_InitStruct);

    // Y- drive HIGH
    GPIO_InitStruct.Pin = Y_NEG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(Y_NEG_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(Y_NEG_PORT, Y_NEG_PIN, GPIO_PIN_SET);

    // X- drive LOW
    GPIO_InitStruct.Pin = X_NEG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(X_NEG_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(X_NEG_PORT, X_NEG_PIN, GPIO_PIN_RESET);

    // Y+ as floating input
    GPIO_InitStruct.Pin = Y_POS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(Y_POS_PORT, &GPIO_InitStruct);

    HAL_Delay(SETTLE_TIME_MS);

    // Read Z1 (using X+)
    Configure_ADC_Channel(X_POS_ADC_CHANNEL);
    HAL_ADC_Start(touch_adc);
    HAL_ADC_PollForConversion(touch_adc, 100);
    uint32_t z1 = ADC_MAX_VALUE - HAL_ADC_GetValue(touch_adc);

    // Change Y+ to ADC input for second reading
    GPIO_InitStruct.Pin = Y_POS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(Y_POS_PORT, &GPIO_InitStruct);

    // Read Z2 (using Y+)
    Configure_ADC_Channel(Y_POS_ADC_CHANNEL);
    HAL_ADC_Start(touch_adc);
    HAL_ADC_PollForConversion(touch_adc, 100);
    uint32_t z2 = ADC_MAX_VALUE - HAL_ADC_GetValue(touch_adc);

    // Calculate pressure
    if (z1 > z2)
    {
        return (float)(z1 - z2) / ADC_MAX_VALUE;
    }
    else
    {
        return (float)(z2 - z1) / ADC_MAX_VALUE;
    }
}

static void Configure_ADC_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5; // Increased sampling time
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(touch_adc, &sConfig);
}

static uint16_t Get_Median(uint16_t arr[], uint8_t size)
{
    // Simple bubble sort
    for (uint8_t i = 0; i < size - 1; i++)
    {
        for (uint8_t j = 0; j < size - i - 1; j++)
        {
            if (arr[j] > arr[j + 1])
            {
                uint16_t temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
    return arr[size / 2];
}

void Touch_CalibrateInit(Touch_Calibration *cal)
{
    // Top Left
    cal->cal_points[0].x = 37.5;
    cal->cal_points[0].y = 39.3;

    // Top Right
    cal->cal_points[1].x = 273.7;
    cal->cal_points[1].y = 47.6;

    // Bottom Left
    cal->cal_points[2].x = 37.0;
    cal->cal_points[2].y = 363.1;

    // Bottom Right
    cal->cal_points[3].x = 277.8;
    cal->cal_points[3].y = 440.2;

    cal->display_width = 320;
    cal->display_height = 480;
    cal->first_point = 1;
}

void Touch_MapCoordinates(Touch_Calibration *cal, uint16_t touch_x, uint16_t touch_y,
                          uint16_t *display_x, uint16_t *display_y)
{
    // X mapping (simpler as it's more linear)
    float x_left = (cal->cal_points[0].x + cal->cal_points[2].x) / 2;  // Average left x
    float x_right = (cal->cal_points[1].x + cal->cal_points[3].x) / 2; // Average right x

    float x_scale = (float)cal->display_width / (x_right - x_left);
    *display_x = (touch_x - x_left) * x_scale;

    // Bound X coordinates
    if (*display_x >= cal->display_width)
        *display_x = cal->display_width - 1;
    if (*display_x < 0)
        *display_x = 0;

    // Y mapping (need to handle inversion and potential skew)
    float y_top = (cal->cal_points[0].y + cal->cal_points[1].y) / 2;    // Average top y
    float y_bottom = (cal->cal_points[2].y + cal->cal_points[3].y) / 2; // Average bottom y

    float y_scale = (float)cal->display_height / (y_bottom - y_top);
    *display_y = (touch_y - y_top) * y_scale;

    // Bound Y coordinates
    if (*display_y >= cal->display_height)
        *display_y = cal->display_height - 1;
    if (*display_y < 0)
        *display_y = 0;
}

void Touch_ResetLine(Touch_Calibration *cal) {
	cal->first_point = 1;
}

bool Phantom_point(uint16_t display_x, uint16_t display_y) {
	return (display_x <= 2 || display_y <= 2);
}

void Touch_DrawHandler(HX8357_HandleTypeDef *display, Touch_Calibration *cal, Zumo_Calibration *zum, UART_HandleTypeDef *huart, TouchEvent event)
{
    const uint16_t MAX_DISTANCE = 12800;
    const uint16_t MIN_DISTANCE = 90;
    bool reset_point = true;

    if (event.touched && event.pressure > 0.1)
    {
        uint16_t display_x, display_y;
        Touch_MapCoordinates(cal, event.x, event.y, &display_x, &display_y);

        HX8357_BeginTouch(display);

        if (Phantom_point(display_x, display_y)) {
        	return;
        }

        if (!cal->first_point)
        {
            int dx = abs(display_x - cal->last_x);
            int dy = abs(display_y - cal->last_y);
            int squared_dist = (dx*dy + dy*dy);

            if (squared_dist > MAX_DISTANCE) {
            	Touch_ResetLine(cal);
            }
            else if ((squared_dist > MIN_DISTANCE)) {
//            	HX8357_DrawThickLine(display, cal->last_x, cal->last_y, display_x, display_y, RED, 2);
            	HX8357_DrawLine(display, cal->last_x, cal->last_y, display_x, display_y, RED);
            }
            else {
            	reset_point = false;
            }
        }
        HX8357_EndTouch(display);

        Zumo_XbeeHandler(display, huart, zum, display_x, display_y);

        if (reset_point){
        	cal->last_x = display_x;
			cal->last_y = display_y;
			cal->first_point = 0;
        }
    }
}

void Draw_ClearButton(HX8357_HandleTypeDef *display, ClearButton *btn) {
    HX8357_BeginTouch(display);
    HX8357_FillRect(display, btn->x, btn->y, btn->size, btn->size, btn->color);

    // Draw X symbol in border_color
    uint16_t padding = btn->size/4;
    HX8357_DrawThickLine(display, btn->x + padding, btn->y + padding,
                        btn->x + btn->size - padding, btn->y + btn->size - padding,
                        btn->border_color, 2);
    HX8357_DrawThickLine(display, btn->x + btn->size - padding, btn->y + padding,
                        btn->x + padding, btn->y + btn->size - padding,
                        btn->border_color, 2);
    HX8357_EndTouch(display);
}

bool Is_ClearButtonPressed(ClearButton *btn, uint16_t x, uint16_t y) {
    return (x >= btn->x && x <= (btn->x + btn->size) &&
            y >= btn->y && y <= (btn->y + btn->size));
}

