/*
 * hx8357_stm32.c
 *
 *  Created on: Nov 5, 2024
 *      Author: madhavss
 */

#include "hx8357_stm32.h"

static void HX8357_Select(HX8357_HandleTypeDef *display) {
    HAL_GPIO_WritePin(display->cs_port, display->cs_pin, GPIO_PIN_RESET);
}

static void HX8357_Unselect(HX8357_HandleTypeDef *display) {
    HAL_GPIO_WritePin(display->cs_port, display->cs_pin, GPIO_PIN_SET);
}

static void HX8357_SetCommand(HX8357_HandleTypeDef *display) {
    HAL_GPIO_WritePin(display->dc_port, display->dc_pin, GPIO_PIN_RESET);
}

static void HX8357_SetData(HX8357_HandleTypeDef *display) {
    HAL_GPIO_WritePin(display->dc_port, display->dc_pin, GPIO_PIN_SET);
}

void HX8357_WriteCommand(HX8357_HandleTypeDef *display, uint8_t cmd) {
    HX8357_SetCommand(display);
    HX8357_Select(display);
    HAL_SPI_Transmit(display->hspi, &cmd, 1, HAL_MAX_DELAY);
    HX8357_Unselect(display);
}

void HX8357_WriteData(HX8357_HandleTypeDef *display, uint8_t data) {
    HX8357_SetData(display);
    HX8357_Select(display);
    HAL_SPI_Transmit(display->hspi, &data, 1, HAL_MAX_DELAY);
    HX8357_Unselect(display);
}

void HX8357_Init(HX8357_HandleTypeDef *display) {
    // Hardware reset sequence
    HAL_Delay(10);

    // Software reset
    HX8357_WriteCommand(display, HX8357_SWRESET);
    HAL_Delay(100);

    // Exit sleep
    HX8357_WriteCommand(display, HX8357_SLPOUT);
    HAL_Delay(120);

    // Basic display initialization commands will go here
    // (Will expand later)
}
