/*
 * xbee.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Wimbazu
 */

#ifndef INC_XBEE_H_
#define INC_XBEE_H_

#include "main.h"
#include "hx8357_stm32.h"
#include <stdbool.h>

typedef struct
{
	struct
	{
		float x;
		float y;
	} cal_points[4];

    uint16_t canvas_width;
    uint16_t canvas_height;
    uint16_t display_width;
	uint16_t display_height;

	uint16_t last_display_x;
	uint16_t last_display_y;
    uint16_t last_x;
	uint16_t last_y;
	bool pen_down;
    uint8_t first_point;
} Zumo_Calibration;

void Zumo_CalibrateInit(Zumo_Calibration *zum);
void Zumo_ResetLine(Zumo_Calibration *zum);
void WriteToXbee(UART_HandleTypeDef *huart, uint16_t canvas_x, uint16_t canvas_y, bool pen_up);
void Zumo_XbeeHandler(HX8357_HandleTypeDef *display, UART_HandleTypeDef *huart, Zumo_Calibration *zum, int display_x, int display_y);
void Zumo_MapCoordinates(Zumo_Calibration *zum, uint16_t display_x, uint16_t display_y,
                          uint16_t *canvas_x, uint16_t *canvas_y);



#endif /* INC_XBEE_H_ */
