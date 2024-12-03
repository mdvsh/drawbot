#include "touch.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

void Zumo_CalibrateInit(Zumo_Calibration *zum)
{
	// Top Left
	zum->cal_points[0].x = 37.5;
	zum->cal_points[0].y = 39.3;

	// Top Right
	zum->cal_points[1].x = 273.7;
	zum->cal_points[1].y = 47.6;

	// Bottom Left
	zum->cal_points[2].x = 37.0;
	zum->cal_points[2].y = 363.1;

	// Bottom Right
	zum->cal_points[3].x = 277.8;
	zum->cal_points[3].y = 440.2;

	zum->last_x = 0;
	zum->last_y = 0;

    zum->canvas_width = 144;
    zum->canvas_height = 96; //quarter inch increments (24x36)

    zum->display_width = 480;
	zum->display_height = 320;

	zum->last_angle = 0;
	zum->pen_down = false;
    zum->first_point = 1;
}

void Zumo_ResetLine(Zumo_Calibration *zum) {
	zum->first_point = 1;
}

void WriteToXbee(UART_HandleTypeDef *huart, uint16_t canvas_x, uint16_t canvas_y, bool pen_up) {
    uint8_t data[5];

    data[0] = (canvas_x >> 8) & 0xFF;  // High byte of canvas_x
    data[1] = canvas_x & 0xFF;         // Low byte of canvas_x
    data[2] = (canvas_y >> 8) & 0xFF;  // High byte of canvas_y
    data[3] = canvas_y & 0xFF;         // Low byte of canvas_y
    data[4] = pen_up ? 1 : 0;          // Pen status (1 for up, 0 for down)

    HAL_UART_Transmit(huart, data, sizeof(data), HAL_MAX_DELAY);
}


void Zumo_XbeeHandler(HX8357_HandleTypeDef *display, UART_HandleTypeDef *huart, Zumo_Calibration *zum, int display_x, int display_y)
{
	const uint16_t MIN_DISTANCE = 36;     // 6 squared - for ~1.5 inch minimum spacing
	const uint16_t MAX_DISTANCE = 20000;  // ~141 squared - ~35 inches max movement
    const int MIN_ANGLE_CHANGE = 20;    // degrees
    static bool was_touching = false;

    uint16_t canvas_x, canvas_y;
    Zumo_MapCoordinates(zum, display_x, display_y, &canvas_x, &canvas_y);

    int dx = canvas_x - zum->last_x;
    int dy = canvas_y - zum->last_y;
    int squared_dist = dx*dx + dy*dy;
    bool record_point = false;

    // Calculate and round angle
    float raw_angle = atan2f(dy, dx) * 180.0f / M_PI;
    int rounded_angle = ((int)(raw_angle + 2.5f) / 5) * 5;
    if (rounded_angle < 0) rounded_angle += 360;
    if (rounded_angle >= 360) rounded_angle -= 360;

    int angle_diff = abs(rounded_angle - zum->last_angle);
    if (angle_diff > 180) angle_diff = 360 - angle_diff;

    // Simplified pen state logic
    if (zum->first_point) {
        // Starting a new line
        record_point = true;
        zum->pen_down = false;
        printf("Starting new line - Pen UP, Moving to (%d, %d)\r\n", canvas_x, canvas_y);
    }
    else if (squared_dist > MIN_DISTANCE) {
        // Continue drawing
        record_point = true;
        zum->pen_down = true;
        printf("Drawing - Pen DOWN at (%d, %d)\r\n", canvas_x, canvas_y);

        if (squared_dist > MAX_DISTANCE) {
            // Too far - start new segment
            zum->first_point = 1;
            zum->pen_down = false;
            printf("Distance too large - Starting new segment\r\n");
        }
    }

    if (record_point) {
        zum->last_angle = rounded_angle;

        // Draw blue line for pen-down movements
        if (zum->pen_down) {
            HX8357_DrawThickLine(display, zum->last_display_x, zum->last_display_y,
                                display_x, display_y, BLUE, 2);
        }

        // Send command to robot with pen state
        WriteToXbee(huart, canvas_x, canvas_y, !zum->pen_down);

        zum->last_display_x = display_x;
        zum->last_display_y = display_y;
        zum->last_x = canvas_x;
        zum->last_y = canvas_y;

        if (!zum->pen_down) {
            zum->first_point = 0;  // Clear first_point after recording movement
        }
    }
}

void Zumo_MapCoordinates(Zumo_Calibration *zum, uint16_t display_x, uint16_t display_y,
                            uint16_t *canvas_x, uint16_t *canvas_y)
{
    // Calculate scaling factors
    float x_scale = (float)zum->canvas_width / (float)zum->display_width;
    float y_scale = (float)zum->canvas_height / (float)zum->display_height;

    // Map display_y to canvas_x (for left-to-right movement)
    *canvas_x = (uint16_t)(display_y * x_scale);

    // Map display_x to canvas_y (for bottom-to-top movement)
    *canvas_y = (uint16_t)(display_x * y_scale);

    // Bounds checking for X coordinate (left-right bounds)
    if (*canvas_x >= zum->canvas_width)
        *canvas_x = zum->canvas_width - 1;
    if (*canvas_x < 0)
        *canvas_x = 0;

    // Bounds checking for Y coordinate (bottom-top bounds)
    if (*canvas_y >= zum->canvas_height)
        *canvas_y = zum->canvas_height - 1;
    if (*canvas_y < 0)
        *canvas_y = 0;
}
