#include "touch.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

void Zumo_CalibrateInit(Zumo_Calibration *zum)
{
    zum->last_x = 0;
    zum->last_y = 0;

    zum->canvas_width = 60;
    zum->canvas_height = 35;

    zum->display_width = 480;
    zum->display_height = 320;

    zum->pen_down = false;
    zum->first_point = 1;
}

void Zumo_ResetLine(Zumo_Calibration *zum)
{
    zum->first_point = 1;
}

void WriteToXbee(UART_HandleTypeDef *huart, uint16_t canvas_x, uint16_t canvas_y, bool pen_up)
{
    uint8_t data[5];
    data[0] = (canvas_x >> 8) & 0xFF; // High byte of canvas_x
    data[1] = canvas_x & 0xFF;        // Low byte of canvas_x
    data[2] = (canvas_y >> 8) & 0xFF; // High byte of canvas_y
    data[3] = canvas_y & 0xFF;        // Low byte of canvas_y
    data[4] = pen_up ? 1 : 0;         // Pen status (1 for up, 0 for down)

    HAL_UART_Transmit(huart, data, sizeof(data), HAL_MAX_DELAY);
}

void Zumo_XbeeHandler(HX8357_HandleTypeDef *display, UART_HandleTypeDef *huart,
                      Zumo_Calibration *zum, int display_x, int display_y)
{
    const uint16_t MIN_DISTANCE = 100;  // 10 squared - for 10 cm minimum spacing
    const uint16_t MAX_DISTANCE = 1600; // 40 squared - 40 cm max movement

    uint16_t canvas_x, canvas_y;
    Zumo_MapCoordinates(zum, display_x, display_y, &canvas_x, &canvas_y);

    int dx = canvas_x - zum->last_x;
    int dy = canvas_y - zum->last_y;
    int squared_dist = dx * dx + dy * dy;
    bool record_point = false;

    // Simplified pen state logic
    if (zum->first_point) {
        // Starting a new line
        record_point = true;
        zum->pen_down = false;
        printf("Starting new line - Pen UP, Moving to (%d, %d)\r\n", canvas_x, canvas_y);
    } else if (squared_dist > MIN_DISTANCE) {
        // Continue drawing
        record_point = true;
        zum->pen_down = true;
        printf("Drawing - Pen DOWN at (%d, %d)\r\n", canvas_x, canvas_y);

        if (squared_dist > MAX_DISTANCE) {
            // Too far - start new segment
            zum->pen_down = false;
            zum->first_point = 1;
            printf("Distance too large - Starting new segment\r\n");
        }
    }

    if (record_point) {
        // Draw red line for pen-down movements
        if (zum->pen_down) {
            HX8357_DrawLine(display, zum->last_display_x, zum->last_display_y, display_x, display_y,
                            RED, 2);
        }

        // Send command to robot with pen state
        WriteToXbee(huart, canvas_x, canvas_y, !zum->pen_down);

        zum->last_display_x = display_x;
        zum->last_display_y = display_y;
        zum->last_x = canvas_x;
        zum->last_y = canvas_y;

        if (!zum->pen_down) {
            zum->first_point = 0; // Clear first_point after recording movement
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
