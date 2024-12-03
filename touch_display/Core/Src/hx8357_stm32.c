/*
 * hx8357_stm32.c
 *
 *  Created on: Nov 5, 2024
 *      Author: madhavss
 */

#include "hx8357_stm32.h"
#include <string.h>
#include <math.h>

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#define SWAP_UINT16(a, b) \
    {                     \
        uint16_t t = a;   \
        a = b;            \
        b = t;            \
    }
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// Private functions declarations
static void HX8357_Select(HX8357_HandleTypeDef *display);
static void HX8357_Unselect(HX8357_HandleTypeDef *display);
static void HX8357_WriteCommand(HX8357_HandleTypeDef *display, uint8_t cmd);
static void HX8357_WriteData(HX8357_HandleTypeDef *display, const uint8_t *data, uint16_t size);
static HX8357_Status HX8357_SetWindow(HX8357_HandleTypeDef *display, uint16_t x1, uint16_t y1,
                                      uint16_t x2, uint16_t y2);

static const uint8_t init_seq[] = {
    HX8357_SWRESET,
    0x80 + 100 / 5, // Soft reset, then delay 10 ms
    HX8357D_SETC,
    3,
    0xFF,
    0x83,
    0x57,
    0xFF,
    0x80 + 500 / 5, // No command, just delay 300 ms
    HX8357_SETRGB,
    4,
    0x80,
    0x00,
    0x06,
    0x06, // 0x80 enables SDO pin (0x00 disables)
    HX8357D_SETCOM,
    1,
    0x25, // -1.52V
    HX8357_SETOSC,
    1,
    0x68, // Normal mode 70Hz, Idle mode 55 Hz
    HX8357_SETPANEL,
    1,
    0x05, // BGR, Gate direction swapped
    HX8357_SETPWR1,
    6,
    0x00, // Not deep standby
    0x15, // BT
    0x1C, // VSPR
    0x1C, // VSNR
    0x83, // AP
    0xAA, // FS
    HX8357D_SETSTBA,
    6,
    0x50, // OPON normal
    0x50, // OPON idle
    0x01, // STBA
    0x3C, // STBA
    0x1E, // STBA
    0x08, // GEN
    HX8357D_SETCYC,
    7,
    0x02, // NW 0x02
    0x40, // RTN
    0x00, // DIV
    0x2A, // DUM
    0x2A, // DUM
    0x0D, // GDON
    0x78, // GDOFF
    HX8357D_SETGAMMA,
    34,
    0x02,
    0x0A,
    0x11,
    0x1d,
    0x23,
    0x35,
    0x41,
    0x4b,
    0x4b,
    0x42,
    0x3A,
    0x27,
    0x1B,
    0x08,
    0x09,
    0x03,
    0x02,
    0x0A,
    0x11,
    0x1d,
    0x23,
    0x35,
    0x41,
    0x4b,
    0x4b,
    0x42,
    0x3A,
    0x27,
    0x1B,
    0x08,
    0x09,
    0x03,
    0x00,
    0x01,
    HX8357_COLMOD,
    1,
    0x55, // 16 bit
    HX8357_MADCTL,
    1,
    0xC0,
    HX8357_TEON,
    1,
    0x00, // TW off
    HX8357_TEARLINE,
    2,
    0x00,
    0x02,
    HX8357_SLPOUT,
    0x80 + 150 / 5, // Exit Sleep, then delay 150 ms
    HX8357_DISPON,
    0x80 + 50 / 5, // Main screen turn on, delay 50 ms
    0,             // END OF COMMAND LIST
};

void HX8357_Reset(HX8357_HandleTypeDef *display) {
    HAL_GPIO_WritePin(display->rst_port, display->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(10);  // Hold reset for 10ms
    HAL_GPIO_WritePin(display->rst_port, display->rst_pin, GPIO_PIN_SET);
    HAL_Delay(150);  // Wait for 150ms after reset
}

static void HX8357_Select(HX8357_HandleTypeDef *display)
{
    HAL_GPIO_WritePin(display->cs_port, display->cs_pin, GPIO_PIN_RESET);
}

static void HX8357_Unselect(HX8357_HandleTypeDef *display)
{
    HAL_GPIO_WritePin(display->cs_port, display->cs_pin, GPIO_PIN_SET);
}

static void HX8357_WriteCommand(HX8357_HandleTypeDef *display, uint8_t cmd)
{
    HAL_GPIO_WritePin(display->dc_port, display->dc_pin, GPIO_PIN_RESET); // Command mode
    HAL_SPI_Transmit(display->hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(display->dc_port, display->dc_pin, GPIO_PIN_SET); // Back to data mode
}

static void HX8357_WriteData(HX8357_HandleTypeDef *display, const uint8_t *data, uint16_t size)
{
    HAL_SPI_Transmit(display->hspi, (uint8_t *)data, size, HAL_MAX_DELAY);
}

static void HX8357_WriteDataWord(HX8357_HandleTypeDef *display, uint16_t data)
{
    uint8_t data_bytes[2] = {(data >> 8) & 0xFF, data & 0xFF};
    HX8357_WriteData(display, data_bytes, 2);
}

static HX8357_Status HX8357_SetWindow(HX8357_HandleTypeDef *display, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    HX8357_Select(display);

    // Set column address
    HX8357_WriteCommand(display, HX8357_CASET);
    HX8357_WriteDataWord(display, x1);
    HX8357_WriteDataWord(display, x2);

    // Set row address
    HX8357_WriteCommand(display, HX8357_PASET);
    HX8357_WriteDataWord(display, y1);
    HX8357_WriteDataWord(display, y2);

    // Begin writing
    HX8357_WriteCommand(display, HX8357_RAMWR); // why write ?

    return HX8357_OK;
}

// Public function implementations
HX8357_Status HX8357_Init(HX8357_HandleTypeDef *display)
{
    if (!display || !display->hspi)
        return HX8357_PARAM_ERROR;

    // Hardware reset first
    HX8357_Reset(display);

    // Initial pin states
    HX8357_Unselect(display);
    HAL_GPIO_WritePin(display->dc_port, display->dc_pin, GPIO_PIN_SET);

    // Process initialization sequence
    const uint8_t *addr = init_seq;
    uint8_t cmd, x, numArgs;

    while ((cmd = *(addr++)) > 0)
    { // '0' command ends list
        x = *(addr++);
        numArgs = x & 0x7F;
        if (cmd != 0xFF)
        { // '255' is ignored
            if (x & 0x80)
            { // If high bit set, numArgs is a delay time
                HX8357_Select(display);
                HX8357_WriteCommand(display, cmd);
                HX8357_Unselect(display);
            }
            else
            {
                HX8357_Select(display);
                HX8357_WriteCommand(display, cmd);
                HX8357_WriteData(display, addr, numArgs);
                HX8357_Unselect(display);
                addr += numArgs;
            }
        }
        if (x & 0x80)
        {                           // If high bit set...
            HAL_Delay(numArgs * 5); // numArgs is actually a delay time (5ms units)
        }
    }

    // Update display dimensions based on orientation
    if (display->orientation != HX8357_LANDSCAPE)
    {
        display->width = HX8357_HEIGHT;
        display->height = HX8357_WIDTH;
    }
    else
    {
        display->width = HX8357_WIDTH;
        display->height = HX8357_HEIGHT;
    }

    return HX8357_OK;
}

HX8357_Status HX8357_FillScreen(HX8357_HandleTypeDef *display, uint16_t color)
{
    return HX8357_FillRect(display, 0, 0, display->width, display->height, color);
}

HX8357_Status HX8357_FillRect(HX8357_HandleTypeDef *display, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // Clip to display bounds
    if (x >= display->width || y >= display->height)
        return HX8357_OK;
    if ((x + w) > display->width)
        w = display->width - x;
    if ((y + h) > display->height)
        h = display->height - y;

    HX8357_SetWindow(display, x, y, x + w - 1, y + h - 1);

    // Prepare color bytes
    uint8_t color_high = (color >> 8) & 0xFF;
    uint8_t color_low = color & 0xFF;

    // Calculate total number of pixels
    uint32_t total_pixels = w * h;

    // Fill the rectangle
    for (uint32_t i = 0; i < total_pixels; i++)
    {
        HX8357_WriteData(display, &color_high, 1);
        HX8357_WriteData(display, &color_low, 1);
    }

    HX8357_Unselect(display);
    return HX8357_OK;
}

HX8357_Status HX8357_Grid(HX8357_HandleTypeDef *display)
{
    HX8357_FillScreen(display, BLACK);

    // Calculate a grid increment that fits both width and height
    uint32_t grid_inc_x = display->width / 10;  // Tentative grid increment based on width
    uint32_t grid_inc_y = display->height / 10; // Tentative grid increment based on height

    // Make sure both grid increments are as close as possible while ensuring full screen coverage
    uint32_t grid_inc = min(grid_inc_x, grid_inc_y); // Take the smaller of the two

    // Calculate the number of vertical grid lines based on grid_inc
    uint32_t num_grid_lines_x = ceil((float)display->width / grid_inc);
    uint32_t num_grid_lines_y = ceil((float)display->height / grid_inc);

    // Adjust increments to ensure the grid fills the screen
    grid_inc = min(display->width / num_grid_lines_x, display->height / num_grid_lines_y);

    // Draw vertical grid lines (spacing based on grid_inc)
    for (int x = 0; x < display->width; x += grid_inc)
    {
        HX8357_DrawLine(display, x, 0, x, display->height, WHITE);
    }

    // Draw horizontal grid lines (spacing based on grid_inc)
    for (int y = 0; y < display->height; y += grid_inc)
    {
        HX8357_DrawLine(display, 0, y, display->width, y, WHITE);
    }

    HX8357_DrawPixel(display, 3, 3, GREEN);
    HX8357_DrawPixel(display, display->width - 3, 3, YELLOW);

    return HX8357_OK;
}

// Bresenham's line algorithm
HX8357_Status HX8357_DrawLine(HX8357_HandleTypeDef *display, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    int dx = abs(x2 - x1);       // Difference in x
    int dy = abs(y2 - y1);       // Difference in y
    int sx = (x1 < x2) ? 1 : -1; // Step direction for x
    int sy = (y1 < y2) ? 1 : -1; // Step direction for y
    int err = dx - dy;           // Error term for the algorithm

    while (true)
    {
        // Draw pixel at current position
        HX8357_DrawPixel(display, x1, y1, color);

        // If we reached the destination point, break the loop
        if (x1 == x2 && y1 == y2)
            break;

        // Calculate the error term
        int e2 = err * 2;

        // Move in the x direction if the error term allows it
        if (e2 > -dy)
        {
            err -= dy;
            x1 += sx;
        }

        // Move in the y direction if the error term allows it
        if (e2 < dx)
        {
            err += dx;
            y1 += sy;
        }
    }

    return HX8357_OK;
}

HX8357_Status HX8357_DrawPixel(HX8357_HandleTypeDef *display, uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= display->width || y >= display->height)
        return HX8357_PARAM_ERROR;

    HX8357_SetWindow(display, x, y, x, y);

    HX8357_WriteDataWord(display, color);

    HX8357_Unselect(display);
    return HX8357_OK;
}

HX8357_Status HX8357_DrawThickPixel(HX8357_HandleTypeDef *display, uint16_t x, uint16_t y, uint16_t color, uint8_t thickness)
{
    // Draw a filled circle or square for thicker points
    for (int16_t i = -thickness / 2; i <= thickness / 2; i++)
    {
        for (int16_t j = -thickness / 2; j <= thickness / 2; j++)
        {
            if (x + i >= 0 && x + i < display->width && y + j >= 0 && y + j < display->height)
            {
                HX8357_DrawPixel(display, x + i, y + j, color);
            }
        }
    }
    return HX8357_OK;
}

HX8357_Status HX8357_DrawThickLine(HX8357_HandleTypeDef *display, uint16_t x1, uint16_t y1,  uint16_t x2, uint16_t y2, uint16_t color, uint8_t thickness)
{
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        HX8357_DrawThickPixel(display, x1, y1, color, thickness);

        if (x1 == x2 && y1 == y2)
            break;

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y1 += sy;
        }
    }
    return HX8357_OK;
}

void HX8357_BeginTouch(HX8357_HandleTypeDef *display) {
    HX8357_Unselect(display);
}

void HX8357_EndTouch(HX8357_HandleTypeDef *display) {
    HX8357_Select(display);
}
