/*
 * gyro.c
 *
 *  Created on: Nov 19, 2024
 *      Author: madhavss, lbiondos
 */

#include "gyro.h"
#include "hardware.h"
#include <math.h>
#include <stdio.h>

static uint8_t axis_addr_low[3] = {0x28, 0x2A, 0x2C};  // X, Y, Z low addresses
static uint8_t axis_addr_high[3] = {0x29, 0x2B, 0x2D}; // X, Y, Z high addresses
static int16_t gyro_bias[3] = {0, 0, 0};
static float sensitivity = 8.75f;
static float current_heading = 0.0f;
static uint32_t last_update = 0;

static uint8_t read_register(uint8_t reg_addr)
{
    uint8_t value;
    HAL_I2C_Mem_Read(&hi2c1, L3GD20H_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &value, 1,
                     HAL_MAX_DELAY);
    return value;
}

static void write_register(uint8_t reg_addr, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, L3GD20H_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &value, 1,
                      HAL_MAX_DELAY);
}

void gyro_print_registers(void)
{
    printf("CTRL1: 0x%02X\r\n", read_register(CTRL1_REG_ADDR));
    printf("CTRL4: 0x%02X\r\n", read_register(CTRL4_REG_ADDR));
    printf("CTRL5: 0x%02X\r\n", read_register(CTRL5_REG_ADDR));
    printf("STATUS: 0x%02X\r\n", read_register(STATUS_REG_ADDR));
}

bool gyro_init(void)
{
    HAL_Delay(2000);

    uint8_t whoami = read_register(0x0F);
    printf("WHO_AM_I: 0x%02X (should be 0xD7)\r\n", whoami);
    if (whoami != 0xD7) {
        printf("Gyro not responding correctly!\r\n");
        return false;
    }

    // More aggressive reset sequence
    write_register(CTRL5_REG_ADDR, 0x80); // Reboot memory
    HAL_Delay(100);

    // Power up sequence
    write_register(CTRL1_REG_ADDR, 0x00); // Power down first
    HAL_Delay(10);
    write_register(CTRL1_REG_ADDR, 0x0F); // Power up, all axes, 100Hz
    HAL_Delay(10);

    // Configure scale and filtering
    write_register(CTRL4_REG_ADDR, 0x00); // 245 dps
    write_register(CTRL5_REG_ADDR, 0x00); // No FIFO, no HPF

    printf("Gyro initialization:\r\n");
    gyro_print_registers();

    // Verify settings
    uint8_t ctrl1 = read_register(CTRL1_REG_ADDR);
    if ((ctrl1 & 0x0F) != 0x0F) {
        printf("Gyro init failed! CTRL1: 0x%02X\r\n", ctrl1);
        return false;
    }

    HAL_Delay(200);

    gyro_calibrate();
    printf("Calibration complete. Bias: %d, %d, %d\r\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    return true;
}

void gyro_calibrate(void)
{
    const int NUM_SAMPLES = 500;
    int32_t sum[3] = {0, 0, 0};

    printf("Starting calibration - keep robot still...\r\n");
    HAL_Delay(1000);

    for (int i = 0; i < NUM_SAMPLES; i++) {
        for (int axis = 0; axis < 3; axis++) {
            int16_t reading = gyro_read_axis(axis);
            sum[axis] += reading;
        }
        HAL_Delay(2);
    }

    // Calculate average
    for (int axis = 0; axis < 3; axis++) {
        gyro_bias[axis] = sum[axis] / NUM_SAMPLES;
        printf("Axis %d bias: %d\r\n", axis, gyro_bias[axis]);
    }

    // Verify calibration
    float test_reading = gyro_get_dps(2); // Z axis
    printf("Test reading after cal: %.2f dps\r\n", test_reading);

    current_heading = 0.0f;
    last_update = HAL_GetTick();
}

int16_t gyro_read_axis(uint8_t axis)
{
    if (axis > 2)
        return 0;
    uint8_t low = read_register(axis_addr_low[axis]);
    uint8_t high = read_register(axis_addr_high[axis]);
    return (int16_t)((high << 8) | low);
}

float gyro_get_dps(uint8_t axis)
{
    if (axis > 2)
        return 0.0f;
    int16_t raw = gyro_read_axis(axis);
    int16_t adjusted = raw - gyro_bias[axis];
    return (float)adjusted * sensitivity / 1000.0f;
}

float gyro_get_heading(void)
{
    uint32_t now = HAL_GetTick();
    float dt = (now - last_update) / 1000.0f;

    if (dt > 0.0f && dt < 0.5f) {
        float dps = gyro_get_dps(2);

        if (fabs(dps) > 0.1f) {
            // Apply simple low-pass filter
            static float filtered_dps = 0;
            filtered_dps = filtered_dps * 0.7f + dps * 0.3f;

            current_heading += filtered_dps * dt;

            static uint32_t last_print = 0;
            if ((now - last_print) > 500) {
                printf("DPS: %.2f (Raw: %.2f), Heading: %.2f\r\n", filtered_dps, dps,
                       current_heading);
                last_print = now;
            }
        }

        while (current_heading > 180.0f)
            current_heading -= 360.0f;
        while (current_heading < -180.0f)
            current_heading += 360.0f;
    }

    last_update = now;
    return current_heading;
}

void gyro_reset_heading(void)
{
    current_heading = 0.0f;
    last_update = HAL_GetTick();
}
