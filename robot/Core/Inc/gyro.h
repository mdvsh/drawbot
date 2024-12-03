/*
 * gyro.h
 *
 *  Created on: Nov 19, 2024
 *      Author: madhavss, lbiondos
 */

#ifndef INC_GYRO_H_
#define INC_GYRO_H_

#include "main.h"
#include <stdbool.h>

#define L3GD20H_ADDR       0xD6    // I2C address (0x6B << 1)
#define CTRL1_REG_ADDR     0x20
#define CTRL2_REG_ADDR     0x21
#define CTRL3_REG_ADDR     0x22
#define CTRL4_REG_ADDR     0x23
#define CTRL5_REG_ADDR     0x24
#define STATUS_REG_ADDR    0x27

// Configuration options
#define GYRO_SCALE_245     0x00  // ±245 dps
#define GYRO_SCALE_500     0x10  // ±500 dps
#define GYRO_SCALE_2000    0x20  // ±2000 dps

#define GYRO_ODR_100HZ     0x0F  // Output data rate 100Hz, normal mode, all axes enabled
#define GYRO_ODR_200HZ     0x1F  // Output data rate 200Hz, normal mode, all axes enabled
#define GYRO_ODR_400HZ     0x2F  // Output data rate 400Hz, normal mode, all axes enabled

// Function prototypes
bool gyro_init(void);
void gyro_calibrate(void);
float gyro_get_dps(uint8_t axis);  // Returns degrees per second
float gyro_get_heading(void);       // Returns current heading in degrees
void gyro_reset_heading(void);
int16_t gyro_read_axis(uint8_t axis);  // axis: 0=X, 1=Y, 2=Z
void gyro_read_all(int16_t *x, int16_t *y, int16_t *z);

// Debug function
void gyro_print_registers(void);  // Add this for debugging

#endif /* INC_GYRO_H_ */
