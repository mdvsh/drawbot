/*
 * imu.h
 *
 *  Created on: Nov 19, 2024
 *      Author: madhavss
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

// Device address
#define LSM303D_ADDR (0x1D << 1) // SDO/SA0 connected to VDD

// Register addresses
#define WHO_AM_I_REG 0x0F
#define CTRL1_REG    0x20
#define CTRL2_REG    0x21
#define CTRL5_REG    0x24
#define CTRL6_REG    0x25
#define CTRL7_REG    0x26
#define OUT_X_L_A    0x28
#define OUT_X_H_A    0x29
#define OUT_Y_L_A    0x2A
#define OUT_Y_H_A    0x2B
#define OUT_Z_L_A    0x2C
#define OUT_Z_H_A    0x2D
#define OUT_X_L_M    0x08
#define OUT_X_H_M    0x09
#define OUT_Y_L_M    0x0A
#define OUT_Y_H_M    0x0B
#define OUT_Z_L_M    0x0C
#define OUT_Z_H_M    0x0D

typedef struct {
    float x;
    float y;
    float z;
} Vector3D;

typedef struct {
    Vector3D accel;
    Vector3D mag;
} imu_Data;

bool imu_init(void);
void imu_read_accel(Vector3D *accel);
void imu_read_mag(Vector3D *mag);
void imu_read_all(imu_Data *data);
float imu_get_heading(imu_Data *data);

#endif /* INC_IMU_H_ */
