/*
 * imu.c
 *
 *  Created on: Nov 19, 2024
 *      Author: madhavss
 */

#include "imu.h"
#include "hardware.h"
#include "math.h"

// scale factors based on selected sensitivity
#define ACCEL_SCALE_2G 0.061f // mg/LSB
#define MAG_SCALE_4G   0.160f // mgauss/LSB

static struct {
    Vector3D mag_offset;
    Vector3D mag_scale;
} calibration = {.mag_offset = {0.0f, 0.0f, 0.0f}, .mag_scale = {1.0f, 1.0f, 1.0f}};

static uint8_t read_register(uint8_t reg_addr)
{
    uint8_t value;
    HAL_I2C_Mem_Read(&hi2c1, LSM303D_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &value, 1,
                     HAL_MAX_DELAY);
    return value;
}

static void write_register(uint8_t reg_addr, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, LSM303D_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &value, 1,
                      HAL_MAX_DELAY);
}

static int16_t read_16bit_data(uint8_t low_addr, uint8_t high_addr)
{
    uint8_t low = read_register(low_addr);
    uint8_t high = read_register(high_addr);
    return (int16_t)((high << 8) | low);
}

bool imu_init(void)
{
    uint8_t who_am_i = read_register(WHO_AM_I_REG);
    if (who_am_i != 0x49) {
        return false;
    }

    // config accelerometer: ±2g, 50Hz, all axes enabled
    write_register(CTRL1_REG, 0x57);
    write_register(CTRL2_REG, 0x00);

    // config mag: ±4 gauss, high resolution, 50Hz
    write_register(CTRL5_REG, 0x64);
    write_register(CTRL6_REG, 0x20);
    write_register(CTRL7_REG, 0x00);

    return true;
}

void imu_read_all(imu_Data *data)
{
    // read and convert accel data (mg to g)
    data->accel.x = read_16bit_data(OUT_X_L_A, OUT_X_H_A) * ACCEL_SCALE_2G / 1000.0f;
    data->accel.y = read_16bit_data(OUT_Y_L_A, OUT_Y_H_A) * ACCEL_SCALE_2G / 1000.0f;
    data->accel.z = read_16bit_data(OUT_Z_L_A, OUT_Z_H_A) * ACCEL_SCALE_2G / 1000.0f;

    // read and convert mag data (apply calibration)
    float mx = read_16bit_data(OUT_X_L_M, OUT_X_H_M) * MAG_SCALE_4G / 1000.0f;
    float my = read_16bit_data(OUT_Y_L_M, OUT_Y_H_M) * MAG_SCALE_4G / 1000.0f;
    float mz = read_16bit_data(OUT_Z_L_M, OUT_Z_H_M) * MAG_SCALE_4G / 1000.0f;

    // apply mag calibration
    data->mag.x = (mx - calibration.mag_offset.x) * calibration.mag_scale.x;
    data->mag.y = (my - calibration.mag_offset.y) * calibration.mag_scale.y;
    data->mag.z = (mz - calibration.mag_offset.z) * calibration.mag_scale.z;
}

float imu_get_heading(imu_Data *data)
{
    // Normalize accel vector
    float ax = data->accel.x;
    float ay = data->accel.y;
    float az = data->accel.z;
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Calculate pitch and roll
    float pitch = asinf(-ax);
    float roll = asinf(ay / cosf(pitch));

    // Tilt compensate magnetic vector
    float mx = data->mag.x;
    float my = data->mag.y;
    float mz = data->mag.z;

    float cos_pitch = cosf(pitch);
    float sin_pitch = sinf(pitch);
    float cos_roll = cosf(roll);
    float sin_roll = sinf(roll);

    float Xh = mx * cos_pitch + mz * sin_pitch;
    float Yh = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;

    // Calculate heading
    float heading = atan2f(Yh, Xh);

    // Convert to degrees
    heading *= 180.0f / M_PI;

    // Handle negative values
    if (heading < 0)
        heading += 360.0f;

    return heading;
}
