/*
 * kalman.c
 *
 *  Created on: Dec 8, 2024
 *      Author: sharmadhavs
 */

#include "kalman.h"

void kalman_init(KalmanFilter *kf)
{
    kf->heading = 0.0f;
    kf->gyro_bias = 0.0f;

    kf->P[0][0] = 10.0f; // heading uncertainty
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.1f; // bias uncertainty

    kf->Q_angle = 0.005f; // process noise
    kf->Q_bias = 0.001f;  // bias drift
    kf->R_measure = 4.0f;

    kf->last_update = HAL_GetTick();
}

float kalman_update(KalmanFilter *kf, float gyro_rate, float mag_heading)
{
    uint32_t now = HAL_GetTick();
    float dt = (now - kf->last_update) / 1000.0f; // to seconds
    kf->last_update = now;

    // predict
    float rate = gyro_rate - kf->gyro_bias;
    kf->heading += dt * rate;

    // update error covariance matrix P
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    float innovation = mag_heading - kf->heading;
    while (innovation > 180)
        innovation -= 360;
    while (innovation < -180)
        innovation += 360;

    // calc Kalman gain
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // update state estimate
    kf->heading += K[0] * innovation;
    kf->gyro_bias += K[1] * innovation;

    // update error covariance matrix
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    while (kf->heading >= 360)
        kf->heading -= 360;
    while (kf->heading < 0)
        kf->heading += 360;

    return kf->heading;
}
