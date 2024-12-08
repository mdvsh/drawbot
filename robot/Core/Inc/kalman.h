/*
 * kalman.h
 *
 *  Created on: Dec 8, 2024
 *      Author: sharmadhavs
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "main.h"
#include <math.h>

// state vector [heading, gyro_bias]
typedef struct {
    float heading;   // current heading estimate (degrees)
    float gyro_bias; // estimated gyro bias (degrees/sec)
    float P[2][2];   // error covariance matrix
    float Q_angle;   // process noise for angle
    float Q_bias;    // process noise for bias
    float R_measure; // measurement noise
    uint32_t last_update;
} KalmanFilter;

void kalman_init(KalmanFilter *kf);
float kalman_update(KalmanFilter *kf, float gyro_rate, float mag_heading);

#endif /* INC_KALMAN_H_ */
