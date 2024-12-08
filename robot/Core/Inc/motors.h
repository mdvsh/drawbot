/*
 * motors.h
 *
 *  Created on: Nov 19, 2024
 *      Author: madhavss, lbiondos
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "main.h"
#include <stdint.h>

#define MOTOR_LEFT     0
#define MOTOR_RIGHT    1
#define SPEED          300.0f
#define MOTOR_FORWARD  0
#define MOTOR_BACKWARD 1

void motors_init(void);
// Set speed for an individual motor (-300 to +300)
// Positive = forward, Negative = backward
void motor_set_signed_speed(uint8_t motor, float speed);
void motors_stop(void);
void motor_set_speed(uint8_t motor, uint16_t speed);
void motor_set_direction(uint8_t motor, uint8_t direction);
void motors_set_speeds(int16_t left, int16_t right);

void TestMotors(void);

#endif /* INC_MOTORS_H_ */
