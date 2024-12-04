/*
 * servo.h
 *
 *  Created on: Dec 3, 2024
 *      Author: sharmadhavs
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"
#include <stdint.h>

void servo_init(void);
void servo_set_position(int16_t angle);
void servo_set_position_smooth(int16_t target_angle, uint32_t step_delay_ms);
void pen_control(uint8_t up);

#endif /* INC_SERVO_H_ */
