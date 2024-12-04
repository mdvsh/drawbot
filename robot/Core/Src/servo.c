/*
 * servo.c
 *
 *  Created on: Dec 3, 2024
 *      Author: sharmadhavs
 */

#include "servo.h"

extern TIM_HandleTypeDef htim1;

void servo_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void servo_set_position(int16_t angle)
{
    if(angle < -90) angle = -90;
    if(angle > 90) angle = 90;

    // map -90 to +90 to 500-2400 mus;
    uint32_t pulse = 1450 + (angle * 950 / 90);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
}

void servo_set_position_smooth(int16_t target_angle, uint32_t step_delay_ms)
{
    static int16_t current_angle = 0;
    const int16_t STEP_SIZE = 2;

    while(current_angle != target_angle)
    {
        if(current_angle < target_angle) {
            current_angle += STEP_SIZE;
            if(current_angle > target_angle) {
                current_angle = target_angle;
            }
        } else {
            current_angle -= STEP_SIZE;
            if(current_angle < target_angle) {
                current_angle = target_angle;
            }
        }

        servo_set_position(current_angle);
        HAL_Delay(step_delay_ms);
    }
}

void pen_control(uint8_t up)
{
    if(up) {
        servo_set_position_smooth(90, 15);
    } else {
        servo_set_position_smooth(-90, 15);
    }
}
