#include "motors.h"
#include "hardware.h"
#include <stdlib.h>

static TIM_HandleTypeDef *motor_timers[2] = {&htim4, &htim3}; // Left, Right
static uint32_t motor_channels[2] = {TIM_CHANNEL_1, TIM_CHANNEL_2};
static uint16_t motor_pins[2] = {GPIO_PIN_9, GPIO_PIN_8};
static uint16_t current_speed[2] = {0, 0};
static uint8_t current_direction[2] = {MOTOR_FORWARD, MOTOR_FORWARD};

// convert Zumo-style speed (-400 to +400) to timer compare value (0 to 999)
#define ZUMO_TO_PWM(speed) ((uint16_t)(((abs(speed)) * 999) / 400))

void motors_init(void)
{
    // Force reset state first
    motor_set_speed(MOTOR_LEFT, 0);
    motor_set_speed(MOTOR_RIGHT, 0);
    motor_set_direction(MOTOR_LEFT, MOTOR_FORWARD);
    motor_set_direction(MOTOR_RIGHT, MOTOR_FORWARD);

    // Start PWM outputs
    HAL_TIM_PWM_Start(motor_timers[MOTOR_LEFT], motor_channels[MOTOR_LEFT]);
    HAL_TIM_PWM_Start(motor_timers[MOTOR_RIGHT], motor_channels[MOTOR_RIGHT]);
}

void motor_set_speed(uint8_t side, uint16_t speed)
{
    if (side > 1)
        return;
    current_speed[side] = speed;
    __HAL_TIM_SET_COMPARE(motor_timers[side], motor_channels[side], speed);
}

void motor_set_direction(uint8_t side, uint8_t direction)
{
    if (side > 1)
        return;
    current_direction[side] = direction;
    HAL_GPIO_WritePin(GPIOA, motor_pins[side], direction);
}

void motor_set_signed_speed(uint8_t motor, float speed)
{
    int16_t zumo_speed = (int16_t)speed;

    if (zumo_speed > 400)
        zumo_speed = 400;
    if (zumo_speed < -400)
        zumo_speed = -400;

    // Set direction based on sign
    if (zumo_speed >= 0) {
        motor_set_direction(motor, MOTOR_FORWARD);
    } else {
        motor_set_direction(motor, MOTOR_BACKWARD);
    }

    // Convert to PWM value (0-999) and set
    uint16_t pwm_value = ZUMO_TO_PWM(zumo_speed);
    motor_set_speed(motor, pwm_value);
}

void motors_stop(void)
{
    motor_set_speed(MOTOR_LEFT, 0);
    motor_set_speed(MOTOR_RIGHT, 0);
}

void motors_set_speeds(int16_t left, int16_t right)
{
    motor_set_signed_speed(MOTOR_LEFT, left);
    motor_set_signed_speed(MOTOR_RIGHT, right);
}

void motor_smooth_change(uint8_t side, uint8_t new_direction, uint16_t new_speed, uint16_t delay_ms)
{
    if (side > 1)
        return;

    // If direction change needed, slow down first
    if (current_direction[side] != new_direction) {
        while (current_speed[side] > 100) {
            current_speed[side]--;
            motor_set_speed(side, current_speed[side]);
            HAL_Delay(delay_ms);
        }
        motor_set_speed(side, 0);
        motor_set_direction(side, new_direction);
    }

    // Adjust speed
    while (current_speed[side] != new_speed) {
        if (current_speed[side] < new_speed) {
            current_speed[side]++;
        } else {
            current_speed[side]--;
        }
        motor_set_speed(side, current_speed[side]);
        HAL_Delay(delay_ms);
    }
}

void TestMotors(void)
{
    printf("Testing motors...\r\n");

    printf("Forward...\r\n");
    motors_set_speeds(50, 50);
    HAL_Delay(5000);

    printf("Stop...\r\n");
    motors_stop();
    HAL_Delay(500);

    printf("Turn right...\r\n");
    motors_set_speeds(50, -50);
    HAL_Delay(5000);

    printf("Stop...\r\n");
    motors_stop();
    HAL_Delay(500);

    printf("Turn left...\r\n");
    motors_set_speeds(-50, 50);
    HAL_Delay(5000);

    motors_stop();
    printf("Test complete\r\n");
}
