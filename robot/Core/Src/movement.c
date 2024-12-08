/*
 * movement.c
 *
 *  Created on: Nov 25, 2024
 *      Author: madhavss
 */

#include "movement.h"
#include "buzzer.h"
#include "gyro.h"
#include "motors.h"

#include <math.h>
#include <stdio.h>

#define TURN_P_GAIN    7.0f
#define TURN_MIN_SPEED 80
#define TURN_MAX_SPEED 115

#define MOVE_P_GAIN     2.5f // for heading correction
#define MOVE_BASE_SPEED 60
#define MOVE_ACCEL_TIME 190 // ms for acceleration

#define PEN_CHANGE_DELAY 1000 // ms to wait after pen movement

const float BREAK_TURN_THRESHOLD = 4.0f; // todo may need to be 3
const float TURN_DONE_THRESHOLD = 3.0f;  // todo may need to be 3
const float DPS_STABLE_THRESHOLD = 2.0f;

Movement mov = {0};

void Movement_Init(void)
{
    motors_init();
    mov.state = MOV_STATE_IDLE;
    mov.is_complete = true;
    mov.pen_down = false;
    pen_control(1);
}

void Movement_SetMove(float target_heading, float distance, bool should_draw)
{
    mov.target_heading = target_heading;
    mov.target_distance = distance;
    mov.should_draw = should_draw;

    float current_heading = gyro_get_heading();
    float heading_error = mov.target_heading - current_heading;
    heading_error = fmodf(heading_error + 540.0f, 360.0f) - 180.0f;
    mov.state = (fabs(heading_error) < BREAK_TURN_THRESHOLD) ? MOV_STATE_MOVING : MOV_STATE_TURNING;

    mov.is_complete = false;
    mov.start_time = 0;
    mov.back_complete = false;
}

void Movement_Update(void)
{
    buzzer_update();
    float current_heading = gyro_get_heading();
    uint32_t current_time = HAL_GetTick();
    switch (mov.state) {
        case MOV_STATE_TURNING: {
            if (mov.pen_down) {
                pen_control(1);
                mov.pen_down = false;
                mov.last_pen_change = current_time;
                HAL_Delay(PEN_CHANGE_DELAY);
            }

            float heading_error = mov.target_heading - current_heading;
            heading_error = fmodf(heading_error + 540.0f, 360.0f) - 180.0f;

            if (!mov.back_complete) {
                mov.state = MOV_STATE_OFFSET;
                break;
            }

            printf("Turn Debug - Error: %.2f, Heading: %.2f, DPS: %.2f\r\n", heading_error,
                   current_heading, gyro_get_dps(2));
            if (fabs(heading_error) < TURN_DONE_THRESHOLD) {
                motors_stop();

                HAL_Delay(200);

                float current_dps = gyro_get_dps(2);
                if (fabs(current_dps) < DPS_STABLE_THRESHOLD) {
                    printf("Turn complete - Final heading: %.2f\r\n", current_heading);

                    if (!mov.should_draw) {
                        mov.state = MOV_STATE_MOVING;
                    } else {
                        mov.state = MOV_STATE_OFFSET;
                    }
                }
            } else {
                float turn_speed = fabs(heading_error) * TURN_P_GAIN;

                turn_speed = fminf(TURN_MAX_SPEED, fmaxf(TURN_MIN_SPEED, turn_speed));

                //  deadband compensation
                if (turn_speed < 40)
                    turn_speed = 40;

                int16_t left_speed, right_speed;
                if (heading_error < 0) {
                    left_speed = (int16_t)turn_speed;
                    right_speed = -(int16_t)turn_speed;
                } else {
                    left_speed = -(int16_t)turn_speed;
                    right_speed = (int16_t)turn_speed;
                }

                if (right_speed > 0)
                    right_speed = (int16_t)(right_speed * 1.05f);
                if (left_speed > 0)
                    left_speed = (int16_t)(left_speed * 1.0f);

                motors_set_speeds(left_speed, right_speed);
                HAL_Delay(20);
            }
        } break;

        case MOV_STATE_OFFSET: {
            if (mov.start_time == 0) {
                mov.start_time = HAL_GetTick();
                mov.initial_heading = gyro_get_heading();
                mov.distance_moved = 0.0f;
            }

            uint32_t elapsed_time = current_time - mov.start_time;

            float speed_factor = 1.0f;
            if (elapsed_time < MOVE_ACCEL_TIME) {
                speed_factor = (float)elapsed_time / MOVE_ACCEL_TIME;
            }

            int16_t base_speed = (int16_t)(MOVE_BASE_SPEED * speed_factor);
            if (!mov.back_complete) {
                base_speed = -fabs(base_speed);
            }

            float heading_error = mov.initial_heading - current_heading;
            heading_error = fmodf(heading_error + 540.0f, 360.0f) - 180.0f;

            int16_t correction = (int16_t)(heading_error * MOVE_P_GAIN);

            motors_set_speeds(base_speed - correction, base_speed + correction);

            const float DISTANCE_PER_SECOND_AT_BASE_SPEED = 6.0f; // todo: tune
            float distance_increment =
                (base_speed / (float)MOVE_BASE_SPEED) * (DISTANCE_PER_SECOND_AT_BASE_SPEED * 0.05f);

            mov.distance_moved += distance_increment;

            //		printf("Move Debug - Heading Error: %.2f, Speed: %d, Time: %lu, Distance:
            //%.2f/%.2f\r\n", 			   heading_error, base_speed, elapsed_time,
            //mov.distance_moved, mov.target_distance);
            if (fabs(mov.distance_moved) >= 5.8) {
                motors_set_speeds(20, 20); // slow down for slippage reduction
                HAL_Delay(50);
                motors_stop();
                HAL_Delay(50);
                // todo update curent x and y
                if (mov.back_complete) {
                    mov.state = MOV_STATE_MOVING;
                } else {
                    mov.state = MOV_STATE_TURNING;
                }
                mov.back_complete = true;
                mov.start_time = 0;
                printf("Offset complete - Final distance: %.2f\r\n", mov.distance_moved);
            }
        } break;

        case MOV_STATE_MOVING: {
            if (mov.start_time == 0) {
                if (mov.should_draw && !mov.pen_down) {
                    pen_control(0);
                    mov.pen_down = true;
                    mov.last_pen_change = current_time;
                    HAL_Delay(PEN_CHANGE_DELAY);
                }
                // elsethis is a non-drawing move and pen is down
                else if (!mov.should_draw && mov.pen_down) {
                    pen_control(1);
                    mov.pen_down = false;
                    mov.last_pen_change = current_time;
                    HAL_Delay(PEN_CHANGE_DELAY);
                }
                mov.start_time = HAL_GetTick();
                mov.initial_heading = gyro_get_heading();
                mov.distance_moved = 0.0f;
            }

            uint32_t elapsed_time = current_time - mov.start_time;

            float speed_factor = 1.0f;
            if (elapsed_time < MOVE_ACCEL_TIME) {
                speed_factor = (float)elapsed_time / MOVE_ACCEL_TIME;
            }

            int16_t base_speed = (int16_t)(MOVE_BASE_SPEED * speed_factor);

            float heading_error = mov.initial_heading - current_heading;
            heading_error = fmodf(heading_error + 540.0f, 360.0f) - 180.0f;

            int16_t correction = (int16_t)(heading_error * MOVE_P_GAIN);

            motors_set_speeds(base_speed - correction, base_speed + correction);

            const float DISTANCE_PER_SECOND_AT_BASE_SPEED = 6.0f; // todo: tune
            float distance_increment =
                (base_speed / (float)MOVE_BASE_SPEED) * (DISTANCE_PER_SECOND_AT_BASE_SPEED * 0.05f);

            mov.distance_moved += distance_increment;

            //		printf("Move Debug - Heading Error: %.2f, Speed: %d, Time: %lu, Distance:
            //%.2f/%.2f\r\n", 			   heading_error, base_speed, elapsed_time,
            //mov.distance_moved, mov.target_distance);

            if (mov.distance_moved >= mov.target_distance) {
                motors_set_speeds(20, 20); // slow down for slippage reduction
                HAL_Delay(50);
                motors_stop();
                HAL_Delay(50);

                mov.state = MOV_STATE_IDLE;
                mov.is_complete = true;
                mov.start_time = 0;
                printf("Movement complete - Final distance: %.2f\r\n", mov.distance_moved);
            }
        } break;

        case MOV_STATE_IDLE:
            motors_stop();
            if (mov.pen_down) {
                pen_control(1);
                mov.pen_down = false;
                mov.last_pen_change = current_time;
                HAL_Delay(PEN_CHANGE_DELAY);
            }
            break;
    }
}

bool Movement_IsComplete(void)
{
    return mov.is_complete;
}

void Movement_PrintStatus(void)
{
    printf("Movement Status:\r\n");
    printf("State: %d\r\n", mov.state);
    printf("Target Heading: %.2f\r\n", mov.target_heading);
    printf("Current Heading: %.2f\r\n", gyro_get_heading());
    printf("Is Complete: %d\r\n", mov.is_complete);
}

bool ExecuteSquareTest(Movement *movement)
{
    const float SIDE_LENGTH = 10.0f;
    const float SQUARE_ANGLES[] = {0, 90, 180, 270};

    if (movement->test_step >= 4) {
        return true;
    }

    movement->target_heading = SQUARE_ANGLES[movement->test_step];
    movement->target_distance = SIDE_LENGTH;
    movement->state = MOV_STATE_TURNING;
    movement->is_complete = false;
    movement->test_step++;

    printf("Square Test Step %d: Heading to %.1f degrees, Distance %.1f\r\n", movement->test_step,
           movement->target_heading, movement->target_distance);

    return false; // test continue
}
