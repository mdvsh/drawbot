/*
 * movement.c
 *
 *  Created on: Nov 25, 2024
 *      Author: madhavss
 */

#include "movement.h"
#include "motors.h"
#include "gyro.h"
#include <math.h>

#define TURN_P_GAIN 15.0f
#define TURN_MIN_SPEED 75
#define TURN_MAX_SPEED 150

#define MOVE_P_GAIN 2.5f           // for heading correction
#define MOVE_BASE_SPEED 60
#define MOVE_ACCEL_TIME 200        // ms for acceleration
#define MOVE_DECEL_DISTANCE 5      // units before target to start slowing

#define PEN_CHANGE_DELAY 1000    // ms to wait after pen movement
#define QUARTER_INCH_TO_MM 6.35f
#define MOVE_UNIT_DISTANCE QUARTER_INCH_TO_MM

const float TURN_DONE_THRESHOLD = 2.0f;
const float DPS_STABLE_THRESHOLD = 2.0f;

Movement mov = {0};

void Movement_Init(void) {
    motors_init();
    mov.state = MOV_STATE_IDLE;
    mov.is_complete = true;
    mov.pen_down = false;
    mov.quarter_inch_to_mm = QUARTER_INCH_TO_MM;
    pen_control(1);
}

void Movement_SetMove(float target_heading, float distance_quarter_inches, bool should_draw) {
    mov.target_heading = target_heading;
    mov.target_distance = distance_quarter_inches;
    mov.should_draw = should_draw;
    mov.state = MOV_STATE_TURNING;
    mov.is_complete = false;
    mov.start_time = 0;
}

void Movement_Update(void) {
    float current_heading = gyro_get_heading();
    uint32_t current_time = HAL_GetTick();
    switch(mov.state) {
		case MOV_STATE_TURNING:
		{
            if (mov.pen_down) {
                pen_control(1);
                mov.pen_down = false;
                mov.last_pen_change = current_time;
                HAL_Delay(PEN_CHANGE_DELAY);
            }

			float heading_error = mov.target_heading - current_heading;

			heading_error = fmodf(heading_error + 540.0f, 360.0f) - 180.0f;

			printf("Turn Debug - Error: %.2f, Heading: %.2f, DPS: %.2f\r\n",
				   heading_error, current_heading, gyro_get_dps(2));
			if(fabs(heading_error) < TURN_DONE_THRESHOLD) {
				motors_stop();

				HAL_Delay(200);

				float current_dps = gyro_get_dps(2);
				if(fabs(current_dps) < DPS_STABLE_THRESHOLD) {
					printf("Turn complete - Final heading: %.2f\r\n", current_heading);
					mov.state = MOV_STATE_MOVING;
				}
			}
			else {
				float turn_speed = fabs(heading_error) * TURN_P_GAIN;

				turn_speed = fminf(TURN_MAX_SPEED, fmaxf(TURN_MIN_SPEED, turn_speed));

				//  deadband compensation
				if(turn_speed < 40) turn_speed = 40;

				int16_t left_speed, right_speed;
				if(heading_error < 0) {
					left_speed = (int16_t)turn_speed;
					right_speed = -(int16_t)turn_speed;
				} else {
					left_speed = -(int16_t)turn_speed;
					right_speed = (int16_t)turn_speed;
				}

				if(right_speed > 0) right_speed = (int16_t)(right_speed * 1.05f);
				if(left_speed > 0) left_speed = (int16_t)(left_speed * 1.0f);

				motors_set_speeds(left_speed, right_speed);
				HAL_Delay(20);
			}
		}
		break;

		case MOV_STATE_MOVING:
		{
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
                mov.start_time = current_time;
                mov.initial_heading = current_heading;
		    }

		    // TODO: fully integrate by tuning time_factor
            float distance_mm = mov.target_distance * mov.quarter_inch_to_mm;

		    uint32_t elapsed_time = current_time - mov.start_time;

		    float heading_error = mov.initial_heading - current_heading;
		    heading_error = fmodf(heading_error + 540.0f, 360.0f) - 180.0f;

		    // calc speed based on acceleration profile
		    float speed_factor = 1.0f;
		    if (elapsed_time < MOVE_ACCEL_TIME) {
		        speed_factor = (float)elapsed_time / MOVE_ACCEL_TIME;
		    }

		    int16_t base_speed = (int16_t)(MOVE_BASE_SPEED * speed_factor);

		    // heading correction
		    int16_t correction = (int16_t)(heading_error * MOVE_P_GAIN);
		    motors_set_speeds(
		        base_speed - correction,
		        base_speed + correction
		    );

		    // calc approximate distance moved (still could need to be tuned)
		    float time_factor = elapsed_time / 200.0f;
		    if (time_factor >= mov.target_distance) {
		        motors_stop();
		        HAL_Delay(100);
		        mov.state = MOV_STATE_IDLE;
		        mov.is_complete = true;
		        mov.start_time = 0;
		        printf("Movement complete\r\n");
		    }
		}
		break;

        case MOV_STATE_IDLE:
            motors_stop();
            break;
    }
}
bool Movement_IsComplete(void) {
    return mov.is_complete;
}
void Movement_PrintStatus(void) {
    printf("Movement Status:\r\n");
    printf("State: %d\r\n", mov.state);
    printf("Target Heading: %.2f\r\n", mov.target_heading);
    printf("Current Heading: %.2f\r\n", gyro_get_heading());
    printf("Is Complete: %d\r\n", mov.is_complete);
}

bool ExecuteSquareTest(Movement *movement) {
    const float SIDE_LENGTH = 40.0f;
    const float SQUARE_ANGLES[] = {0, 90, 180, 270};

    if (movement->test_step >= 4) {
        return true;
    }

    movement->target_heading = SQUARE_ANGLES[movement->test_step];
    movement->target_distance = SIDE_LENGTH;
    movement->state = MOV_STATE_TURNING;
    movement->is_complete = false;
    movement->test_step++;

    printf("Square Test Step %d: Heading to %.1f degrees, Distance %.1f\r\n",
           movement->test_step, movement->target_heading, movement->target_distance);

    return false; // test continue
}

