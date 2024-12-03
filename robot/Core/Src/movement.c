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

#define TURN_TOLERANCE_DEG  10.0f
#define MOVEMENT_FORWARD_SPEED 50

Movement mov = {0};

void Movement_Init(void) {
    motors_init();
    mov.state = MOV_STATE_IDLE;
    mov.is_complete = true;
}

const float TURN_DONE_THRESHOLD = 2.0f;
const float SLOW_TURN_THRESHOLD = 30.0f;
const int16_t FAST_TURN_SPEED = 100;
const int16_t SLOW_TURN_SPEED = 60;
const float DPS_STABLE_THRESHOLD = 2.0f;

void Movement_Update(void) {
    float current_heading = gyro_get_heading();

    switch(mov.state) {
    case MOV_STATE_TURNING:
			{
				float heading_error = mov.target_heading - current_heading;
				while(heading_error > 180.0f) heading_error -= 360.0f;
				while(heading_error < -180.0f) heading_error += 360.0f;

				printf("Turn Debug - Error: %.2f, Heading: %.2f, DPS: %.2f\r\n",
					   heading_error, current_heading, gyro_get_dps(2));

				// Check if we're done turning
				if(fabs(heading_error) < TURN_DONE_THRESHOLD) {
					motors_stop();
					HAL_Delay(100);

					// Read DPS again after settling
					float current_dps = gyro_get_dps(2);
					if(fabs(current_dps) < DPS_STABLE_THRESHOLD) {
						printf("Turn complete - Final heading: %.2f\r\n", current_heading);
						mov.state = MOV_STATE_MOVING;
					}
				}
				else {
					// Determine turn speed based on error magnitude
					int16_t turn_speed = (fabs(heading_error) > SLOW_TURN_THRESHOLD) ?
										FAST_TURN_SPEED : SLOW_TURN_SPEED;
					if (turn_speed == SLOW_TURN_SPEED) {
						HAL_Delay(50);
					}

					if(heading_error < 0) {
						motors_set_speeds(turn_speed, -turn_speed);
					} else {
						motors_set_speeds(-turn_speed, turn_speed);
					}
					HAL_Delay(50);
				}
			}
            break;

        case MOV_STATE_MOVING:
            {
                float heading_error = mov.target_heading - current_heading;
                while(heading_error > 180.0f) heading_error -= 360.0f;
                while(heading_error < -180.0f) heading_error += 360.0f;

                int16_t base_speed = MOVEMENT_FORWARD_SPEED;
                int16_t correction = (int16_t)(heading_error * 1.5f);  // Tune this plkz

                motors_set_speeds(
                    base_speed - correction,
                    base_speed + correction
                );

                // Use slower forward speed
                motors_set_speeds(MOVEMENT_FORWARD_SPEED, MOVEMENT_FORWARD_SPEED);
                HAL_Delay(mov.target_distance * 200);
                motors_stop();
                HAL_Delay(100);  // Brief pause after stopping
                mov.state = MOV_STATE_IDLE;
                mov.is_complete = true;
                printf("Movement complete\r\n");
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
