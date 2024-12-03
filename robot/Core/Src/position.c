/*
 * position.c
 *
 *  Created on: Nov 19, 2024
 *      Author: madhavss
 */

#include "position.h"
#include "gyro.h"
#include <math.h>

static RobotPosition current_position = {0, 0, 0};
static uint32_t last_update_time = 0;

void position_init(void) {
    current_position.x = 0;
    current_position.y = 0;
    current_position.heading = 0;
    last_update_time = HAL_GetTick();
}

void position_get_current(RobotPosition* pos) {
    pos->x = current_position.x;
    pos->y = current_position.y;
    pos->heading = current_position.heading;
}

void position_set_current(RobotPosition* pos) {
    current_position = *pos;
}
