/*
 * movement.h
 *
 *  Created on: Nov 25, 2024
 *      Author: madhavss
 */

#ifndef INC_MOVEMENT_H_
#define INC_MOVEMENT_H_

#include "main.h"
#include <stdint.h>
#include "stdbool.h"

typedef enum {
    MOV_STATE_IDLE,
    MOV_STATE_TURNING,
    MOV_STATE_MOVING
} MovementState;

typedef struct {
    MovementState state;
    float target_heading;
    float target_distance;
    bool is_complete;
} Movement;

extern Movement mov;

void Movement_Init(void);
void Movement_Update(void);
bool Movement_IsComplete(void);
void Movement_TurnTo(float heading);
void Movement_Forward(float distance_cm);
void Movement_PrintStatus(void) ;

#endif /* INC_MOVEMENT_H_ */
