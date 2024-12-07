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
    TEST_IDLE,
    TEST_SQUARE,
} TestPattern;

typedef enum {
    MOV_STATE_IDLE,
    MOV_STATE_TURNING,
    MOV_STATE_MOVING,
	MOV_STATE_OFFSET,
} MovementState;

typedef struct {
    MovementState state;
    float target_heading;
    float target_distance;
    uint32_t start_time;
    float initial_heading;
    bool should_draw;
    bool is_complete;

    float distance_moved;

    bool pen_down;
    uint32_t last_pen_change;
    float quarter_inch_to_mm;

    TestPattern current_test;
    uint8_t test_step;
    bool test_mode;
    bool back_complete;
    int turn_offset;

} Movement;

extern Movement mov;

void Movement_Init(void);
void Movement_Update(void);
bool Movement_IsComplete(void);
void Movement_SetMove(float target_heading, float distance_quarter_inches, bool should_draw);
void Movement_TurnTo(float heading);
void Movement_Forward(float distance_cm);
void Movement_PrintStatus(void) ;

bool ExecuteSquareTest(Movement *movement);

#endif /* INC_MOVEMENT_H_ */
