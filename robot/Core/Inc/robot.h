/*
 * robot.h
 *
 *  Created on: Nov 25, 2024
 *      Author: madhavss, lbiondos
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define QUEUE_SIZE 64

typedef struct {
    uint16_t x;
    uint16_t y;
    bool pen_up;
    bool valid;
} point;

typedef struct {
    uint16_t canvas_width;
    uint16_t canvas_height;

    uint16_t current_x;
    uint16_t current_y;

    uint16_t desired_x;
    uint16_t desired_y;
    uint16_t desired_distance;
    int desired_heading;

    point points[QUEUE_SIZE];
    uint16_t front;
    uint16_t rear;

    bool active;
    bool pen_up;
    uint32_t last_pen_change;  // Track timing of pen movements
} Robot;

extern Robot global_robot;

void Init(Robot *rob);
void EnQueue(Robot *rob, point p);
point DeQueue(Robot *rob);
void UpdateDirection(Robot *rob);
void UpdateRobotPosition(Robot *rob);
void ReadUart(Robot *rob, uint16_t x, uint16_t y, uint16_t pen);

#endif /* INC_ROBOT_H_ */
