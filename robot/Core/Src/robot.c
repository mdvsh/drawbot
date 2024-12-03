/*
 * robot.c
 *
 *  Created on: Nov 25, 2024
 *      Author: madhavss, lbiondos
 */

#include "robot.h"
#include "movement.h"

Robot global_robot;

void EnQueue(Robot *rob, point p){
	rob->points[rob->rear] = p;
	rob->rear = rob->rear + 1;
};

point DeQueue(Robot *rob) {
    if (rob->front == rob->rear) { // Queue is empty
        return (point){.x = 0, .y = 0, .pen_up = 1, .valid = false};
    }
    point ret = rob->points[rob->front];
    rob->front = rob->front + 1;
    return ret;
}


void Init(Robot *rob){
	rob->canvas_width = 102;
	rob->canvas_height = 76;

	rob->desired_x = 0;
	rob->desired_y = 0;

	rob->desired_distance = 0;
	rob->desired_heading = 0;

	rob->front = 0;
	rob->rear = 1;

	rob->pen_up = 1;
	rob->active = 0;
};


void UpdateDirection(Robot *rob) {
    point new_point = DeQueue(rob);

    if (!new_point.valid) {
        rob->active = false;
        rob->pen_up = true;
        return;
    }

    float delta_x = new_point.x - rob->current_x;
    float delta_y = new_point.y - rob->current_y;

    rob->desired_x = new_point.x;
    rob->desired_y = new_point.y;
    rob->desired_distance = sqrt(delta_x * delta_x + delta_y * delta_y);
//    rob->desired_heading = atan2f(delta_y, delta_x) * (180.0f / M_PI);

    float raw_heading = atan2f(delta_y, delta_x) * (180.0f / M_PI);
    rob->desired_heading = ((int)(raw_heading + 2.5f) / 5) * 5;
    if (rob->desired_heading < 0) rob->desired_heading += 360;

    rob->active = true;

    // change pen state if it's different
    if (rob->pen_up != new_point.pen_up) {
        rob->pen_up = new_point.pen_up;
        rob->last_pen_change = HAL_GetTick();
    }

    rob->active = true;
    rob->pen_up = new_point.pen_up;
}

void UpdateRobotPosition(Robot *rob) {
	rob->current_x = rob->desired_x;
	rob->current_y = rob->desired_y;
}

void ReadUart(Robot *rob, uint16_t x, uint16_t y, uint16_t pen) {
    // Extract Pen state (bit 16)
    bool pen_up = pen & 0x1;
    printf("DATA READ IN: (X: %d, Y: %d, PEN: %d)\r\n", x, y, pen);
    point new_point = {
        .x = x,
        .y = y,
        .pen_up = pen_up,
        .valid = true
    };

    EnQueue(rob, new_point);
}


