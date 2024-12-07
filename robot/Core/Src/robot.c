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

    float delta_forward = new_point.y - rob->current_y;
    float delta_right = new_point.x - rob->current_x;

    rob->desired_x = new_point.x;
    rob->desired_y = new_point.y;
    rob->desired_distance = sqrt(delta_forward * delta_forward + delta_right * delta_right);

    float raw_heading = atan2f(delta_right, delta_forward) * (180.0f / M_PI);
    float heading_difference = fabsf(raw_heading - rob->desired_heading);
    
    if (heading_difference <= 5.0f) {
        raw_heading = rob->desired_heading;
    }
    
    rob->desired_heading = raw_heading;
    
    if (new_point.x == 0 && new_point.y == 0) {
        // user reset, set heading to 0
        rob->desired_heading = 0;
    }
    
    if (rob->desired_heading < 0) rob->desired_heading += 360;
    rob->active = true;
    
    if (rob->pen_up != new_point.pen_up) {
        rob->pen_up = new_point.pen_up;
        rob->last_pen_change = HAL_GetTick();
    }

    rob->pen_up = new_point.pen_up;
}

void UpdateRobotPosition(Robot *rob) {
	rob->current_x = rob->desired_x;
	rob->current_y = rob->desired_y;
	debug_robot_state(rob);
}

void ReadUart(Robot *rob, uint16_t x, uint16_t y, uint16_t pen) {
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

void debug_robot_state(Robot *rob) {
    printf("Current Position: (%u, %u)\r\n", rob->current_x, rob->current_y);
    printf("Desired Position: (%u, %u)\r\n", rob->desired_x, rob->desired_y);
    printf("Desired Distance: %u\r\n", rob->desired_distance);
    printf("Desired Heading: %d\r\n", rob->desired_heading);
    printf("Queue Front: %u\r\n", rob->front);
    printf("Queue Rear: %u\r\n", rob->rear);
    printf("Active: %s\r\n", rob->active ? "true" : "false");
    printf("Pen Up: %s\r\n", rob->pen_up ? "true" : "false");
    printf("Last Pen Change: %u\r\n", rob->last_pen_change);
}


