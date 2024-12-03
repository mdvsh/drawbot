/*
 * position.h
 *
 *  Created on: Nov 19, 2024
 *      Author: madhavss
 */

#ifndef INC_POSITION_H_
#define INC_POSITION_H_

typedef struct {
    float x;        // X position in mm
    float y;        // Y position in mm
    float heading;  // Heading in degrees (0-360)
} RobotPosition;

void position_init(void);
void position_update(void);
void position_get_current(RobotPosition* pos);
void position_set_current(RobotPosition* pos);


#endif /* INC_POSITION_H_ */
