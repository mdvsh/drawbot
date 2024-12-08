/*
 * buzzer.h
 *
 *  Created on: Dec 7, 2024
 *      Author: sharmadhavs
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "main.h"

#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523

typedef enum { BUZZER_IDLE, BUZZER_DRAWING, BUZZER_COMPLETE } BuzzerState;

void buzzer_init(void);
void buzzer_set_state(BuzzerState state);
void buzzer_update(void);
void buzzer_play_startup(void);
void buzzer_play_complete(void);
void buzzer_play_frequency(uint16_t freq, uint16_t duration_ms);

#endif /* INC_BUZZER_H_ */
