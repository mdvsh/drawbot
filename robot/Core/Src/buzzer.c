/*
 * buzzer.c
 *
 *  Created on: Dec 7, 2024
 *      Author: sharmadhavs
 */

#include "buzzer.h"

extern TIM_HandleTypeDef htim2;

static BuzzerState current_state = BUZZER_IDLE;
static uint32_t last_beep_time = 0;
static const uint32_t DRAWING_BEEP_INTERVAL = 2000;

void buzzer_init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void buzzer_set_state(BuzzerState state)
{
    if (state != current_state) {
        current_state = state;

        switch (state) {
            case BUZZER_IDLE:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
                break;

            case BUZZER_DRAWING:
                last_beep_time = HAL_GetTick();
                buzzer_play_frequency(NOTE_C4, 100);
                break;

            case BUZZER_COMPLETE:
                buzzer_play_complete();
                break;
        }
    }
}

void buzzer_update(void)
{
    if (current_state == BUZZER_DRAWING) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_beep_time >= DRAWING_BEEP_INTERVAL) {
            buzzer_play_frequency(NOTE_E4, 50);
            last_beep_time = current_time;
        }
    }
}

void buzzer_play_startup(void)
{
    buzzer_play_frequency(NOTE_C4, 100);
    HAL_Delay(100);
    buzzer_play_frequency(NOTE_E4, 100);
    HAL_Delay(100);
    buzzer_play_frequency(NOTE_G4, 100);
    HAL_Delay(100);
    buzzer_play_frequency(NOTE_C5, 200);
    HAL_Delay(200);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void buzzer_play_complete(void)
{
    buzzer_play_frequency(NOTE_C4, 200);
    HAL_Delay(200);
    buzzer_play_frequency(NOTE_E4, 200);
    HAL_Delay(200);
    buzzer_play_frequency(NOTE_G4, 200);
    HAL_Delay(200);
    buzzer_play_frequency(NOTE_C5, 400);
    HAL_Delay(400);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void buzzer_play_frequency(uint16_t freq, uint16_t duration_ms)
{
    if (freq == 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        return;
    }

    uint32_t period = (65620 / freq);

    if (period > 1000000)
        period = 1000000;
    if (period < 100)
        period = 100;

    __HAL_TIM_SET_AUTORELOAD(&htim2, period);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, period / 2);

    if (duration_ms > 0) {
        HAL_Delay(duration_ms);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
}
