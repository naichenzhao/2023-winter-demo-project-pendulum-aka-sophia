/*
 * Copyright (c) 2019 David Antliff
 *
 * This program provides an example using the esp32-rotary-encoder component.
 * Events are received via an event queue and displayed on the serial console.
 * The task also polls the device position every second to show that the latest
 * event always matches the current position.
 *
 * esp32-rotary-encoder is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * esp32-rotary-encoder is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with esp32-rotary-encoder.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "rotary_encoder.h"
#include "encoder.h"

#define TAG "app"

#define ROT_ENC1_A GPIO_NUM_12
#define ROT_ENC1_B GPIO_NUM_14
#define ROT_ENC2_A GPIO_NUM_27
#define ROT_ENC2_B GPIO_NUM_26

#define ENABLE_HALF_STEPS true
#define RESET_AT 0
#define FLIP_DIRECTION false

#define MAX_VAL 56310
#define TOTAL_DIST  55
#define ENCODER_TICKS 1024

// Encoder 1 is to get the angluar displacement
// Encoder 2 is for linear displcement

rotary_encoder_info_t encoder_1 = {0};
rotary_encoder_info_t encoder_2 = {0};
void encoder_init() {
    // Initialise the rotary encoder device with the GPIOs for A and B signals
    ESP_ERROR_CHECK(rotary_encoder_init(&encoder_1, ROT_ENC1_A, ROT_ENC1_B));
    ESP_ERROR_CHECK(rotary_encoder_init(&encoder_2, ROT_ENC2_A, ROT_ENC2_B));
    ESP_ERROR_CHECK(rotary_encoder_enable_half_steps(&encoder_1, ENABLE_HALF_STEPS));
    ESP_ERROR_CHECK(rotary_encoder_enable_half_steps(&encoder_2, ENABLE_HALF_STEPS));

    // Create a queue for events from the rotary encoder driver.
    // Tasks can read from this queue to receive up to date position information.
    QueueHandle_t event_queue1 = rotary_encoder_create_queue();
    ESP_ERROR_CHECK(rotary_encoder_set_queue(&encoder_1, event_queue1));

    QueueHandle_t event_queue2 = rotary_encoder_create_queue();
    ESP_ERROR_CHECK(rotary_encoder_set_queue(&encoder_2, event_queue2));
}

void reset_encoder1() {
    rotary_encoder_reset(&encoder_1);
}

void reset_encoder2() {
    rotary_encoder_reset(&encoder_2);
}

int get_count1() {
    rotary_encoder_state_t state1 = {0};
    ESP_ERROR_CHECK(rotary_encoder_get_state(&encoder_1, &state1));

    return state1.position;
}

int get_count2() {
    rotary_encoder_state_t state2 = {0};
    ESP_ERROR_CHECK(rotary_encoder_get_state(&encoder_2, &state2));

    return state2.position;
}

float get_dist() {
    return ((float)get_count2() * TOTAL_DIST) / MAX_VAL;
}

float get_angle() {
    return ((float)get_count1() * 360) / ENCODER_TICKS;
}