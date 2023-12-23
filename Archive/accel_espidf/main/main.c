#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "accel.h"


#define ACCEL_RST GPIO_NUM_9 // Designe GPIO you want ot toggle
const int Kp  = 2;

float last_x = 0;
float last_theta = 0;
int64_t last_time = 0;

float curr_x = 0;
float curr_theta = 0;
int64_t curr_time = 0;

float x_dot = 0;
float theta_dot = 0;
int64_t dt;

void app_main(void) {
    accel_init();
    gpio_reset_pin(ACCEL_RST);
    gpio_set_direction(ACCEL_RST, GPIO_MODE_INPUT);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    last_x = get_dist();
    last_theta = (int) gyro_x();
    last_time = esp_timer_get_time();
    set_ref();

    printf("Starting main loop\n");
    while(1) {
        if(gpio_get_level(ACCEL_RST) == 0) {
            set_ref();
            printf("Resetting gyro\n");
        } else {
            // accel_update();
            curr_x = get_dist();
            curr_theta = (int) gyro_x();
            curr_time = esp_timer_get_time();

            dt = curr_time - last_time;
            x_dot = ((curr_x - last_x) / (dt)) * 1000000;
            theta_dot = ((curr_theta - last_theta) / (dt)) * 1000000;

            // printf("x:%f, theta:%f, dx:%f, dtheta:%f\n", curr_x, curr_theta, x_dot, theta_dot);
            last_x = curr_x;
            last_theta = curr_theta;
            last_time = curr_time;
        }

        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // set_motor(80);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // set_motor(-80);
        set_motor(Kp * curr_theta);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        
    }

    printf("done\n");
    
}
