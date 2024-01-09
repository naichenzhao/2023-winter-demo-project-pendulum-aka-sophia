#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
// #include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "motor.h"
#include "accel.h"
#include "encoder.h"


#define ACCEL_RST GPIO_NUM_32 // GPIO to reset the Accelerometer
#define MOTOR_TOGGLE GPIO_NUM_35 // GPIO to reset the Accelerometer

uint8_t RUN_MOTOR = 0;

float last_x = 0;
float last_theta = 0;
int64_t last_time = 0;

float curr_x = 0;
float curr_theta = 0;
int64_t curr_time = 0;

float x_dot = 0;
float theta_dot = 0;
int64_t dt;

float motor_Speed = 0;

void reset_state();
void change_motorstate();

void update_state();

// Controller functions

float bang_bang_controller(float curr_theta, float curr_x, float curr_dx, float curr_dtheta);
float pd_controller(float curr_theta, float curr_x, float curr_dtheta, float curr_dx);
//

void app_main(void) {
    // esp_task_wdt_config_t cfg = {0};
    // cfg.timeout_ms = 20000;
    // esp_task_wdt_init(&cfg);

    // Use interrupts for sensor reset and motor enable
    gpio_reset_pin(ACCEL_RST);
    gpio_set_direction(ACCEL_RST, GPIO_MODE_INPUT);
    gpio_set_intr_type(ACCEL_RST, GPIO_INTR_POSEDGE);
    gpio_pulldown_en(ACCEL_RST);
    gpio_pullup_dis(ACCEL_RST);

    gpio_reset_pin(MOTOR_TOGGLE);
    gpio_set_direction(MOTOR_TOGGLE, GPIO_MODE_INPUT);
    gpio_set_intr_type(MOTOR_TOGGLE, GPIO_INTR_POSEDGE);
    gpio_pulldown_en(MOTOR_TOGGLE);
    gpio_pullup_dis(MOTOR_TOGGLE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ACCEL_RST, reset_state, (void *)ACCEL_RST);
    gpio_isr_handler_add(MOTOR_TOGGLE, change_motorstate, (void *)MOTOR_TOGGLE);

    motor_init();
    accel_init();
    encoder_init();

    vTaskDelay(100 / portTICK_PERIOD_MS);

    last_x = get_dist();
    last_theta = gyro_x();
    last_time = esp_timer_get_time();
    set_gyro_ref();


    esp_task_wdt_deinit();

    printf("Starting main loop\n");
    while(1) {
        update_state();
        // float motor_Speed = bang_bang_controller(curr_theta, curr_x, theta_dot, x_dot);
        motor_Speed = motor_Speed + (dt) * pd_controller(curr_theta, curr_x, theta_dot, x_dot);
        

        if ((curr_theta < 45 && curr_theta > -45) && (curr_x < 20 && curr_x > -20)) {
            set_motor(motor_Speed);
        } else {
            set_motor(0);
        }

        // print the current state
        // printf("x:%f, theta:%f, dx:%f, dtheta:%f, gyro:%f\n", curr_x, curr_theta, x_dot, theta_dot, gyro_x());
        printf("x:%f, theta:%f, dx:%f, dtheta:%f, gyro:%f, motor:%f\n", curr_x, curr_theta, x_dot, theta_dot, gyro_x(), motor_Speed);
        // printf("gyro:%d, angle:%f, dist:%f, motor:%f, pain:45, suffering:-45\n", gyro_x(), curr_theta, curr_x, motor_Speed);
    }
    
}

float bang_bang_controller(float curr_theta, float curr_x, float curr_dtheta, float curr_dx) {
    if (curr_theta > 0) {
        return 100000;
    } else if (curr_theta < 0) {
        return -100000;
    } else {
        return 0;
    }
}

float pd_controller(float curr_theta, float curr_x, float curr_dtheta, float curr_dx) {
    const float kp_theta = -0.15; 
    const float kd_theta = -0.015; 
    const float kp_x = -0.03;     
    const float kd_x = -0.01;     

    float p_term_theta = kp_theta * (-curr_theta); 
    float d_term_theta = kd_theta * curr_dtheta;

    float p_term_x = kp_x * (-curr_x); 
    float d_term_x = kd_x * curr_dx;

    float control_output_theta = p_term_theta - d_term_theta;
    float control_output_x = p_term_x - d_term_x;

    return control_output_theta + control_output_x;
}




void update_state() {
    curr_x = get_dist();
    curr_theta = get_angle();
    curr_time = esp_timer_get_time();

    dt = curr_time - last_time;
    x_dot = ((curr_x - last_x) / (dt)) * 1000000;
    theta_dot = ((curr_theta - last_theta) / (dt)) * 1000000;

    // // printf("x:%f, theta:%f, dx:%f, dtheta:%f\n", curr_x, curr_theta, x_dot, theta_dot);
    last_x = curr_x;
    last_theta = curr_theta;
    last_time = curr_time;
}

void reset_state() {
    set_gyro_ref();
    reset_encoder1();
    reset_encoder2();
    motor_Speed = 0;
    // printf("Resetting gyro\n");
}

void change_motorstate() {
    if (RUN_MOTOR == 0) {
        RUN_MOTOR = 1;
        // printf("Turning ON Motor\n");
    } else {
        RUN_MOTOR = 0;
        // printf("Turning OFF Motor\n");
    }

}