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
#include "driver/uart.h"
#include <string.h>

// Define the UART port number to be used for remote controlling
#define UART_NUM UART_NUM_2

#define UART_TX_GPIO_NUM GPIO_NUM_33
#define UART_RX_GPIO_NUM GPIO_NUM_25
#define BEARLY_RESET_GPIO GPIO_NUM_2


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
bool bearly_ready = false;


void reset_state();
void change_motorstate();

void update_state();

void initialize_uart() {
    // Configure parameters of an UART driver
    uart_config_t uart_config = {
        .baud_rate = 115200, // Set your desired baud rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Install UART driver (we don't need an event queue here)
    // Note: Ensure the UART buffer sizes are large enough for the data you're sending/receiving
    uart_driver_install(UART_NUM, 1024 * 2, 0, 0, NULL, 0);

    // Set UART parameters
    uart_param_config(UART_NUM, &uart_config);

    // Set UART pins
    uart_set_pin(UART_NUM, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Controller functions

float bang_bang_controller(float curr_theta, float curr_x, float curr_dx, float curr_dtheta);
float pd_controller(float curr_theta, float curr_x, float curr_dtheta, float curr_dx);
float remote_controller(float curr_theta, float curr_x, float curr_dtheta, float curr_dx);
//

void app_main(void) {
    // esp_task_wdt_config_t cfg = {0};
    // cfg.timeout_ms = 20000;
    // esp_task_wdt_init(&cfg);

    // Set BEARLY_RESET_GPIO as an output, and set it to 1
    gpio_reset_pin(BEARLY_RESET_GPIO);
    gpio_set_direction(BEARLY_RESET_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BEARLY_RESET_GPIO, 1);

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
    encoder_init();
    initialize_uart();

    vTaskDelay(100 / portTICK_PERIOD_MS);

    last_x = get_dist();
    last_theta = gyro_x();
    last_time = esp_timer_get_time();
    set_gyro_ref();


    esp_task_wdt_deinit();

    printf("Starting main loop\n");
    while(1) {
        update_state();

        // motor_Speed = bang_bang_controller(curr_theta, curr_x, theta_dot, x_dot);
        // while (!bearly_ready) {
        //     vTaskDelay(100 / portTICK_PERIOD_MS);
        // }
        float target = remote_controller(curr_theta, curr_x, theta_dot, x_dot);
        motor_Speed = (motor_Speed + (dt) * target);
        if (motor_Speed > 2048) {
            motor_Speed = 2048;
        } else if (motor_Speed < -2048) {
            motor_Speed = -2048;
        }
        // motor_Speed = 2000;
        

        if ((curr_theta < 45 && curr_theta > -45) && (curr_x < 23 && curr_x > -23)) {
            set_motor(motor_Speed);
        } else {
            set_motor(0);
        }

        // print the current state
        // printf("x:%f, theta:%f, dx:%f, dtheta:%f, gyro:%f\n", curr_x, curr_theta, x_dot, theta_dot, gyro_x());
        printf("x:%f, theta:%f, dx:%f, dtheta:%f, motor:%f\n", curr_x, curr_theta, x_dot, theta_dot, motor_Speed);
        // printf("gyro:%d, angle:%f, dist:%f, motor:%f, pain:45, suffering:-45\n", gyro_x(), curr_theta, curr_x, motor_Speed);
    }
    
}


float remote_controller(float curr_theta, float curr_x, float curr_dtheta, float curr_dx) {
    // Buffer for sending data
    uint8_t data_to_send[16]; // Adjust the size based on your data format

    // Copy the float values into the byte array (assuming little-endian format)
    memcpy(data_to_send, &curr_theta, sizeof(float));
    memcpy(data_to_send + 4, &curr_x, sizeof(float));
    memcpy(data_to_send + 8, &curr_dtheta, sizeof(float));
    memcpy(data_to_send + 12, &curr_dx, sizeof(float));

    // Transmit the data over UART
    uart_write_bytes(UART_NUM, (const char *)data_to_send, sizeof(data_to_send));

    // Buffer for receiving data
    uint8_t data_received[4];
    int length = 0;

    // Wait for the float value to be received
    while (length != sizeof(float)) {
        length += uart_read_bytes(UART_NUM, data_received + length, sizeof(float) - length, portMAX_DELAY);
    }

    // Convert the received bytes back into a float (assuming little-endian format)
    float received_value;
    memcpy(&received_value, data_received, sizeof(float));

    return received_value;
}


float bang_bang_controller(float curr_theta, float curr_x, float curr_dtheta, float curr_dx) {
    if (curr_theta > 0) {
        return 2000;
    } else if (curr_theta < 0) {
        return -2000;
    } else {
        return 0;
    }
}

float pd_controller(float curr_theta, float curr_x, float curr_dtheta, float curr_dx) {
    const float kp_theta = -0.012; 
    // const float kd_theta = -0.015;
    const float kd_theta = -0.0012;
    const float kp_x = 0.0024;
    // const float kd_x = -0.01;
    const float kd_x = 0.0008;

    float p_term_theta = kp_theta * (-curr_theta); 
    float d_term_theta = kd_theta * curr_dtheta;

    float p_term_x = kp_x * (-curr_x); 
    float d_term_x = kd_x * curr_dx;

    float control_output_theta = p_term_theta - d_term_theta;
    float control_output_x = p_term_x - d_term_x;

    return control_output_theta + control_output_x;
}

// Reset bearly by holding the reset pin low for 1 second, then high for 1 second
void reset_bearly() {
    if(bearly_ready) {
        gpio_set_level(BEARLY_RESET_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BEARLY_RESET_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        bearly_ready = true;
    }
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
    bearly_ready = false;
    set_gyro_ref();
    reset_encoder1();
    reset_encoder2();
    x_dot = 0;
    theta_dot = 0;
    curr_x = 0;
    curr_theta = 0;
    motor_Speed = 0;
    reset_bearly();
    
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