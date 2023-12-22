#include "driver/ledc.h"
#include "driver/gpio.h"
#include "motor.h"

#define LEDC_TIMER_0 LEDC_TIMER_0
#define LEDC_TIMER_1 LEDC_TIMER_1
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_0 (5) // Define the output GPIO
#define LEDC_OUTPUT_IO_1 (4) // Define the output GPIO

#define LEDC_CHANNEL_0 LEDC_CHANNEL_0
#define LEDC_CHANNEL_1 LEDC_CHANNEL_1
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (8192)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY (8000)           // Frequency in Hertz. Set frequency at 8 kHz

#define DIR_PIN GPIO_NUM_4

void motor_init(void) {

    // Setup PWM_0 for pin 4
    ledc_timer_config_t ledc_timer_0 = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_0));

    ledc_channel_config_t ledc_channel_0 = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO_0,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_0));

    // Setup PWM_1 for pin 5
    ledc_timer_config_t ledc_timer_1 = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_1));
    ledc_channel_config_t ledc_channel_1 = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO_1,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));


    // Set motor speeds to 0
    set_motor(0);
}

// Get speed input in radians per second, convert that to frequency
void set_motor(float speed) {
    int converted_speed = (int) (speed * LEDC_DUTY) / 100;
    converted_speed = (converted_speed > LEDC_DUTY) ? LEDC_DUTY : converted_speed;
    converted_speed = (converted_speed < -LEDC_DUTY) ? -LEDC_DUTY : converted_speed;

    printf("%d\n", converted_speed);
    if (speed > 0) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, converted_speed);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0);
    } else {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, -converted_speed);
    }
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
}
