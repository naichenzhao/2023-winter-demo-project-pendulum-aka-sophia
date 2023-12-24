#include "driver/ledc.h"
#include "driver/gpio.h"
#include "motor.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (5) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_11_BIT 
#define LEDC_DUTY (1024)                 // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY (4000)           // Frequency in Hertz. Set frequency at 4 kHz

#define DIR_PIN GPIO_NUM_17
#define STEPS_REV 6400
#define PWM_LIMIT 40
#define MAX_FREQ 30000

void motor_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    gpio_reset_pin(DIR_PIN);
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);
}

void set_freq(int freq)
{
    if (freq < PWM_LIMIT) {
        ledc_set_freq(LEDC_MODE, LEDC_CHANNEL, LEDC_FREQUENCY);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    } else {
        ledc_set_freq(LEDC_MODE, LEDC_CHANNEL, freq);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
    }
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// Get speed input in radians per second, convert that to frequency
void set_motor(float speed) {

    speed = speed > MAX_FREQ? MAX_FREQ:speed;
    speed = speed < -MAX_FREQ ? -MAX_FREQ : speed;

    if (speed > 0) {
        gpio_set_level(DIR_PIN, 1);
        set_freq((int) speed);
    } else {
        gpio_set_level(DIR_PIN, 0);
        set_freq((int) -speed);
    }
}