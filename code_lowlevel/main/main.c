#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#include "motor.h"
#include "accel.h"

const int MAX_FREQ  = 75000;



void app_main(void) {
    // motor_init();
    accel_init();
    printf("Gyro setup complete\n");

    while(1) {
        // printf("%u\n", gyro_x());
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    printf("done\n");
    
}
