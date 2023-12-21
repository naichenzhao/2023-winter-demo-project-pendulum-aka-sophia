#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#include "accel.h"

// Define ports and addresses
#define I2C_MASTER_SCL_IO 18
#define I2C_MASTER_SDA_IO 19
#define TEST_I2C_PORT -1

#define MPU0_ADDR 0x69


// Define other global variables
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

void set_default(void) {

    // Setup writes
    writeByte(dev_handle, MPU9250_PWR_MGMT_1, 0x80);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    writeByte(dev_handle, MPU9250_PWR_MGMT_1, 0x00);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    writeByte(dev_handle, MPU9250_PWR_MGMT_1, 0x01);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    writeByte(dev_handle, MPU9250_MPU_CONFIG, 0x03);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    writeByte(dev_handle, MPU9250_SMPLRT_DIV, 0x02);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint8_t c = readByte(dev_handle, MPU9250_GYRO_CONFIG); // get current GYRO_CONFIG register value
    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3

    c = c & ~0xE0;           // Clear self-test bits [7:5]
    c = c & ~0x03;           // Clear Fchoice bits [1:0]
    c = c & ~0x18;           // Clear GFS bits [4:3]
    c = c | (uint8_t)3 << 3; // Set full scale range for the gyro (11 on 4:3)
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    writeByte(dev_handle, MPU9250_GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = readByte(dev_handle, MPU9250_ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    c = c & ~0xE0;                                  // Clear self-test bits [7:5]
    c = c & ~0x18;                                  // Clear AFS bits [4:3]
    c = c | (uint8_t)3 << 3;                        // Set full scale range for the accelerometer (11 on 4:3)
    writeByte(dev_handle, MPU9250_ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = readByte(dev_handle, MPU9250_ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F;                                   // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;                                    // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    writeByte(dev_handle, MPU9250_ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(dev_handle, MPU9250_INT_PIN_CFG, 0x22); // enable Magnetometer bypass
    writeByte(dev_handle, MPU9250_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void accel_init(void) {

    // Setup I2C Config
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TEST_I2C_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // Check device exists
    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, MPU0_ADDR, -1));
    printf("MPU9250 device found...\n");

    // Setup device config
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU0_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    // Check WHOAMI for sensor
    uint8_t ID = readByte(dev_handle, MPU9250_WHO_AM_I_MPU9250);
    printf("WHOAMI: 0x%02x\n", ID);
    
    set_default();
}

int gyro_x()
{
    uint8_t upper = readByte(dev_handle, 0x43);
    uint8_t lower = readByte(dev_handle, 0x44);
    return (upper << 8) | lower;
}

void writeByte(i2c_master_dev_handle_t i2c_dev, uint8_t subaddr, uint8_t data) {
    uint8_t write_buf[2] = {subaddr, data};
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev, write_buf, sizeof(write_buf), -1));
}

uint8_t readByte(i2c_master_dev_handle_t i2c_dev, uint8_t subaddr) {
    uint8_t write_buf[1] = {subaddr};
    uint8_t ret_buf[1];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_dev, write_buf, sizeof(write_buf), ret_buf, 1, -1));
    return *ret_buf;
}

uint8_t readBytes(i2c_master_dev_handle_t i2c_dev, uint8_t subaddr, uint8_t cont, uint8_t* dest)  {
    uint8_t write_buf[1] = {subaddr};
    uint8_t ret_buf[1];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_dev, write_buf, sizeof(write_buf), ret_buf, 1, -1));
    return *ret_buf;
}