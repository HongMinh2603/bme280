#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "bme280.h"

#define I2C_MASTER_SCL_IO 22       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 21       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000  /*!< I2C master clock frequency */
#define BME280_I2C_ADDRESS  0x76   /*!< I2C address for BME280 sensor */

void app_main(void) {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);

    if (bme280_init(I2C_MASTER_NUM, BME280_I2C_ADDRESS) == ESP_OK) {
        bme280_data_t data;
        while (1) {
            if (bme280_read_data(I2C_MASTER_NUM, BME280_I2C_ADDRESS, &data) == ESP_OK) {
                printf("Temperature: %.2f C, Humidity: %.2f %%, Pressure: %.2f hPa\n",
                       data.temperature, data.humidity, data.pressure);
            } else {
                printf("Failed to read data from BME280 sensor\n");
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    } else {
        printf("Failed to initialize BME280 sensor\n");
    }
}
