/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "ultrasonic.h"
#include <esp_err.h>

#define TRIG GPIO_NUM_6
#define ECHO GPIO_NUM_7
#define INTERRUPT GPIO_NUM_8
#define MAX_DISTANCE_CM 500

void record_rep(void)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIG,
        .echo_pin = ECHO
    };
    ultrasonic_init(&sensor);

    while (1) {
        float dist;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &dist);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else
            printf("Distance: %0.04f cm\n", dist*100);

        vTaskDelay(pdMS_TO_TICKS(500));
    }

    }

void app_main(void) {
    int recordState = 0;

}