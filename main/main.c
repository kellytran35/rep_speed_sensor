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

#define LED 2
#define TRIG GPIO_NUM_6
#define ECHO GPIO_NUM_7
#define INTERRUPT_PIN GPIO_NUM_4
#define MAX_DISTANCE_CM 500
#define MEAS_TASK_NOTIFY_START  (1U << 0)
#define MEAS_TASK_NOTIFY_STOP   (1U << 1)

int readings = 0;
float dist;
volatile bool recording = false;
QueueHandle_t interputQueue;
ultrasonic_sensor_t sensor;
TaskHandle_t meas_task_handle; 

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}


void take_sample () {
    esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &dist);
            if (res != ESP_OK)
            {
                printf("Error %d: ", res);
                switch (res)
                {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping - device in invalid state\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout - no device found\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (distance potentially too big?)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
                }
            }
            else
                readings++;
            printf("%0.04f cm, Reading: %d\n", dist * 100, readings);
}

void ultrasonic_control(void *pvParameters)
{
    int pinNumber;
    while (true)
    {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
            if (!recording)
            {
                recording = true;
                readings = 0;
                printf("Recording started\n");
                xTaskNotify(meas_task_handle, MEAS_TASK_NOTIFY_START, eSetBits);

            }
            else
            {
                printf("Recording ended\n");
                recording = false;
                xTaskNotify(meas_task_handle, MEAS_TASK_NOTIFY_STOP, eSetBits);
            }
        }
    }
}

void ultrasonic_meas(void *pvParameters) {
    uint32_t notified;
    while (1) {
        xTaskNotifyWait(0xFFFFFFFF, 0, &notified, portMAX_DELAY);
        if (notified && MEAS_TASK_NOTIFY_START) {
            while (1) {
                take_sample();
                uint32_t inner_notified = 0;
                xTaskNotifyWait(0, 0xFFFFFFFF, &inner_notified, pdMS_TO_TICKS(1000));
                if (inner_notified & MEAS_TASK_NOTIFY_STOP) {
                    break;
                }
            }
        }
    }
}


void app_main(void)
{

    sensor.trigger_pin = TRIG;
    sensor.echo_pin = ECHO;
    ultrasonic_init(&sensor);

    esp_rom_gpio_pad_select_gpio(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(INTERRUPT_PIN);
    gpio_set_direction(INTERRUPT_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(INTERRUPT_PIN);
    gpio_pullup_dis(INTERRUPT_PIN);
    gpio_set_intr_type(INTERRUPT_PIN, GPIO_INTR_POSEDGE);

    interputQueue = xQueueCreate(10, sizeof(int));
    xTaskCreate(ultrasonic_control, "Ultrasonic control", 2048, NULL, 1, NULL);
    xTaskCreate(ultrasonic_meas, "Ultrasonic measurement", 4096, NULL, 5, &meas_task_handle);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INTERRUPT_PIN, gpio_interrupt_handler, (void *)INTERRUPT_PIN);

}