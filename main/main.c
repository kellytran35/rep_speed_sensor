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
#include "driver/gptimer.h"
#include "esp_spiffs.h"

#define LED 2
#define TRIG GPIO_NUM_6
#define ECHO GPIO_NUM_7
#define INTERRUPT_PIN GPIO_NUM_4
#define MAX_DISTANCE_CM 500
#define MEAS_TASK_NOTIFY_START (1U << 0)
#define MEAS_TASK_NOTIFY_STOP (1U << 1)

int runId = 0;
uint64_t timeSample;
float dist;
volatile bool recording = false;
FILE* dataFile;
QueueHandle_t interputQueue;
ultrasonic_sensor_t sensor;
TaskHandle_t meas_task_handle;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}

void timer_callback(void *param)
{
}

void take_sample(gptimer_handle_t gptimer)
{
    esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &dist);
    if (res != ESP_OK)
    {
        printf("Error %d: ", res);
        if (dataFile != NULL) {
            fprintf(dataFile, "Error %d: ", res);
        }
        
        switch (res)
        {
        case ESP_ERR_ULTRASONIC_PING:
            printf("Cannot ping - device in invalid state\n");
            if (dataFile != NULL) fprintf(dataFile, "Cannot ping\n");
            break;
        case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
            printf("Ping timeout - no device found\n");
            if (dataFile != NULL) fprintf(dataFile, "Ping timeout\n");
            break;
        case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
            printf("Echo timeout (distance potentially too big?)\n");
            if (dataFile != NULL) fprintf(dataFile, "Echo timeout\n");
            break;
        default:
            printf("%s\n", esp_err_to_name(res));
            if (dataFile != NULL) fprintf(dataFile, "%s\n", esp_err_to_name(res));
        }
    }
    else {
        gptimer_get_raw_count(gptimer, &timeSample);
        float elapsed_sec = timeSample / 1e6;
        printf("%0.04f cm, Time: %f\n", dist * 100, elapsed_sec);
        
        // Write to file
        if (dataFile != NULL) {
            fprintf(dataFile, "%0.04f cm, Time: %f\n", dist * 100, elapsed_sec);
            fflush(dataFile); // Ensure data is written immediately
        }
    }
}

void ultrasonic_control(void *pvParameters)
{
    gptimer_handle_t gptimer = (gptimer_handle_t) pvParameters;
    int pinNumber;
    char filename[50];
    while (true)
    {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
            if (!recording)
            {
                recording = true;
                printf("Recording started\n");
                sprintf(filename, "/spiffs/run_%d.txt", runId);
                // Open file for writing (append mode)
                dataFile = fopen(filename, "w");
                if (dataFile == NULL) {
                    printf("Failed to open file for writing\n");
                } else {
                    fprintf(dataFile, "Run %d\n", runId);
                    fflush(dataFile);
                }

                gptimer_set_raw_count(gptimer, 0);
                xTaskNotify(meas_task_handle, MEAS_TASK_NOTIFY_START, eSetBits);
            }
            else
            {
                printf("Recording ended\n");
                recording = false;
                xTaskNotify(meas_task_handle, MEAS_TASK_NOTIFY_STOP, eSetBits);

                // Close the file
                if (dataFile != NULL) {
                    fprintf(dataFile, "=== Recording Ended ===\n\n");
                    fclose(dataFile);
                    dataFile = NULL;
                    printf("Data saved to %s\n", filename);
                }

                runId++;
            }
        }
    }
}

void ultrasonic_meas(void *pvParameters)
{
    uint32_t notified;
    gptimer_handle_t gptimer = (gptimer_handle_t) pvParameters;
    while (1)
    {
        xTaskNotifyWait(0xFFFFFFFF, 0, &notified, portMAX_DELAY);
        if (notified & MEAS_TASK_NOTIFY_START)
        {
            while (1)
            {
                take_sample(gptimer);
                uint32_t inner_notified = 0;
                xTaskNotifyWait(0, 0xFFFFFFFF, &inner_notified, pdMS_TO_TICKS(1000));
                if (inner_notified & MEAS_TASK_NOTIFY_STOP)
                {
                    break;
                }
            }
        }
    }
}

void app_main(void)
{

    // Initialize SPIFFS
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_vfs_spiffs_register(&conf);

    // configure pins
    sensor.trigger_pin = TRIG;
    sensor.echo_pin = ECHO;
    ultrasonic_init(&sensor);

    esp_rom_gpio_pad_select_gpio(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);

    // create timer
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,   
        .resolution_hz = 1 * 1000 * 1000,   // 1 MHz resolution
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    // configure interrupts and tasks
    esp_rom_gpio_pad_select_gpio(INTERRUPT_PIN);
    gpio_set_direction(INTERRUPT_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(INTERRUPT_PIN);
    gpio_pullup_dis(INTERRUPT_PIN);
    gpio_set_intr_type(INTERRUPT_PIN, GPIO_INTR_POSEDGE);

    interputQueue = xQueueCreate(10, sizeof(int));
    xTaskCreate(ultrasonic_control, "Ultrasonic control", 8192, gptimer, 1, NULL);
    xTaskCreate(ultrasonic_meas, "Ultrasonic measurement", 8192, gptimer, 5, &meas_task_handle);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INTERRUPT_PIN, gpio_interrupt_handler, (void *)INTERRUPT_PIN);
}