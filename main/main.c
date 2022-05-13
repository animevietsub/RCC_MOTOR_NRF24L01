/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : RCC_MOTOR_NRF24L01
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 Espressif.
 * All rights reserved.
 *
 * Vo Duc Toan / B1907202
 * Can Tho University.
 * March - 2022
 * Built with ESP-IDF Version: 4.4.
 * Target device: ESP32-WROOM.
 *
 ******************************************************************************
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

#include "hc595_i2s_pwm.h"
#include "fastmath.h"

#include "main.h"
#include "mirf.h"

static const char *TAG_NRF24L01 = "[NRF24L01]";

hc595_control_t hc595_control;
nrf24l01_data_t nrf24l01_data = {
    .AL_DATA = 0,
    .ML_DATA = 0,
    .AR_DATA = 0,
    .MR_DATA = 0,
};
uint8_t *nrf24l01_data_bytes;

void writeByteToStruct(const void *object, size_t size, uint8_t *in_bytes)
{
    unsigned char *byte;
    for (byte = object; size--; ++byte)
    {
        *byte = *in_bytes;
        in_bytes++;
    }
}

static void taskNRFReceiver()
{
    NRF24_t dev;
    Nrf24_init(&dev);
    uint8_t payload = sizeof(nrf24l01_data_t);
    uint8_t channel = 100;
    Nrf24_config(&dev, channel, payload);
    Nrf24_setRADDR(&dev, (uint8_t *)"GAMOT");
    Nrf24_SetSpeedDataRates(&dev, 1);
    Nrf24_setRetransmitDelay(&dev, 0);
    Nrf24_printDetails(&dev);
    while (1)
    {
        if (Nrf24_dataReady(&dev))
        {
            Nrf24_getData(&dev, nrf24l01_data_bytes);
            writeByteToStruct(&nrf24l01_data, sizeof(nrf24l01_data_t), nrf24l01_data_bytes);
            // ESP_LOGI(TAG_NRF24L01, "AL_DATA: %d", nrf24l01_data.AL_DATA);
            // ESP_LOGI(TAG_NRF24L01, "ML_DATA: %d", nrf24l01_data.ML_DATA);
            // ESP_LOGI(TAG_NRF24L01, "AR_DATA: %d", nrf24l01_data.AR_DATA);
            // ESP_LOGI(TAG_NRF24L01, "MR_DATA: %d", nrf24l01_data.MR_DATA);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}

static void taskMotorControl()
{
    while (1)
    {
        if (nrf24l01_data.MR_DATA > MAG_THRESHOLD)
        {
            if (nrf24l01_data.MR_DATA > 100)
                nrf24l01_data.MR_DATA = 100;
            L298N_SetPWMDir(&hc595_control, L298N_CHANNEL_L, (int8_t)(nrf24l01_data.MR_DATA * sinf((nrf24l01_data.AR_DATA + 45) / DEGREE_TO_RAD)));
            L298N_SetPWMDir(&hc595_control, L298N_CHANNEL_R, (int8_t)(nrf24l01_data.MR_DATA * cosf((nrf24l01_data.AR_DATA + 45) / DEGREE_TO_RAD)));
        }
        else
        {
            L298N_Stop(&hc595_control, L298N_CHANNEL_L);
            L298N_Stop(&hc595_control, L298N_CHANNEL_R);
        }
        if (nrf24l01_data.ML_DATA > MAG_THRESHOLD)
        {
            Servo_SetAngle(&hc595_control, nrf24l01_data.AL_DATA);
        }
        else
        {
            Servo_SetAngle(&hc595_control, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    nrf24l01_data_bytes = malloc(sizeof(nrf24l01_data_t));
    HC595_I2SInit();
    HC595I2SPWM_Init(&hc595_control);
    xTaskCreate(taskNRFReceiver, "[taskNRFReceiver]", 1024 * 3, NULL, 2, NULL);
    xTaskCreate(taskMotorControl, "[taskMotorControl]", 1024 * 3, NULL, 3, NULL);
}
