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

#include "main.h"

hc595_control_t hc595_control;

void app_main(void)
{
    HC595_I2SInit();
    HC595I2SPWM_Init(&hc595_control);
    L298N_SetDirection(&hc595_control, L298N_CHANNEL_L, L298N_DIRECTION_CW);
    L298N_SetDirection(&hc595_control, L298N_CHANNEL_R, L298N_DIRECTION_CW);
    L298N_SetPWMDuty(&hc595_control, L298N_CHANNEL_L, 50);
    L298N_SetPWMDuty(&hc595_control, L298N_CHANNEL_R, 50);
}
