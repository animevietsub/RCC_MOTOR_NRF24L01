/**
 ******************************************************************************
 * @file           : hc595_i2s_pwm.c
 * @brief          : HC595 with I2S & PWM
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
#include "hc595_i2s_pwm.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "esp_log.h"

DRAM_ATTR uint16_t HC595_BUFFER = 0x0000;
DRAM_ATTR uint16_t TIMER_L_COUNTER = 0;
DRAM_ATTR uint16_t TIMER_R_COUNTER = 0;
DRAM_ATTR uint16_t TIMER_SERVO_COUNTER = 0;

DRAM_ATTR QueueHandle_t xQueue1;
DRAM_ATTR int64_t timeStartReceive, timeEndReceive;

DRAM_ATTR uint8_t D_PWM_L = 0;
DRAM_ATTR uint8_t D_PWM_R = 0;
DRAM_ATTR uint8_t D_PWM_SERVO = 7;

static const char *TAG = "[PWM_TIMER]";

IRAM_ATTR void HC595_QueueDelayI2S(uint32_t delayUs)
{
    uint16_t HC595_TEMP_BUFFER = HC595_BUFFER;
    for (uint32_t i = 0; i < delayUs / I2S_WS_PERIOD; i++)
    {
        xQueueSend(xQueue1, &HC595_TEMP_BUFFER, portMAX_DELAY);
    }
}

static void HC595_TaskSend()
{
    size_t i2s_bytes_write = DMA_BUFFER_PREPARE * sizeof(uint16_t);                                      // For first writing
    uint16_t *sampleData = heap_caps_malloc(DMA_BUFFER_PREPARE * sizeof(uint16_t) * 1, MALLOC_CAP_8BIT); // Create DMA-buffer
    memset(sampleData, 0x0000, DMA_BUFFER_PREPARE * sizeof(uint16_t) * 1);                               // Clear memory data
    uint16_t *sampleDataBegin = sampleData;                                                              // sampleData begin address
    uint16_t lastData = 0x0000;
    uint32_t queueMessagesWaiting = 0;
    // uint8_t dmaSelect = 0;
    while (1)
    {
        sampleData = sampleDataBegin + 1; // Start from first-half, add 1 more shift
        for (uint16_t i = 0; i < DMA_BUFFER_PREPARE / 2; i++)
        {
            if (uxQueueMessagesWaiting(xQueue1) > 0)
            {
                queueMessagesWaiting = uxQueueMessagesWaiting(xQueue1);
                if ((DMA_BUFFER_PREPARE / 2) - i < queueMessagesWaiting)
                {
                    queueMessagesWaiting = (DMA_BUFFER_PREPARE / 2) - i;
                }
                for (uint32_t temp = 0; temp < queueMessagesWaiting; temp++)
                {
                    xQueueReceive(xQueue1, sampleData, portMAX_DELAY); // Get new data
                    lastData = *sampleData;
                    sampleData++;
                    *sampleData = 0x0000;
                    sampleData++;
                }
                i += queueMessagesWaiting - 1;
            }
            else
            {
                *sampleData = lastData;
                sampleData++;
                *sampleData = 0x0000;
                sampleData++;
            }
        }
        sampleData = sampleData - DMA_BUFFER_PREPARE - 1; // Remove the shift
        i2s_write(I2S_NUM, sampleData, DMA_BUFFER_PREPARE * sizeof(uint16_t), &i2s_bytes_write, portMAX_DELAY);
    }
    heap_caps_free(sampleData);
    vTaskDelete(NULL);
}

void HC595_I2SInit()
{
    xQueue1 = xQueueCreate(DMA_BUFFER_PREPARE * QUEUE_DMA_MULTIPLIER, sizeof(uint16_t));
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = (HC595_CLKFREQ / I2S_NUM_CHANNEL / I2S_NUM_BIT),
        .bits_per_sample = I2S_NUM_BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .use_apll = false,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = DMA_BUFFER_COUNT,
        .dma_buf_len = DMA_BUFFER_LENGTH,
    };
    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = HC595_NUM_SRCLK,
        .ws_io_num = HC595_NUM_RCLK,
        .data_out_num = HC595_NUM_SER,
        .data_in_num = I2S_PIN_NO_CHANGE,
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
    i2s_start(I2S_NUM);
    xTaskCreatePinnedToCore(HC595_TaskSend, "[HC595_TaskSend]", 1024 * 4, NULL, 3, NULL, 1);
}

IRAM_ATTR void HC595_SendDataToQueue() // Add new data to Queue
{
    uint16_t HC595_TEMP_BUFFER = HC595_BUFFER;
    xQueueSend(xQueue1, &HC595_TEMP_BUFFER, portMAX_DELAY);
}

IRAM_ATTR void PWM_SetDutyCycle(uint16_t *COUNTER, uint16_t FREQUENCY, uint16_t PIN_OUT, uint8_t DUTY_CYCLE)
{
    if (*COUNTER < (PWM_FREQUENCY / FREQUENCY) * DUTY_CYCLE / 100)
    {
        HC595_BUFFER |= PIN_OUT;
    }
    else
    {
        HC595_BUFFER &= ~PIN_OUT;
    }
}

IRAM_ATTR void PWM_CheckDutyCycle(uint16_t *COUNTER, uint16_t FREQUENCY)
{
    *COUNTER = *COUNTER + 1;
    if (*COUNTER >= (PWM_FREQUENCY / FREQUENCY))
    {
        *COUNTER = 0;
    }
}

static bool IRAM_ATTR pwm_isr_callback() // PWM ISR send to I2S
{
    PWM_SetDutyCycle(&TIMER_L_COUNTER, PWM_MOTOR_FREQUENCY, PWM_ML_PINOUT, D_PWM_L);
    PWM_SetDutyCycle(&TIMER_R_COUNTER, PWM_MOTOR_FREQUENCY, PWM_MR_PINOUT, D_PWM_R);
    PWM_SetDutyCycle(&TIMER_SERVO_COUNTER, PWM_SERVO_FREQUENCY, PWM_SERVO_PINOUT, D_PWM_SERVO);
    HC595_QueueDelayI2S(1000000 / PWM_FREQUENCY);
    PWM_CheckDutyCycle(&TIMER_L_COUNTER, PWM_MOTOR_FREQUENCY);
    PWM_CheckDutyCycle(&TIMER_R_COUNTER, PWM_MOTOR_FREQUENCY);
    PWM_CheckDutyCycle(&TIMER_SERVO_COUNTER, PWM_SERVO_FREQUENCY);
    return pdTRUE;
}

void HC595I2SPWM_Init(hc595_control_t *control)
{
    timer_config_t config = {
        .divider = PWM_TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    timer_init(PWM_TIMER_GROUP, PWM_TIMER_IDX, &config);
    timer_set_counter_value(PWM_TIMER_GROUP, PWM_TIMER_IDX, 0);
    timer_set_alarm_value(PWM_TIMER_GROUP, PWM_TIMER_IDX, PWM_ALARM_VALUE);
    timer_enable_intr(PWM_TIMER_GROUP, PWM_TIMER_IDX);
    timer_isr_callback_add(PWM_TIMER_GROUP, PWM_TIMER_IDX, pwm_isr_callback, NULL, 0);
    timer_start(PWM_TIMER_GROUP, PWM_TIMER_IDX);
    (*control).D_PWM_L = &D_PWM_L;
    (*control).D_PWM_R = &D_PWM_R;
    (*control).D_PWM_SERVO = &D_PWM_SERVO;
}

void L298N_SetDirection(hc595_control_t *control, l298n_channel_t channel, l298n_direction_t dir)
{
    if (channel == L298N_CHANNEL_L)
    {
        switch (dir)
        {
        case L298N_DIRECTION_CW:
            HC595_BUFFER |= MLA_PINOUT;
            HC595_BUFFER &= ~MLB_PINOUT;
            break;
        case L298N_DIRECTION_CCW:
            HC595_BUFFER &= ~MLA_PINOUT;
            HC595_BUFFER |= MLB_PINOUT;
            break;
        case L298N_DIRECTION_HH:
            HC595_BUFFER |= MLA_PINOUT;
            HC595_BUFFER |= MLB_PINOUT;
            break;
        case L298N_DIRECTION_HL:
            HC595_BUFFER &= ~MLA_PINOUT;
            HC595_BUFFER &= ~MLB_PINOUT;
            break;
        }
        (*control).L298N_DIRECTION_L = dir;
    }
    else if (channel == L298N_CHANNEL_R)
    {
        switch (dir)
        {
        case L298N_DIRECTION_CW:
            HC595_BUFFER |= MRA_PINOUT;
            HC595_BUFFER &= ~MRB_PINOUT;
            break;
        case L298N_DIRECTION_CCW:
            HC595_BUFFER &= ~MRA_PINOUT;
            HC595_BUFFER |= MRB_PINOUT;
            break;
        case L298N_DIRECTION_HH:
            HC595_BUFFER |= MRA_PINOUT;
            HC595_BUFFER |= MRB_PINOUT;
            break;
        case L298N_DIRECTION_HL:
            HC595_BUFFER &= ~MRA_PINOUT;
            HC595_BUFFER &= ~MRB_PINOUT;
            break;
        }
        (*control).L298N_DIRECTION_R = dir;
    }
}

void L298N_SetPWMDuty(hc595_control_t *control, l298n_channel_t channel, uint8_t percent)
{
    if (channel == L298N_CHANNEL_L)
    {
        *((*control).D_PWM_L) = percent;
    }
    else if (channel == L298N_CHANNEL_R)
    {
        *((*control).D_PWM_R) = percent;
    }
}

void L298N_Stop(hc595_control_t *control, l298n_channel_t channel)
{
    L298N_SetPWMDuty(control, channel, 0);
    L298N_SetDirection(control, channel, L298N_DIRECTION_HL);
}

void L298N_Brake(hc595_control_t *control, l298n_channel_t channel)
{
    L298N_SetPWMDuty(control, channel, 100);
    L298N_SetDirection(control, channel, L298N_DIRECTION_HL);
}