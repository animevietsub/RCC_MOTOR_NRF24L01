/**
 ******************************************************************************
 * @file           : main.h
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
#ifndef __MAIN_H__
#define __MAIN_H__

typedef struct
{
    int16_t AL_DATA;
    int16_t ML_DATA;
    int16_t AR_DATA;
    int16_t MR_DATA;
} nrf24l01_data_t;

void writeByteToStruct(const void *object, size_t size, uint8_t *in_bytes);

#endif
