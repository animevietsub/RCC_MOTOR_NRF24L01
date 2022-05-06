#ifndef __HC595_I2S_PWM_H__
#define __HC595_I2S_PWM_H__

#define PWM_FREQUENCY (16 * 1000)
#define PWM_TIMER_GROUP (TIMER_GROUP_0) // Timer group
#define PWM_TIMER_IDX (TIMER_0)         // Timer index
#define PWM_TIMER_DIVIDER (16)          //  Hardware timer clock divider
#define PWM_SCALE (TIMER_BASE_CLK / PWM_TIMER_DIVIDER)
#define PWM_ALARM_VALUE (PWM_SCALE / PWM_FREQUENCY)

#define HC595_CLKFREQ (8 * 1000 * 1000)
#define I2S_NUM_CHANNEL 2
#define I2S_NUM_BIT I2S_BITS_PER_SAMPLE_16BIT
#define I2S_NUM (0)
#define I2S_WS_PERIOD ((16 * I2S_NUM_CHANNEL) / (HC595_CLKFREQ / (1000 * 1000))) // 16 bit data - 4 us
#define DMA_BUFFER_LENGTH 64
#define DMA_BUFFER_COUNT 16
#define DMA_BUFFER_PREPARE (DMA_BUFFER_LENGTH * 2) // 64 queues * 2 channels
#define QUEUE_DMA_MULTIPLIER 8

#define HC595_NUM_RCLK 17
#define HC595_NUM_SRCLK 16
#define HC595_NUM_SER 5

#define PWM_MOTOR_FREQUENCY (800)
#define PWM_SERVO_FREQUENCY (50)

#define MLA_PINOUT (BIT4)
#define MLB_PINOUT (BIT3)
#define PWM_ML_PINOUT (BIT5)
#define MRA_PINOUT (BIT2)
#define MRB_PINOUT (BIT1)
#define PWM_MR_PINOUT (BIT0)
#define PWM_SERVO_PINOUT (BIT7)
#define PWM_LED_PINOUT (BIT6)

typedef enum
{
    L298N_DIRECTION_CW,  // Clockwise direction
    L298N_DIRECTION_CCW, // Counter-clockwise direction
    L298N_DIRECTION_HH,  // Hold high
    L298N_DIRECTION_HL,  // Hold low
} l298n_direction_t;

typedef enum
{
    L298N_CHANNEL_L, // Channel A
    L298N_CHANNEL_R, // Channel B
} l298n_channel_t;

typedef struct
{
    uint8_t *D_PWM_L;
    uint8_t *D_PWM_R;
    uint8_t *D_PWM_SERVO;
    l298n_direction_t L298N_DIRECTION_L;
    l298n_direction_t L298N_DIRECTION_R;
} hc595_control_t;

void HC595_I2SInit();
void HC595I2SPWM_Init(hc595_control_t *control);
void L298N_SetDirection(hc595_control_t *control, l298n_channel_t channel, l298n_direction_t dir);
void L298N_SetPWMDuty(hc595_control_t *control, l298n_channel_t channel, uint8_t percent);
void L298N_Stop(hc595_control_t *control, l298n_channel_t channel);
void L298N_Brake(hc595_control_t *control, l298n_channel_t channel);


#endif