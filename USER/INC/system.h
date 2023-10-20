#ifndef _SYSTEM_H
#define _SYSTEM_H

#include "stm32f4xx.h"
#include "pwm.h"
#include "delay.h"
#include "led.h"
#include "tim4.h"
#include "mpu6050.h"
#include "i2c.h"
#include "delay.h"
#include "system.h"

//uint8_t LED_Scan = 0;
//uint8_t IMU_Scan = 0;
//uint8_t MPU_Scan = 0;
//uint8_t IRQ_Scan = 0;
//uint8_t Batt_Scan = 0;
//uint8_t ANO_Scan = 0;

void nvic_init(void);
void system_init(void);
void task_schedule(void);

#endif
