#include "system.h"

void nvic_init(void)
{
//    NVIC_InitTypeDef NVIC_InitStruct;

//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

//    NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;           // TIM4中断通道
//    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级0
//    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;        // 子优先级1
//    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;           // 使能TIM4中断通道
//    NVIC_Init(&NVIC_InitStruct);                           // 中断优先级初始化函数

//    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
//    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
//    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStruct);

//    NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;          // 配置外部中断通道
//    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2; // 设置抢占优先级为0
//    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;        // 设置子优先级为1
//    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;           // 使能外部中断通道
//    NVIC_Init(&NVIC_InitStruct);                           // 中断优先级初始化函数
}

void system_init(void)
{
    nvic_init();
    // led_init();
    // Delay_init();
    MPU_IIC_Init();
    MPU_Init();
}

void task_schedule(void)
{
//    if (ANO_Scan)
//    {
//    }
    if (1)
    {
        Delay_us(5);
        //IMU_Scan = 0;
        prepare_data();
        imu_update(&g_rad, &a_filt, &anglt_real);
    }
}
