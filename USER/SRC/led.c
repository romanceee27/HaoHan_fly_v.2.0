#include "led.h"

void LED_Configuration()
{
    GPIO_Set(GPIOA, PIN15, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE);
    GPIO_Set(GPIOC, PIN10 | PIN11 | PIN12, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    LED_BLUE_OFF;
    LED_GREEN_OFF;
    LED_YELLOW_OFF;
    LED_RED_OFF;
}

void LED_Show(void)
{
    LED_RED_ON;
    OSTimeDly(1000);
    LED_YELLOW_ON;
    OSTimeDly(1000);
    LED_BLUE_ON;
    OSTimeDly(1000);
    LED_GREEN_ON;
    OSTimeDly(1000);
    LED_RED_OFF;
    OSTimeDly(1000);
    LED_YELLOW_OFF;
    OSTimeDly(1000);
    LED_BLUE_OFF;
    OSTimeDly(1000);
    LED_GREEN_OFF;
    OSTimeDly(1000);
}

void LED_OFF(void)
{
    LED_RED_OFF;
    LED_YELLOW_OFF;
    LED_BLUE_OFF;
    LED_GREEN_OFF;
}

void LED_ON(void)
{
    LED_RED_ON;
    LED_YELLOW_ON;
    LED_BLUE_ON;
    LED_GREEN_ON;
}
