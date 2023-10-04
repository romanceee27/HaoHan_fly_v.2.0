#ifndef _LED_H
#define _LED_H

#include "sys.h"
#include "delay.h"
#include "includes.h"

#define LED_RED_OFF GPIOA->BSRRL = PIN15
#define LED_YELLOW_OFF GPIOC->BSRRL = PIN10
#define LED_BLUE_OFF GPIOC->BSRRL = PIN11
#define LED_GREEN_OFF GPIOC->BSRRL = PIN12

#define LED_RED_ON GPIOA->BSRRH = PIN15
#define LED_YELLOW_ON GPIOC->BSRRH = PIN10
#define LED_BLUE_ON GPIOC->BSRRH = PIN11
#define LED_GREEN_ON GPIOC->BSRRH = PIN12

#define LED_RED_TOGGLE GPIOA->ODR ^= PIN15
#define LED_YELLOW_TOGGLE GPIOC->ODR ^= PIN10
#define LED_BLUE_TOGGLE GPIOC->ODR ^= PIN11
#define LED_GREEN_TOGGLE GPIOC->ODR ^= PIN12

void LED_Configuration(void);
void LED_Show(void);
void LED_OFF(void);
void LED_ON(void);

#endif
