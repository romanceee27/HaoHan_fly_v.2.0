#ifndef _BEEP_H
#define _BEEP_H

#include "sys.h"
#include "delay.h"
#include "includes.h"

#define BEEP_ON     	 GPIOB->BSRRL = GPIO_Pin_9
#define BEEP_OFF     	 GPIOB->BSRRH = GPIO_Pin_9

void Beep_Configuration(void);
void Beep_Show(u8 num);

#endif

