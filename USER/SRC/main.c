#include "main.h"

int main(void)

{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	MPU_Init();
	LED_Configuration();
	Delay_init();
	MPU_Init();

	while (1)
	{	
	}
}
