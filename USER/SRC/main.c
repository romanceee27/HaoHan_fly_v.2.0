#include "main.h"

int main(void)

{
	system_init();

	while (1)
	{
		task_schedule();
	}
}
