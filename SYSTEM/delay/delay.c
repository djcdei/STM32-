#include"delay.h"
void delay_us(uint32_t n)
{
	SysTick->CTRL = 0;			 // 关闭系统定时器
	SysTick->LOAD = n * 168 - 1; // 1us的延时
	SysTick->VAL = 0;			 // 清空current value寄存器和清空count flag标志
	SysTick->CTRL = 5;			 // 使能系统定时器，并使用168MHz作为系统定时器的时钟源
	// while ((SysTick->CTRL & 0x10000)==0);//等待计数值变为0，则count flag是置1，则跳出while循环
	while ((SysTick->CTRL & (1 << 16)) == 0)
		; // 等待计数值变为0，则count flag是置1，则跳出while循环

	SysTick->CTRL = 0; // 关闭系统定时器
}

void delay_ms(uint32_t n)
{
#if 0
	while(n--)
	{
		SysTick->CTRL = 0; // 关闭系统定时器
		SysTick->LOAD = 168000-1; // 1ms的延时
		SysTick->VAL = 0; // 清空current value寄存器和清空count flag标志
		SysTick->CTRL = 5; // 使能系统定时器，并使用168MHz作为系统定时器的时钟源
		//while ((SysTick->CTRL & 0x10000)==0);//等待计数值变为0，则count flag是置1，则跳出while循环
		while ((SysTick->CTRL & (1<<16))==0);//等待计数值变为0，则count flag是置1，则跳出while循环
	}
	
	SysTick->CTRL = 0; // 关闭系统定时器
#else
	while (n--)
	{
		SysTick->CTRL = 0;		   // 关闭系统定时器
		SysTick->LOAD = 21000 - 1; // 1ms的延时
		SysTick->VAL = 0;		   // 清空current value寄存器和清空count flag标志
		SysTick->CTRL = 1;		   // 使能系统定时器，并使用168MHz作为系统定时器的时钟源
		// while ((SysTick->CTRL & 0x10000)==0);//等待计数值变为0，则count flag是置1，则跳出while循环
		while ((SysTick->CTRL & (1 << 16)) == 0)
			; // 等待计数值变为0，则count flag是置1，则跳出while循环
	}
	SysTick->CTRL = 0; // 关闭系统定时器
#endif
}
