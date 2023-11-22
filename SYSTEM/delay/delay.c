#include"delay.h"
void delay_us(uint32_t n)
{
	SysTick->CTRL = 0;			 // �ر�ϵͳ��ʱ��
	SysTick->LOAD = n * 168 - 1; // 1us����ʱ
	SysTick->VAL = 0;			 // ���current value�Ĵ��������count flag��־
	SysTick->CTRL = 5;			 // ʹ��ϵͳ��ʱ������ʹ��168MHz��Ϊϵͳ��ʱ����ʱ��Դ
	// while ((SysTick->CTRL & 0x10000)==0);//�ȴ�����ֵ��Ϊ0����count flag����1��������whileѭ��
	while ((SysTick->CTRL & (1 << 16)) == 0)
		; // �ȴ�����ֵ��Ϊ0����count flag����1��������whileѭ��

	SysTick->CTRL = 0; // �ر�ϵͳ��ʱ��
}

void delay_ms(uint32_t n)
{
#if 0
	while(n--)
	{
		SysTick->CTRL = 0; // �ر�ϵͳ��ʱ��
		SysTick->LOAD = 168000-1; // 1ms����ʱ
		SysTick->VAL = 0; // ���current value�Ĵ��������count flag��־
		SysTick->CTRL = 5; // ʹ��ϵͳ��ʱ������ʹ��168MHz��Ϊϵͳ��ʱ����ʱ��Դ
		//while ((SysTick->CTRL & 0x10000)==0);//�ȴ�����ֵ��Ϊ0����count flag����1��������whileѭ��
		while ((SysTick->CTRL & (1<<16))==0);//�ȴ�����ֵ��Ϊ0����count flag����1��������whileѭ��
	}
	
	SysTick->CTRL = 0; // �ر�ϵͳ��ʱ��
#else
	while (n--)
	{
		SysTick->CTRL = 0;		   // �ر�ϵͳ��ʱ��
		SysTick->LOAD = 21000 - 1; // 1ms����ʱ
		SysTick->VAL = 0;		   // ���current value�Ĵ��������count flag��־
		SysTick->CTRL = 1;		   // ʹ��ϵͳ��ʱ������ʹ��168MHz��Ϊϵͳ��ʱ����ʱ��Դ
		// while ((SysTick->CTRL & 0x10000)==0);//�ȴ�����ֵ��Ϊ0����count flag����1��������whileѭ��
		while ((SysTick->CTRL & (1 << 16)) == 0)
			; // �ȴ�����ֵ��Ϊ0����count flag����1��������whileѭ��
	}
	SysTick->CTRL = 0; // �ر�ϵͳ��ʱ��
#endif
}
