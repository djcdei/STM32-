#include "sr04.h" 
#include <stdio.h>
static GPIO_InitTypeDef GPIO_InitStructure;
/*超声波模块初始化*/
void sr04_init(void)
{
	// 打开端口B的硬件时钟，就是对端口B供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	// 打开端口E的硬件时钟，就是对当前硬件供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	// 配置PB6端口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		 // 第6号引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 // 输出功能模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	 // 推挽输出，增强驱动能力，引脚的输出电流更大
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;	 // 低速模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 没有使用内部上拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 根据时序图，PB6当前为低电平
	PBout(6) = 0;

	// 配置PE6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		 // 第6号引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 // 输入功能模式（既可以软件代码控制，也可以其他外设控制）
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;	 // 低速模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 没有使用内部上拉电阻
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}
/*根据时序图封装超声波获取距离信息的函数*/
int sr04_get_distance(void)
{
	double distance = 0;
	int32_t save = 0;
	int32_t cnt = 0;
	PBout(6) = 1;
	delay_us(10); // 延时10us之后使得PB6电平变为低电平
	PBout(6) = 0;
	// 等待回响信号变为高电平
	while (PEin(6) == 0)
		;

	while (PEin(6) == 1) // 检测回响电平持续时间，通过计算转换为距离
	{
		cnt++;
		delay_us(9);
	}
	distance = (double)(3 * cnt / 2);
	if (distance >= 20 && distance <= 1000)
	{
		printf("distance = %.3lfmm \r\n", distance);
		save = 3; // 危险等级3
	}
	else if (distance > 1000 && distance <= 2000)
	{
		printf("distance = %.3lfmm \r\n", distance);
		save = 2; // 危险等级2
	}
	else
	{
		printf("distance = %.3lfmm \r\n", distance);
		save = 1; // 危险等级1
	}

	return save;
}







