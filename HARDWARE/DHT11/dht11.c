#include "dht11.h"

static GPIO_InitTypeDef GPIO_InitStructure;/*温湿度模块，连接PG9*/
void dht11_init(void)
{
	// 打开PG9的硬件端口时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	// 配置PG9端口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		 // 第9号引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 // 输出功能模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	 // 推挽输出，增强驱动能力，引脚的输出电流更大
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;	 // 低速模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 没有使用内部上拉电阻
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	PGout(9) = 1;
}

/*读取温湿度模块获取到的信息*/
int32_t dht11_read(uint8_t *buf)
{
	uint32_t t = 0;
	int32_t i = 0, j = 0;
	uint8_t d = 0;
	uint8_t *p = buf;
	uint8_t check_sum = 0;
	int32_t flag = 0;

	// 配置PG9端口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		 // 第9号引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 // 输出功能模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	 // 推挽输出，增强驱动能力，引脚的输出电流更大
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;	 // 低速模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 没有使用内部上拉电阻
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// 根据时序图，PB6当前为低电平
	PGout(9) = 0;
	delay_ms(20);
	PGout(9) = 1;
	delay_us(30);

	// 配置为输入模式
	// 配置PG9端口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		 // 第9号引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 // 输出功能模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	 // 推挽输出，增强驱动能力，引脚的输出电流更大
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;	 // 低速模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 没有使用内部上拉电阻
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// 等待低电平响应信号
	t = 0;
	while (PGin(9))
	{
		t++;
		delay_us(1);
		if (t >= 4000)
			flag = -1;
	}
	// 测量该低电平响应信号是否合法正常是80us的低电平
	t = 0;
	while (PGin(9) == 0)
	{
		t++;
		delay_us(1);
		if (t >= 100)
			flag = -2;
	}
	// 测量该高电平响应信号是否合法正常是80us的高电平
	t = 0;
	while (PGin(9))
	{
		t++;
		delay_us(1);
		if (t >= 100)
			flag = -3;
	}
	for (j = 0; j < 5; j++)
	{
		for (d = 0, i = 7; i >= 0; i--)
		{
			// 等待数据0或者数据1的前置电平结束
			t = 0;
			while (PGin(9) == 0)
			{
				t++;
				delay_us(1);
				if (t >= 100)
					flag = -4;
			}
			// 延时40us （延时时间在28us ~ 70us）
			delay_us(40);

			if (PGin(9))
			{
				d |= 1 << i; // 将d变量对应的bit置1
				// 等待高电平持续完毕
				t = 0;
				while (PGin(9))
				{
					t++;
					delay_us(1);

					if (t >= 100)
						flag = -5;
				}
			}
		}
		p[j] = d; // 得到一个字节（8bit）的数据
	}
	// 延时50us，可以忽略通讯结束的低电平
	delay_us(50);

	// 计算校验和，检查接收到的数据是否准确
	check_sum = (p[0] + p[1] + p[2] + p[3]) & 0xFF;

	if (check_sum == p[4])
		flag = 0;
	else
		flag = -6;
	return flag;
}

