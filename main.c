#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define FLASH_SAVE_ADDR 			ADDR_FLASH_SECTOR_4	//扇区4别名
#define ADDR_FLASH_SECTOR_4 		((u32)0x08010000)	//扇区4起始地址

#define PAin(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOA_BASE + 0x10 - 0x40000000) * 32 + n * 4))
#define PBin(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOB_BASE + 0x10 - 0x40000000) * 32 + n * 4))
#define PEin(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x10 - 0x40000000) * 32 + n * 4))
#define PGin(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOG_BASE + 0x10 - 0x40000000) * 32 + n * 4))
#define PEout(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x14 - 0x40000000) * 32 + n * 4))
#define PFout(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOF_BASE + 0x14 - 0x40000000) * 32 + n * 4))
#define PBout(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOB_BASE + 0x14 - 0x40000000) * 32 + n * 4))
#define PGout(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOG_BASE + 0x14 - 0x40000000) * 32 + n * 4))

#define W25Q128_CS		PBout(14)//片选引脚
#define W25Q128_SCLK	PBout(3)//spi时钟线
#define W25Q128_MOSI	PBout(5)//主出从入
#define W25Q128_MISO	PBin(4)//主入从出

static GPIO_InitTypeDef 		GPIO_InitStructure;
static EXTI_InitTypeDef 		EXTI_InitStructure;
static NVIC_InitTypeDef 		NVIC_InitStructure;
static TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
static RTC_InitTypeDef 			RTC_InitStructure;
static ADC_InitTypeDef       	ADC_InitStructure;
static ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
static TIM_OCInitTypeDef 		TIM_OCInitStructure;// 配置占空比结构体初始化(输出比较)
static USART_InitTypeDef 		USART_InitStructure;
static SPI_InitTypeDef  		SPI_InitStructure;

volatile uint8_t g_usart1_buf[256];
volatile uint8_t g_usart3_buf[256];
volatile uint8_t g_time_buf[32];		//记录当前时间RTC实时时钟
volatile uint8_t g_temp_buf[32];		//记录当前温湿度

volatile uint32_t g_usart1_cnt = 0;	//用于usart1数据接收计数
volatile uint32_t g_usart3_cnt = 0;
volatile uint32_t tim14_cnt = 0;	// 记录定时器14的计数值，用于配置占空比
volatile uint32_t tim1_cnt = 0;		//记录定时器1的计数值，用于配置占空比
volatile uint32_t g_usart1_event = 0;
volatile uint32_t g_usart3_event = 0;
volatile uint32_t g_tim3_event = 0;				
volatile uint32_t g_ble_connect_event = 0;		//蓝牙连接事件
volatile uint32_t g_feed_event = 0;				//看门狗复位事件
volatile uint32_t g_rtc_wakup_event = 0;		//RTC唤醒事件
volatile uint32_t g_rtc_alarm_event = 0;		//RTC闹钟A事件
volatile uint32_t g_rtc_reset = 0;				//RTC时间计数复位标志
uint8_t redata; // 存储接收的数据
uint8_t dht11_buf[5]; // 存储温湿度模块读取到的数据

int32_t dht11_read_ret;//温湿度读取函数返回值
typedef struct __flash_t
{
	uint8_t g_flash_buf[64];//记录时间和温湿度拼接起来的字符数组
	uint32_t offset;//结构体在扇区中的偏移数
}flash_t;
flash_t flash_buf;//放入flash扇区4的结构体

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

void tim4_init(void)		//监测串口1和串口3的数据是否发送完毕
{
	// 使能tim4的硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	// 配置tim4的分频值、计数值
	// tim4硬件时钟=84MHz/8400=10000Hz，就是进行10000次计数，就是1秒时间的到达
	TIM_TimeBaseStructure.TIM_Period = 10000 / 1000 - 1; // 计数值10,0 -> 9就是1毫秒时间的到达
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;		 // 预分频值8400
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;		 // 时钟分频，当前是没有的，不需要进行配置
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	// 配置tim4的中断
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	// 使能tim4
	TIM_Cmd(TIM4, ENABLE);
}

void tim3_init(void)
{
	// 使能tim3的硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	// 配置tim3的分频值、计数值
	// tim3硬件时钟=84MHz/8400=10000Hz，就是进行10000次计数，就是1秒时间的到达
	TIM_TimeBaseStructure.TIM_Period = 10000 / 20 - 1; // 计数值500,0 -> 499就是50毫秒时间的到达
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;	   // 预分频值8400
	//TIM_TimeBaseStructure.TIM_ClockDivision = 0;	   // 时钟分频，当前是没有的，不需要进行配置
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	// 配置tim3的中断
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


// 初始化定时器1
void tim1_init(void)
{
	// 打开硬件电源时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	/* 配置PE13 为复用模式*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; // 第13引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			 // 复用功能模式（既可以软件代码控制，也可以其他外设控制）
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 // 推挽输出，增强驱动能力，引脚的输出电流更大
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		 // 引脚的速度最大为100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		 // 没有使用内部上拉电阻

	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// 将PE13/14引脚连接到定时器1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	
	// TIM1的定时配置
	// TIM1的硬件时钟频率 = 84 * 2 = 168MHz
	TIM_TimeBaseStructure.TIM_Period = (10000 / 100) - 1; // 计数值0~99，决定输出频率为100hz
	TIM_TimeBaseStructure.TIM_Prescaler = 16800 - 1;	  // 当前的预分频值为16800，分频之后变成10000hz
	// TIM_TimeBaseStructure.TIM_ClockDivision = 0;//不支持时钟分频，也称之为2次分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数，0~TIM_Period
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: TIM1 Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			  // 模式1：只要TIMx_CNT(计数值)<TIMx_CCR1(比较值),通道1便为有效状态，否则为无效状态
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 打开/关闭脉冲输出
	TIM_OCInitStructure.TIM_Pulse = 50;							  // 比较值 Capture Compare Register
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  // 有效状态设置为高电平

	// 配置定时器1的通道3
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	// 使能TIM1的计数器开始工作
	TIM_CtrlPWMOutputs(TIM1, ENABLE); // 高级定时器1/8特有的函数

	TIM_Cmd(TIM1, ENABLE);
}

// 初始化定时器14
void tim14_init(void)
{
	// 打开硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	/* 配置PF9 PF10为输出模式，让这根引脚具有输出高低电平的功能 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		   // 第9号引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // 复用功能模式（既可以软件代码控制，也可以其他外设控制）
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出，增强驱动能力，引脚的输出电流更大
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 引脚的速度最大为100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 没有使用内部上拉电阻

	GPIO_Init(GPIOF, &GPIO_InitStructure);

	// 将PF9引脚连接到定时器14
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_TIM14);

	// TIM14的定时配置
	// TIM14的硬件时钟频率 = 42 * 2 = 84MHz
	TIM_TimeBaseStructure.TIM_Period = (10000 / 100) - 1; // 计数值0~99，决定输出频率为100hz
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;		  // 当前的预分频值为8400，分频之后变成10000hz
	// TIM_TimeBaseStructure.TIM_ClockDivision = 0;//不支持时钟分频，也称之为2次分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数，0~TIM_Period
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: TIM14 Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			  // 模式1：只要TIMx_CNT(计数值)<TIMx_CCR1(比较值),通道1便为有效状态，否则为无效状态
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 打开/关闭脉冲输出
	TIM_OCInitStructure.TIM_Pulse = 50;							  // 比较值 Capture Compare Register
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  // 有效状态设置为高电平

	// 配置定时器14的通道1
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);

	// 使能TIM14的计数器开始工作
	TIM_Cmd(TIM14, ENABLE);
}

// 封装函数方便设置TIM14的输出脉冲频率
void tim14_set_freq(uint32_t freq)
{
	/* 关闭TIM14 */
	TIM_Cmd(TIM14, DISABLE);

	/*定时器的基本配置，用于配置定时器的输出脉冲的频率为 freq Hz */
	TIM_TimeBaseStructure.TIM_Period = (10000 / freq) - 1; // 设置定时脉冲的频率
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;		   // 第一次分频，简称为预分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	tim14_cnt = TIM_TimeBaseStructure.TIM_Period; // 记录TIM14定时器的计数值，方便设置比较值，得到需要的占空比
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

	/* 使能TIM14 */
	TIM_Cmd(TIM14, ENABLE);
}

// 封装函数方便设置TIM14的输出脉冲频率
void tim1_set_freq(uint32_t freq)
{
	/* 关闭TIM1 */
	TIM_Cmd(TIM1, DISABLE);

	/*定时器的基本配置，用于配置定时器的输出脉冲的频率为 freq Hz */
	TIM_TimeBaseStructure.TIM_Period = (10000 / freq) - 1; // 设置定时脉冲的频率
	TIM_TimeBaseStructure.TIM_Prescaler = 16800 - 1;	   // 第一次分频，简称为预分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	tim1_cnt = TIM_TimeBaseStructure.TIM_Period; // 记录TIM1定时器的计数值，方便设置比较值，得到需要的占空比
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* 使能TIM1 */
	TIM_Cmd(TIM1, ENABLE);
}

// 封装函数方便设置占空比,通过TIM14当前的计数值，设置对应的比较值
void tim14_set_duty(uint32_t duty)
{
	uint32_t cmp;
	cmp = (tim14_cnt + 1) * duty / 100;
	TIM_SetCompare1(TIM14, cmp);
}
// 封装函数方便设置占空比,通过TIM1当前的计数值，设置对应的比较值
void tim1_set_duty(uint32_t duty)
{
	uint32_t cmp;
	cmp = (tim1_cnt + 1) * duty / 100;
	TIM_SetCompare3(TIM1, cmp); // 设置定时器通道3的比较值
}

/*外部中断初始化*/
void exti_init(void)
{
	// 打开端口A的硬件时钟，就是对端口A供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// 打开端口E的硬件时钟，就是对当前硬件供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	// 打开端口G的硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 // 输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; // 最高速度
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使能内部上下拉电阻
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		 // 端口的第2根引脚
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		 // 端口的第5根引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed; // 
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;		 // 端口的第14根引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 // 输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; // 最高速度
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使能内部上下拉电阻
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//PA0（按键0）连接到EXTI0
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	//PE2（按键1）连接到EXIT2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
	//PE5 连接到EXIT5
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource5);
	//配置外部中断0,2
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		// 中断触发
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 检测按键的按下（下降沿触发）
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				// 使能
	EXTI_Init(&EXTI_InitStructure);
	//配置外部中断5
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //双边沿触发
	EXTI_Init(&EXTI_InitStructure);
	
	//NVIC管理EXTI0、EXIT2、EXIT5，接纳其中断请求和配置它的优先级
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			 // EXTI0的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F; // 抢占优先级：0x0F
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;		 // 响应（子）优先级：0x0F
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 // 允许接纳其中断请求，打开该通道
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			 // 指定是EXTI2的中断请求通道，填写的是中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F; // 抢占优先级 0x1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;		 // 响应优先级 0x2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 // 使能EXTI2中断请求通道
	NVIC_Init(&NVIC_InitStructure);
	
	/* 通过NVIC管理外部中断9~5的中断请求：中断号、优先级、中断打开/关闭 */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;//响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//中断打开
	NVIC_Init(&NVIC_InitStructure);	
}

// 串口USART1的初始化配置，普通串口通信,（跟串口调试助手通信）
void Init_Usart1(uint32_t baud)
{
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable USART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Configure USART Tx and Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 端口复用模式
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使能上下拉电阻
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 复用引脚A9 / A10的功能，配置为USART1_TX / USART1_RX （查阅硬件原理图）
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	//串口1初始化
	USART_InitStructure.USART_BaudRate = baud; // 设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// Init USART1
	USART_Init(USART1, &USART_InitStructure);

	// 若需要中断，则配置中断相关参数，并添加进嵌套中断向量控制器
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 设置为:接收数据寄存器不空中断（参数2）

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		  // 指定USART1_IQR通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // 响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // 使能中断请求通道
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);
}

// 串口USART3的初始化配置，用于蓝牙通信
void Init_Usart3(uint32_t baud)
{
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* Configure USART Tx and Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 端口复用
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使能上下拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 复用引脚PB10 / PB11的功能，配置为USART3_TX / USART3_RX （查阅硬件原理图）
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	USART_InitStructure.USART_BaudRate = baud; // 设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// Init USART3
	USART_Init(USART3, &USART_InitStructure);
	/* Enable USART */
	USART_Cmd(USART3, ENABLE);

	// 若需要中断，则配置中断相关参数
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 设置为:接收数据寄存器不空中断（参数2）

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		  // 指定USART3_IQR通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // 响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // 使能中断请求通道
	NVIC_Init(&NVIC_InitStructure);
}




RTC_TimeTypeDef  		RTC_TimeStructure;
RTC_DateTypeDef 		RTC_DateStructure;
RTC_AlarmTypeDef 		RTC_AlarmStructure;
/*RTC实时时钟初始化，启动唤醒中断功能*/
void rtc_wakeup_init(void)				
{
	 /* Enable the PWR clock ,使能电源时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Allow access to RTC ，允许访问RTC*/
	PWR_BackupAccessCmd(ENABLE);
	
	/*使能LSI时钟（32KHz）*/
	RCC_LSICmd(ENABLE);
	/*等待LST时钟使能成功 Wait till LSI is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
	/*选择时钟源LSI*/
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	/*配置RTC输出频率1Hz-->设置预分频值*/
	/* RTC时钟计算方式ck_spre(1Hz) = RTCCLK(LSE 32Khz) /(uwAsynchPrediv + 1)/(uwSynchPrediv + 1)*/
	/* Enable the RTC Clock ，使能RTC时钟*/
	RCC_RTCCLKCmd(ENABLE);
	/* Wait for RTC APB registers synchronisation ，等待RTC相关寄存器就绪*/
	RTC_WaitForSynchro();

	/* Configure the RTC data register and RTC prescaler，配置RTC数据寄存器与RTC的分频值 */
	RTC_InitStructure.RTC_AsynchPrediv = 0x7F;		//异步分频系数 128-1
	RTC_InitStructure.RTC_SynchPrediv = 0xF9;		//同步分频系数 250-1
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;//24小时格式
	RTC_Init(&RTC_InitStructure);
	
	if(g_rtc_reset==0)
	{	/* Set the time to 14h 59mn 53s PM */
		RTC_TimeStructure.RTC_H12     = RTC_H12_PM;
		RTC_TimeStructure.RTC_Hours   = 0x05;
		RTC_TimeStructure.RTC_Minutes = 0x20;
		RTC_TimeStructure.RTC_Seconds = 0x25; 
		RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure); 
	   /* Set the date: Friday January 11th 2013 */
		RTC_DateStructure.RTC_Year = 0x13;
		RTC_DateStructure.RTC_Month = RTC_Month_January;
		RTC_DateStructure.RTC_Date = 0x11;
		RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Saturday;
		RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure);
	}

	
	//关闭唤醒功能
	RTC_WakeUpCmd(DISABLE);	   

	//为唤醒功能选择RTC配置好的时钟源
	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
	
	//设置唤醒计数值为自动重载，写入默认值为0
	RTC_SetWakeUpCounter(1-1);
	//清除RTC唤醒中断标志
	RTC_ClearITPendingBit(RTC_IT_WUT);
	//使能RTC唤醒定时器中断，就是计数1次完成后就会触发一次中断
	RTC_ITConfig(RTC_IT_WUT,ENABLE);
	
	RTC_WakeUpCmd(ENABLE);
	
	/*使能唤醒功能,配置外部中断线22*/
	/* Configure EXTI Line22，配置外部中断控制线22 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line22;			//当前使用外部中断控制线22
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//上升沿触发中断 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;			//使能外部中断控制线22
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;		//允许RTC唤醒中断触发
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//抢占优先级为0x3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		//响应优先级为0x3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能
	NVIC_Init(&NVIC_InitStructure);	
	
}

/*自己封装设置动态设置闹钟的函数*/
void rtc_alarm_init(uint8_t Hours,uint8_t Minutes,uint8_t Seconds,uint8_t AlarmDateWeekDay)
{
	//关闭闹钟功能
	RTC_AlarmCmd(RTC_Alarm_A,DISABLE); 
	/* Set the alarm 05h:20min:30s 设置闹钟*/
	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = Hours;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = Minutes;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = Seconds;
	RTC_AlarmStructure.RTC_AlarmDateWeekDay = AlarmDateWeekDay;
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_None;
	
	//使能闹钟功能
	/* Configure the RTC Alarm A register 配置为A闹钟*/
	RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);

	/* Enable RTC Alarm A Interrupt 使能闹钟中断*/
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);

	/* Enable the alarm 使能闹钟A*/
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

	EXTI_InitStructure.EXTI_Line = EXTI_Line17;			//当前使用外部中断控制线17
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//上升沿触发中断 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;			//使能外部中断控制线17
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;		//允许RTC唤醒中断触发
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//抢占优先级为0x3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//响应优先级为0x3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能
	NVIC_Init(&NVIC_InitStructure);	
	
}

/*ADC3初始化，监测光敏电阻电压值*/
void adc_init(void)
{
	/*使能硬件时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	/*配置GPIOF7引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	/* ADC Common Init */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//单个ADC工作
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;//ADC硬件时钟频率=84MHz/2=42MHz
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;//关闭DMA模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样点的间隔时间=5*（1/42MHz）
	ADC_CommonInit(&ADC_CommonInitStructure);
	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//分辨率12bit，分辨率越高，得到数据更准确，灵敏度更高
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//ADC1只转换1个通道
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//ADC连续不断的进行转换；若是DISABLE就转换一次
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//不需要边沿触发工作，使用软件触发ADC工作
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐存储
	ADC_InitStructure.ADC_NbrOfConversion = 1;//只转换1个通道，恒为1；如果ADC转换多个通道，请指定数量
	ADC_Init(ADC3, &ADC_InitStructure);
	
	/* ADC3 regular channel 5 configuration，顺序为1，采样时间为3*(1/42MHz)*/
	ADC_RegularChannelConfig(ADC3, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);

	ADC_Cmd(ADC3, ENABLE);
	ADC_SoftwareStartConv(ADC3);//通过软件启动ADC3的转换
}

/*独立看门狗初始化*/
void iwdg_init(void)
{
	//允许访问独立看门狗
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	//配置独立看门狗的预分频值为256
	//得到独立看门狗硬件时钟为32KHz/256=125Hz
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	
	//设置独立看门狗的独立计数值为 0 ~124
	IWDG_SetReload(125-1);//也就是1s时间到达
	
	//重载（刷新）计数值
	IWDG_ReloadCounter();
	
	//使能独立看门狗工作
	IWDG_Enable();
}

/*flash初始化（擦除）方便写入数据*/
void flash_init(void)
{
	//解锁FLASH，才能改写数据（擦除、编程）
	FLASH_Unlock();
	//擦除扇区4，单位时间内擦除32bit
	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
					FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);	
	
	FLASH_EraseSector(FLASH_Sector_4,VoltageRange_3);
	
	//锁定FLASH，只能读，不能写，也不能擦除
	FLASH_Lock(); 
}


/*模拟SPI flash（W25Q128）初始化*/
void w25qxx_simulate_init(void)
{
	//使能硬件PB4(看原理图)时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	//配置PB3( /CS ),PB5(MOSI),PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 	//	配置为输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; 	// 	最高速度
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//	推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// 	不使能内部上下拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//配置PB4(MISO)为输入模式，作为MISO引脚读取数据
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;			//	引脚PB4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 	//	输入模式
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//工作在模式3，时钟线引脚PB3初始电平为高电平
	W25Q128_SCLK = 1;
	W25Q128_MOSI = 1;
	W25Q128_CS = 1;//拉高片选引脚
}
/*SPI flash（W25Q128）初始化*/
void w25qxx_init(void)
{
	
#if 0
	//使能硬件SPI1和PB4(看原理图)时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	//配置PB3,4,5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	 	//复用模式（引脚既可以被硬件控制，也可以被软件控制）
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; 	// 最高速度
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// 不使能内部上下拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 将PB3、4、5引脚分别复用为SPI1_SCK、SPI1_MISO、SPI1_MOSI
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
	//配置PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;//引脚PB14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 	//输出模式
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	PBout(14)=1;//拉高
	
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//SPI一般使用双线全双工，SPI总线能高速跟多个设备通信，速度快
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//M4工作在主机角色
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//看时序图，以最小的字节单元
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							//看时序图，CPOL=1，SPI时钟线空闲时为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						//看时序图,	CPHA=1,MISO设置为第二边沿（在这里为上升沿）采样数据
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//通过软件代码控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;	//看从机的数据手册，不能超速，SCLK的频率=84MHz/16=5.25MHz
	
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//看从机的数据手册
	SPI_Init(SPI1, &SPI_InitStructure);
	//使能SPI1工作
	SPI_Cmd(SPI1, ENABLE);
#else
	w25qxx_simulate_init();//模拟spi初始化
#endif
}



/*自己封装SPI发送数据函数（同步通信，发送完一个字节后同时也会返回数据）*/
uint8_t SPI1_SendByte(uint8_t txd)
{
#if 0
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI1, txd);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
#else
	int32_t i;
	uint8_t recv_data = 0;
	
	//循环8位bit的数据，逐位发送(高位先发)
	for(i=7;i>=0;i--)
	{
		//判断该bit位是否为高电平
		if(txd &(1<<i))
		{
			W25Q128_MOSI=1;//MOSI设置为高电平
		}
		else
		{
			W25Q128_MOSI=0;
		}
			
		//设置时钟线为低电平，MOSI引脚输出数据
		W25Q128_SCLK=0;
		delay_us(1);
		//输出完数据后同时进行数据的接收（高位先收），就要拉高时钟线
		W25Q128_SCLK=1;//
		delay_us(1);
		if(W25Q128_MISO)
		{
			recv_data |= (1<<i);
		}	
	}
	return recv_data;
#endif
}

/*读取spi 设备id和厂商id数据*/
void w25qxx_read_id(uint8_t *m_id,uint8_t *d_id)
{
	//片选引脚PB14输出低电平，从机开始工作
	PBout(14)=0;
	//发送0x90(看时序图90h)
	SPI1_SendByte(0x90);
	//发送24bit地址，分三次发送
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);
	
	//传递任意参数获取厂商id
	*m_id = SPI1_SendByte(0xFF);
	*d_id = SPI1_SendByte(0xFF);
	//通信结束将CS（PB14）引脚拉高
	PBout(14)=1;
}

//spi写使能 WRITE ENABLE
void w25qxx_write_enable(void)
{
	//cs引脚拉低，从机开始工作
	W25Q128_CS=0;
	//发送0x06
	SPI1_SendByte(0x06);
	//CS引脚为高电平，从机停止工作
	W25Q128_CS=1;
}
//spi写不使能 WRITE DISABLE
void w25qxx_write_disable(void)
{
	W25Q128_CS=0;
	SPI1_SendByte(0x04);
	W25Q128_CS=1;
}

//读寄存器 Read Status Register的状态
uint8_t read_status_register(void)
{
	uint8_t sta;
	W25Q128_CS=0;
	SPI1_SendByte(0x05);
	//发送任意8位bit数据，获取寄存器状态
	sta = SPI1_SendByte(0xFF);
	W25Q128_CS=1;
	return sta;	
}

//spi flash扇区擦除(sector erase)
void w25qxx_sector_erase(uint32_t sector_addr)//选择扇区地址
{
	uint8_t sta;
	//写使能
	w25qxx_write_enable();
	delay_us(1);
	W25Q128_CS=0;//从机开始工作
	//发送0x20扇区擦除指令
	SPI1_SendByte(0x20);
	//发送24bits的地址数据
	SPI1_SendByte((sector_addr >> 16) & 0xFF);
	SPI1_SendByte((sector_addr >> 8) & 0xFF);
	SPI1_SendByte(sector_addr & 0xFF);
	//监测read status register寄存器的BUSY 比特位是否为0（表示数据擦除完毕）
	while(1)
	{
		sta=read_status_register();
		if((sta & 0x01)==0x00)
			break;
		delay_us(1);
	}
		
	w25qxx_write_disable();
}

//spi flash页编程(page program)参数：扇区首地址、数据首地址、数据长度
void w25qxx_page_program(uint32_t page_addr,uint8_t *buf,uint32_t len)
{
	uint8_t sta;
	uint8_t *p=buf;
	//写使能
	w25qxx_write_enable();
	//发送0x02页编程指令
	SPI1_SendByte(0x02);
	//发送24bits的地址数据
	SPI1_SendByte((page_addr >> 16) & 0xFF);
	SPI1_SendByte((page_addr >> 8) & 0xFF);
	SPI1_SendByte(page_addr & 0xFF);
	
	//判断len是否超过256.超过则要进行新一轮页编程
	len = len > 256 ? 256 : len;
	//逐位写入buf中的数据
	while(len--)
	{
		SPI1_SendByte(*p);
		p++;
	}
	
	//CS引脚为高电平，从机停止工作
	W25Q128_CS=1;

	delay_us(1);
	//监测read status register寄存器的BUSY 比特位是否为0（表示数据擦除完毕）
	while(1)
	{
		sta=read_status_register();
		if((sta & 0x01)==0x00)
			break;
		delay_us(1);
	}
	w25qxx_write_disable();
}
//读取spi数据根据W25Q128S资料手册26页时序图编写,参数：起始地址、存放读取数据的缓冲区，要读取的数据长度
void w25qxx_read_data(uint32_t addr,uint8_t *buf,uint32_t len)
{
	uint8_t *p = buf;
	
	//CS引脚为低电平，从机开始工作
	W25Q128_CS=0;
	
	//先发送0x03 读取数据指令
	SPI1_SendByte(0x03);

	//发送24bit地址（高地址先发送）巧妙的方法
	SPI1_SendByte((addr>>16)&0xFF);
	SPI1_SendByte((addr>>8)&0xFF);
	SPI1_SendByte(addr&0xFF);
	
	while(len--)
	{
		//每次读取1byte的数据
		*p=SPI1_SendByte(0xFF);
		p++;
	}
	//CS引脚为高电平，从机停止工作
	W25Q128_CS=1;	
}


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
/*温湿度模块，连接PG9*/
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
	// 测量该低电平响应信号是否合法
	t = 0;
	while (PGin(9) == 0)
	{
		t++;
		delay_us(1);
		if (t >= 100)
			flag = -2;
	}
	// 测量该高电平响应信号是否合法
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
		save = 2; // 危险等级3
	}
	else
	{
		printf("distance = %.3lfmm \r\n", distance);
		save = 1; // 危险等级3
	}

	return save;
}

/*自己封装的发送字符流函数*/
void my_USART_SendData(USART_TypeDef *USARTx, uint8_t *data)
{
	while (*data != '\0')
	{
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
			; // 判断发送数据缓冲区是否为空
		USART_SendData(USARTx, *data);
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
			; // 判断数据是否发送完成
		data++;
	}
}

struct __FILE
{
	int handle; /* Add whatever you need here */
};
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	// while((USART1->SR & 0x01<<7) == (uint16_t)RESET);
	return (ch);
}

//将int类型的整数（0~59）转化为8位16进制(例如：23->0x23)
uint8_t to_hex(uint32_t num)
{
    uint8_t hex_num = 0;
    
    hex_num = (num / 10) * 16 + (num % 10);//仅支持2位十进制整数转换
	
    return hex_num;
}

//滤波算法得到更平滑的ADC转换数据
uint32_t filter_out(void)
{
	uint32_t adc_val;	//adc转换得到的12bit二进制值
	uint32_t adc_vol;	//将二进制值转换成的实际电压值
	uint32_t i;		//一个计数值
	for(adc_val=0,i=0;i<1000;i++)
	{
		while(ADC_GetFlagStatus(ADC3,ADC_FLAG_EOC)==RESET);
		ADC_ClearFlag(ADC3,ADC_FLAG_EOC);
		
		adc_val+=ADC_GetConversionValue(ADC3);
	}
	adc_val/=1000;//多次测量求平均值
	adc_vol=adc_val*3300/4095;
	printf("adc_vol=%dmv\r\n",adc_vol);
	return adc_vol;
}

/*自己封装的将数据写入flash的函数*/
void flash_write(flash_t *flash_write_buf )
{
	int i = 0;
	FLASH_Unlock();

	for(i = 0; i<sizeof(flash_t)/4;i++)//按字写入,一个字4个字节，所以要除以4
	{
		FLASH_ProgramWord(0x08010000+flash_write_buf->offset * sizeof(flash_t) + i*4, *((uint32_t *)flash_write_buf + i));
	}
	
	FLASH_Lock(); 
}

void flash_read(uint32_t num)
{
	int i = 0;
	volatile uint32_t *p ;
	flash_t flash_read_buf;
	FLASH_Unlock();
	
	for(i=0;i<num;i++)
	{
		flash_read_buf = *((volatile flash_t*)(0x08010000 + i * sizeof(flash_t)));	
		p=(volatile uint32_t *)((0x08010000 + i * sizeof(flash_t)));
		if(*p==0xffffffff)
		{
			printf("%08x\r\n:",*p);
		}
		else
			my_USART_SendData(USART3,flash_read_buf.g_flash_buf);
	}
	
	FLASH_Lock();
}

//计算flash扇区4还未存放数据的地址到首地址的偏移量
uint32_t flash_read_offset(uint32_t *start_addr)
{
	volatile uint32_t *p = start_addr;
	uint32_t count = 0;	
	FLASH_Unlock();
	while(*p != 0xffffffff)
	{
		count++;
		p = (uint32_t *)((uint8_t *)p + sizeof(flash_t));
	}
	FLASH_Lock();
	return count;//返回当前数据块
}


int main(void)
{
	char *p;			//一个指针
	uint8_t date;		//闹钟：日
	uint8_t hours;		//闹钟：时
	uint8_t minutes;	//闹钟：分
	uint8_t seconds;	//闹钟：秒
	uint8_t d_id;		//spi设备id
	uint8_t m_id;		//spi flash厂商id
	uint32_t duty;		//PWM占空比	
	int32_t distance_save = 0;//超声波测距安全等级
	int32_t i;					//一个计数值
	uint8_t w25qxx_buf[64]={0};
	// int pwm_cmp;
	// 打开端口硬件时钟，就是对当前硬件供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	/* 配置PF10为输出模式，让这根引脚具有输出高低电平的功能 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;		   // 第10号引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 输出功能模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出，增强驱动能力，引脚的输出电流更大
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 引脚的速度最大为100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 没有使用内部上拉电阻
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	exti_init();
	tim1_init();
	tim3_init();
	tim4_init();
	tim14_init();
	tim14_set_freq(100);
	tim1_set_freq(100);
	
	w25qxx_init();//SPI总线（w25qxx）初始化
	
	adc_init();	//ADC初始化
	iwdg_init();//独立看门狗初始化
	sr04_init();  // 超声波模块初始化
	dht11_init(); // 温湿度模块初始化
	
	delay_ms(100);
	Init_Usart1(115200);//串口1初始化，波特率115200
	delay_ms(100);
	Init_Usart3(9600);//串口3初始化，连接蓝牙模块通信，波特率9600
	
	PFout(9) = PFout(10) = PEout(13) = PEout(14) = 1;
	
	my_USART_SendData(USART3, "------串口测试开始------\r\n");
	my_USART_SendData(USART3, "------发送  start 					启动温湿度数据记录模式------\n");
	my_USART_SendData(USART3, "------发送  show 					显示当前所有的温湿度记录------\n");
	my_USART_SendData(USART3, "------发送  stop 					停止温湿度记录------\n");
	my_USART_SendData(USART3, "------发送  reset 					擦除所有的温湿度记录------\n");
	my_USART_SendData(USART3, "------发送  Alarm set-d-h-m-s# 		设置闹钟A响铃时间------\n");
	

	if (RTC_ReadBackupRegister(RTC_BKP_DR0) != 4455)
	{
		rtc_wakeup_init();//实时时钟RTC初始化
		flash_init();	//清空扇区4
		RTC_WriteBackupRegister(RTC_BKP_DR0, 4455);
	}
	else
	{
		g_rtc_reset=1;//禁止RTC时间复位
		rtc_wakeup_init();//实时时钟RTC初始化
	}
	
	w25qxx_read_id(&m_id,&d_id);//获取spi设备id和厂商id
	printf("m_id:%x,d_id:%x\r\n",m_id,d_id);
	w25qxx_sector_erase(0);//擦除扇区0
	memset(w25qxx_buf,'a',sizeof(w25qxx_buf));
	//向0地址写入w25qxx_buf里面的数据
	w25qxx_page_program(0,w25qxx_buf,sizeof(w25qxx_buf));
	memset(w25qxx_buf,0,sizeof(w25qxx_buf));
	//从0地址开始读取数据
	w25qxx_read_data(0,w25qxx_buf,sizeof(w25qxx_buf));
	printf("read addr at 0:\r\n");
	
	for(i=0; i<64; i++)
	{
		printf("%c ",w25qxx_buf[i]);
	}
	
	printf("\r\n");
	
	
	while (1)
	{
		filter_out();
		if (g_usart1_event)//串口1发送数据完成，开始解析
		{
			printf("usart1 recv data:%s\r\n", g_usart1_buf);
			my_USART_SendData(USART3, (uint8_t*)g_usart1_buf);
			// my_USART_SendData(USART3,g_usart1_buf);
			g_usart1_event = 0;
			g_usart1_cnt = 0;
			memset((void *)g_usart1_buf, 0, sizeof g_usart1_buf);
		}
		
		if (g_usart3_event)//蓝牙发送数据完成，开始解析
		{
			printf("usart3 recv data:%s\r\n", g_usart3_buf);
			if (strstr((char *)g_usart3_buf, "duty"))
			{
				p = strtok((char *)g_usart3_buf, "=");
				p = strtok(NULL, "=");
				duty = atoi(p);
				printf("atoi(p):%d\n", duty); // 把数字字符（const char *）转化为数字整形（int）
				tim14_set_duty(duty);
			}
			//启动温湿度数据记录模式
			if(strstr((char *)g_usart3_buf,"temp?#")||strstr((char *)g_usart3_buf,"start"))
			{
				my_USART_SendData(USART3,"---开始温湿度记录---\r\n");
				g_tim3_event=1;
			}
			if(strstr((char *)g_usart3_buf,"Alarm set-"))
			{
				printf("设置时间\n");
				//截取 日-时-分-秒
				p=strtok((char *)g_usart3_buf, "-");
				//printf("p:%s\r\n",p);
				date=to_hex(atoi(strtok(NULL,"-")));
				//printf("date:%d\r\n",date);
				hours=to_hex(atoi(strtok(NULL,"-")));
				//printf("hours:%d\r\n",hours);
				minutes=to_hex(atoi(strtok(NULL,"-")));
				//printf("minutes:%d\r\n",minutes);
				seconds=to_hex(atoi(strtok(NULL,"#")));
				//printf("seconds:%d\r\n",seconds);
				rtc_alarm_init(hours,minutes,seconds,date);//设置时间	
			}
			if(strstr((char *)g_usart3_buf,"stop"))//启动温湿度数据记录模式
			{
				my_USART_SendData(USART3,"---停止温湿度记录---\r\n");
				g_tim3_event=0;
			}
			if(strstr((char *)g_usart3_buf,"show"))
			{
				my_USART_SendData(USART3,"---查看温湿度记录---\r\n");
				flash_read(10);
			}
			if(strstr((char *)g_usart3_buf,"reset"))
			{
				my_USART_SendData(USART3,"---清空flash扇区4成功---\r\n");
				flash_init();
			}
			g_usart3_event = 0;
			g_usart3_cnt = 0;
			memset((void *)g_usart3_buf, 0, sizeof g_usart3_buf);
		}
		
		if (g_tim3_event)//触发测量温度事件，
		{	
			dht11_read_ret= dht11_read(dht11_buf);
			if(dht11_read_ret==0)
			{
				sprintf((char *)g_temp_buf,"Temp=%d.%d Humi=%d.%d",dht11_buf[2], dht11_buf[3], dht11_buf[0], dht11_buf[1]);
				
				if(dht11_buf[2]>=29)
				{
					my_USART_SendData(USART3,"---温度超过阈值,停止喂狗---\r\n");
					g_feed_event=1;
				}	
			}				
			else
			{
				printf("dht11 read error code is %d\r\n", dht11_read_ret);
			}
	
			PFout(10) ^= 1;
			delay_ms(6000);//延时6秒，标志位不复位，每6秒记录一次
			PFout(10) ^= 1;
		}
		
		if(g_ble_connect_event)//蓝牙连接事件
		{
			if(PEin(5))
			{
				PEout(14)=0;
				printf("ble connect\r\n");
			}
			else
			{
				PEout(14)=1;
			
				printf("ble disconnect\r\n");			
			}
		
			g_ble_connect_event=0;
		}
		
		if(g_rtc_wakup_event)//RTC唤醒事件
		{
			//获取时间
			RTC_GetTime(RTC_Format_BCD,&RTC_TimeStructure);
			//获取日期
			RTC_GetDate(RTC_Format_BCD, &RTC_DateStructure);
		
			if(g_tim3_event)
			{
				flash_buf.offset=flash_read_offset((uint32_t *)(0x08010000));
				if(flash_buf.offset>=10)
				{
					printf("---擦除扇区成功---\r\n");
					flash_buf.offset=0;
					flash_init();
				}
					
				sprintf((char *)(flash_buf.g_flash_buf),"[%03d]20%02x/%02x/%02x Week:%x %02x:%02x:%02x %s\r\n",flash_buf.offset,
					RTC_DateStructure.RTC_Year,
					RTC_DateStructure.RTC_Month,
					RTC_DateStructure.RTC_Date,
					RTC_DateStructure.RTC_WeekDay,
					RTC_TimeStructure.RTC_Hours,
					RTC_TimeStructure.RTC_Minutes,
					RTC_TimeStructure.RTC_Seconds,
					g_temp_buf);
				flash_write(&flash_buf);
				printf("offset=%d\r\ng_flash_buf=%s\r\n",flash_buf.offset,flash_buf.g_flash_buf);
			}
			g_rtc_wakup_event=0;//复位
		}
		
		if(g_rtc_alarm_event)
		{
			my_USART_SendData(USART3,"---闹钟A响了---\r\n");
			g_rtc_alarm_event=0;
		}
		
		distance_save = sr04_get_distance();
		
		switch (distance_save)
		{
		case 0:
			printf("distance error\r\n");
			break;
		case 1:
			printf("安全等级1,安全！\r\n");
			tim1_set_duty(100);
			break;
		case 2:
			printf("安全等级2,注意！！\r\n");
			tim1_set_duty(60);
			break;
		case 3:
			printf("安全等级3,危险！！！\r\n");
			tim1_set_duty(0);
			break;
		default:
			break;
		}

		delay_ms(1000);
	}
}

void EXTI0_IRQHandler(void)
{
	// 检测EXTI0是否有中断请求
	if (EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		/* 关闭定时器3 */
		TIM_Cmd(TIM3, DISABLE);

		/* 清空当前计数值 */
		TIM_SetCounter(TIM3, 0);

		/* 启动定时器3，就算有一些纹波存在，也会重置定时器 */
		TIM_Cmd(TIM3, ENABLE);

		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void EXTI2_IRQHandler(void)
{
	// 检测EXTI2是否有中断请求
	if (EXTI_GetITStatus(EXTI_Line2) == SET)
	{
		/* 关闭定时器3 */
		TIM_Cmd(TIM3, DISABLE);

		/* 清空当前计数值 */
		TIM_SetCounter(TIM3, 0);

		/* 启动定时器3，就算有一些纹波存在，也会重置定时器 */
		TIM_Cmd(TIM3, ENABLE);

		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

void EXTI9_5_IRQHandler(void)//外部中断线5，监测蓝牙连接与断开
{
	/* 检测中断是否有触发 */
	if(EXTI_GetITStatus(EXTI_Line5) == SET)
	{
		/* 添加用户代码 */
		g_ble_connect_event=1;

		/* 清空中断标志位，告诉CPU当前事件已经处理完毕
           思考题：如果不清空标志位，会出现什么现象？
		*/
		EXTI_ClearITPendingBit(EXTI_Line5);
				
	}
}
void TIM3_IRQHandler(void)
{
	// 检测标志位
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		if (PEin(2) == 0)
		{
			g_tim3_event = 1;//按键2按下触发温度测量事件
		}
		TIM_Cmd(TIM3, DISABLE); // 关闭定时器
		// 清空标志位
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

void TIM4_IRQHandler(void)//监测串口1和串口3的数据是否发送完毕
{
	static uint32_t cnt = 0;
	static uint32_t cnt3 = 0;
	if(g_feed_event == 0)
		IWDG_ReloadCounter();//刷新计数值，防止计数值计数到0使得cpu复位（俗称喂狗）
	// 检测标志位
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		if (cnt != g_usart1_cnt)//监测USART1数据是否接收完毕
		{
			cnt = g_usart1_cnt;
		}
		else if ((cnt == g_usart1_cnt) && cnt)
		{
			g_usart1_event = 1; // 通知串口数据发送完毕
			cnt = 0;			// 复位
		}
		
		if (cnt3 != g_usart3_cnt)//监测USART3数据是否接收完毕
		{
			cnt3 = g_usart3_cnt;
		}
		else if ((cnt3 == g_usart3_cnt) && cnt3)
		{
			g_usart3_event = 1;
			cnt3 = 0;
		}
		// 清空标志位
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

// USART1中断服务函数(接收串口发送来的数据)
void USART1_IRQHandler(void) // 切忌不能加printf函数打印，否则消耗时间太多，数据接收不完整
{
	// 检测USART1是否发生了USART_IT_RXNE中断，也就是触发了接收数据中断事件
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		// 添加用户代码
		redata = USART_ReceiveData(USART1);
	
		if (g_usart1_cnt < sizeof(g_usart1_buf))
		{
			g_usart1_buf[g_usart1_cnt++] = redata;
		}

		// 清空标志位
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
 
void USART3_IRQHandler(void)
{
	uint8_t bdata;
	// 检测USART3是否发生了USART_IT_RXNE中断，也就是触发了接收数据中断事件
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		// 添加用户代码
		bdata = USART_ReceiveData(USART3);
		
		if (g_usart3_cnt < sizeof(g_usart3_buf))
		{
			g_usart3_buf[g_usart3_cnt++] = bdata;
		}

		// 清空标志位
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}

//实时时钟RTC唤醒中断
void RTC_WKUP_IRQHandler(void)
{
	//检测标志位
	if(RTC_GetITStatus(RTC_IT_WUT) != RESET)
	{
		//添加用户代码
		g_rtc_wakup_event=1;
	
		//清空标志位	
		RTC_ClearITPendingBit(RTC_IT_WUT);
		EXTI_ClearITPendingBit(EXTI_Line22);

	}
}

//实时时钟RTC闹钟中断
void RTC_Alarm_IRQHandler(void)
{
	//检测标志位（闹钟A）
	if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
	{
		//添加用户代码
		g_rtc_alarm_event=1;
	
		//清空标志位	
		RTC_ClearITPendingBit(RTC_IT_ALRA);
		EXTI_ClearITPendingBit(EXTI_Line17);

	}
}
	
