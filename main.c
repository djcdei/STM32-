#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define FLASH_SAVE_ADDR 			ADDR_FLASH_SECTOR_4	//����4����
#define ADDR_FLASH_SECTOR_4 		((u32)0x08010000)	//����4��ʼ��ַ

#define PAin(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOA_BASE + 0x10 - 0x40000000) * 32 + n * 4))
#define PBin(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOB_BASE + 0x10 - 0x40000000) * 32 + n * 4))
#define PEin(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x10 - 0x40000000) * 32 + n * 4))
#define PGin(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOG_BASE + 0x10 - 0x40000000) * 32 + n * 4))
#define PEout(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x14 - 0x40000000) * 32 + n * 4))
#define PFout(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOF_BASE + 0x14 - 0x40000000) * 32 + n * 4))
#define PBout(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOB_BASE + 0x14 - 0x40000000) * 32 + n * 4))
#define PGout(n) 	*((volatile uint32_t *)(0x42000000 + (GPIOG_BASE + 0x14 - 0x40000000) * 32 + n * 4))

#define W25Q128_CS		PBout(14)//Ƭѡ����
#define W25Q128_SCLK	PBout(3)//spiʱ����
#define W25Q128_MOSI	PBout(5)//��������
#define W25Q128_MISO	PBin(4)//����ӳ�

static GPIO_InitTypeDef 		GPIO_InitStructure;
static EXTI_InitTypeDef 		EXTI_InitStructure;
static NVIC_InitTypeDef 		NVIC_InitStructure;
static TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
static RTC_InitTypeDef 			RTC_InitStructure;
static ADC_InitTypeDef       	ADC_InitStructure;
static ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
static TIM_OCInitTypeDef 		TIM_OCInitStructure;// ����ռ�ձȽṹ���ʼ��(����Ƚ�)
static USART_InitTypeDef 		USART_InitStructure;
static SPI_InitTypeDef  		SPI_InitStructure;

volatile uint8_t g_usart1_buf[256];
volatile uint8_t g_usart3_buf[256];
volatile uint8_t g_time_buf[32];		//��¼��ǰʱ��RTCʵʱʱ��
volatile uint8_t g_temp_buf[32];		//��¼��ǰ��ʪ��

volatile uint32_t g_usart1_cnt = 0;	//����usart1���ݽ��ռ���
volatile uint32_t g_usart3_cnt = 0;
volatile uint32_t tim14_cnt = 0;	// ��¼��ʱ��14�ļ���ֵ����������ռ�ձ�
volatile uint32_t tim1_cnt = 0;		//��¼��ʱ��1�ļ���ֵ����������ռ�ձ�
volatile uint32_t g_usart1_event = 0;
volatile uint32_t g_usart3_event = 0;
volatile uint32_t g_tim3_event = 0;				
volatile uint32_t g_ble_connect_event = 0;		//���������¼�
volatile uint32_t g_feed_event = 0;				//���Ź���λ�¼�
volatile uint32_t g_rtc_wakup_event = 0;		//RTC�����¼�
volatile uint32_t g_rtc_alarm_event = 0;		//RTC����A�¼�
volatile uint32_t g_rtc_reset = 0;				//RTCʱ�������λ��־
uint8_t redata; // �洢���յ�����
uint8_t dht11_buf[5]; // �洢��ʪ��ģ���ȡ��������

int32_t dht11_read_ret;//��ʪ�ȶ�ȡ��������ֵ
typedef struct __flash_t
{
	uint8_t g_flash_buf[64];//��¼ʱ�����ʪ��ƴ���������ַ�����
	uint32_t offset;//�ṹ���������е�ƫ����
}flash_t;
flash_t flash_buf;//����flash����4�Ľṹ��

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

void tim4_init(void)		//��⴮��1�ʹ���3�������Ƿ������
{
	// ʹ��tim4��Ӳ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	// ����tim4�ķ�Ƶֵ������ֵ
	// tim4Ӳ��ʱ��=84MHz/8400=10000Hz�����ǽ���10000�μ���������1��ʱ��ĵ���
	TIM_TimeBaseStructure.TIM_Period = 10000 / 1000 - 1; // ����ֵ10,0 -> 9����1����ʱ��ĵ���
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;		 // Ԥ��Ƶֵ8400
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;		 // ʱ�ӷ�Ƶ����ǰ��û�еģ�����Ҫ��������
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	// ����tim4���ж�
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	// ʹ��tim4
	TIM_Cmd(TIM4, ENABLE);
}

void tim3_init(void)
{
	// ʹ��tim3��Ӳ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	// ����tim3�ķ�Ƶֵ������ֵ
	// tim3Ӳ��ʱ��=84MHz/8400=10000Hz�����ǽ���10000�μ���������1��ʱ��ĵ���
	TIM_TimeBaseStructure.TIM_Period = 10000 / 20 - 1; // ����ֵ500,0 -> 499����50����ʱ��ĵ���
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;	   // Ԥ��Ƶֵ8400
	//TIM_TimeBaseStructure.TIM_ClockDivision = 0;	   // ʱ�ӷ�Ƶ����ǰ��û�еģ�����Ҫ��������
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	// ����tim3���ж�
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


// ��ʼ����ʱ��1
void tim1_init(void)
{
	// ��Ӳ����Դʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	/* ����PE13 Ϊ����ģʽ*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; // ��13����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			 // ���ù���ģʽ���ȿ������������ƣ�Ҳ��������������ƣ�
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 // �����������ǿ�������������ŵ������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		 // ���ŵ��ٶ����Ϊ100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		 // û��ʹ���ڲ���������

	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// ��PE13/14�������ӵ���ʱ��1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	
	// TIM1�Ķ�ʱ����
	// TIM1��Ӳ��ʱ��Ƶ�� = 84 * 2 = 168MHz
	TIM_TimeBaseStructure.TIM_Period = (10000 / 100) - 1; // ����ֵ0~99���������Ƶ��Ϊ100hz
	TIM_TimeBaseStructure.TIM_Prescaler = 16800 - 1;	  // ��ǰ��Ԥ��ƵֵΪ16800����Ƶ֮����10000hz
	// TIM_TimeBaseStructure.TIM_ClockDivision = 0;//��֧��ʱ�ӷ�Ƶ��Ҳ��֮Ϊ2�η�Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ�����0~TIM_Period
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: TIM1 Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			  // ģʽ1��ֻҪTIMx_CNT(����ֵ)<TIMx_CCR1(�Ƚ�ֵ),ͨ��1��Ϊ��Ч״̬������Ϊ��Ч״̬
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // ��/�ر��������
	TIM_OCInitStructure.TIM_Pulse = 50;							  // �Ƚ�ֵ Capture Compare Register
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  // ��Ч״̬����Ϊ�ߵ�ƽ

	// ���ö�ʱ��1��ͨ��3
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	// ʹ��TIM1�ļ�������ʼ����
	TIM_CtrlPWMOutputs(TIM1, ENABLE); // �߼���ʱ��1/8���еĺ���

	TIM_Cmd(TIM1, ENABLE);
}

// ��ʼ����ʱ��14
void tim14_init(void)
{
	// ��Ӳ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	/* ����PF9 PF10Ϊ���ģʽ����������ž�������ߵ͵�ƽ�Ĺ��� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		   // ��9������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // ���ù���ģʽ���ȿ������������ƣ�Ҳ��������������ƣ�
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // �����������ǿ�������������ŵ������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // ���ŵ��ٶ����Ϊ100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   // û��ʹ���ڲ���������

	GPIO_Init(GPIOF, &GPIO_InitStructure);

	// ��PF9�������ӵ���ʱ��14
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_TIM14);

	// TIM14�Ķ�ʱ����
	// TIM14��Ӳ��ʱ��Ƶ�� = 42 * 2 = 84MHz
	TIM_TimeBaseStructure.TIM_Period = (10000 / 100) - 1; // ����ֵ0~99���������Ƶ��Ϊ100hz
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;		  // ��ǰ��Ԥ��ƵֵΪ8400����Ƶ֮����10000hz
	// TIM_TimeBaseStructure.TIM_ClockDivision = 0;//��֧��ʱ�ӷ�Ƶ��Ҳ��֮Ϊ2�η�Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ�����0~TIM_Period
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: TIM14 Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			  // ģʽ1��ֻҪTIMx_CNT(����ֵ)<TIMx_CCR1(�Ƚ�ֵ),ͨ��1��Ϊ��Ч״̬������Ϊ��Ч״̬
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // ��/�ر��������
	TIM_OCInitStructure.TIM_Pulse = 50;							  // �Ƚ�ֵ Capture Compare Register
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  // ��Ч״̬����Ϊ�ߵ�ƽ

	// ���ö�ʱ��14��ͨ��1
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);

	// ʹ��TIM14�ļ�������ʼ����
	TIM_Cmd(TIM14, ENABLE);
}

// ��װ������������TIM14���������Ƶ��
void tim14_set_freq(uint32_t freq)
{
	/* �ر�TIM14 */
	TIM_Cmd(TIM14, DISABLE);

	/*��ʱ���Ļ������ã��������ö�ʱ������������Ƶ��Ϊ freq Hz */
	TIM_TimeBaseStructure.TIM_Period = (10000 / freq) - 1; // ���ö�ʱ�����Ƶ��
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;		   // ��һ�η�Ƶ�����ΪԤ��Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	tim14_cnt = TIM_TimeBaseStructure.TIM_Period; // ��¼TIM14��ʱ���ļ���ֵ���������ñȽ�ֵ���õ���Ҫ��ռ�ձ�
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

	/* ʹ��TIM14 */
	TIM_Cmd(TIM14, ENABLE);
}

// ��װ������������TIM14���������Ƶ��
void tim1_set_freq(uint32_t freq)
{
	/* �ر�TIM1 */
	TIM_Cmd(TIM1, DISABLE);

	/*��ʱ���Ļ������ã��������ö�ʱ������������Ƶ��Ϊ freq Hz */
	TIM_TimeBaseStructure.TIM_Period = (10000 / freq) - 1; // ���ö�ʱ�����Ƶ��
	TIM_TimeBaseStructure.TIM_Prescaler = 16800 - 1;	   // ��һ�η�Ƶ�����ΪԤ��Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	tim1_cnt = TIM_TimeBaseStructure.TIM_Period; // ��¼TIM1��ʱ���ļ���ֵ���������ñȽ�ֵ���õ���Ҫ��ռ�ձ�
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* ʹ��TIM1 */
	TIM_Cmd(TIM1, ENABLE);
}

// ��װ������������ռ�ձ�,ͨ��TIM14��ǰ�ļ���ֵ�����ö�Ӧ�ıȽ�ֵ
void tim14_set_duty(uint32_t duty)
{
	uint32_t cmp;
	cmp = (tim14_cnt + 1) * duty / 100;
	TIM_SetCompare1(TIM14, cmp);
}
// ��װ������������ռ�ձ�,ͨ��TIM1��ǰ�ļ���ֵ�����ö�Ӧ�ıȽ�ֵ
void tim1_set_duty(uint32_t duty)
{
	uint32_t cmp;
	cmp = (tim1_cnt + 1) * duty / 100;
	TIM_SetCompare3(TIM1, cmp); // ���ö�ʱ��ͨ��3�ıȽ�ֵ
}

/*�ⲿ�жϳ�ʼ��*/
void exti_init(void)
{
	// �򿪶˿�A��Ӳ��ʱ�ӣ����ǶԶ˿�A����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// �򿪶˿�E��Ӳ��ʱ�ӣ����ǶԵ�ǰӲ������
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	// �򿪶˿�G��Ӳ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 // ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; // ����ٶ�
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��ʹ���ڲ�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		 // �˿ڵĵ�2������
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		 // �˿ڵĵ�5������
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed; // 
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;		 // �˿ڵĵ�14������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 // ���ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; // ����ٶ�
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��ʹ���ڲ�����������
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//PA0������0�����ӵ�EXTI0
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	//PE2������1�����ӵ�EXIT2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
	//PE5 ���ӵ�EXIT5
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource5);
	//�����ⲿ�ж�0,2
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		// �жϴ���
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // ��ⰴ���İ��£��½��ش�����
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				// ʹ��
	EXTI_Init(&EXTI_InitStructure);
	//�����ⲿ�ж�5
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //˫���ش���
	EXTI_Init(&EXTI_InitStructure);
	
	//NVIC����EXTI0��EXIT2��EXIT5���������ж�����������������ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			 // EXTI0���жϺ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F; // ��ռ���ȼ���0x0F
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;		 // ��Ӧ���ӣ����ȼ���0x0F
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 // ����������ж����󣬴򿪸�ͨ��
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			 // ָ����EXTI2���ж�����ͨ������д�����жϺ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F; // ��ռ���ȼ� 0x1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;		 // ��Ӧ���ȼ� 0x2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 // ʹ��EXTI2�ж�����ͨ��
	NVIC_Init(&NVIC_InitStructure);
	
	/* ͨ��NVIC�����ⲿ�ж�9~5���ж������жϺš����ȼ����жϴ�/�ر� */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�жϺ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;//��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//�жϴ�
	NVIC_Init(&NVIC_InitStructure);	
}

// ����USART1�ĳ�ʼ�����ã���ͨ����ͨ��,�������ڵ�������ͨ�ţ�
void Init_Usart1(uint32_t baud)
{
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable USART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Configure USART Tx and Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // �˿ڸ���ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��ʹ������������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ��������A9 / A10�Ĺ��ܣ�����ΪUSART1_TX / USART1_RX ������Ӳ��ԭ��ͼ��
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	//����1��ʼ��
	USART_InitStructure.USART_BaudRate = baud; // ���ò�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// Init USART1
	USART_Init(USART1, &USART_InitStructure);

	// ����Ҫ�жϣ��������ж���ز���������ӽ�Ƕ���ж�����������
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // ����Ϊ:�������ݼĴ��������жϣ�����2��

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		  // ָ��USART1_IQRͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // ��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // ʹ���ж�����ͨ��
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);
}

// ����USART3�ĳ�ʼ�����ã���������ͨ��
void Init_Usart3(uint32_t baud)
{
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* Configure USART Tx and Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // �˿ڸ���
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��ʹ������������
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// ��������PB10 / PB11�Ĺ��ܣ�����ΪUSART3_TX / USART3_RX ������Ӳ��ԭ��ͼ��
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	USART_InitStructure.USART_BaudRate = baud; // ���ò�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// Init USART3
	USART_Init(USART3, &USART_InitStructure);
	/* Enable USART */
	USART_Cmd(USART3, ENABLE);

	// ����Ҫ�жϣ��������ж���ز���
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // ����Ϊ:�������ݼĴ��������жϣ�����2��

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		  // ָ��USART3_IQRͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // ��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // ʹ���ж�����ͨ��
	NVIC_Init(&NVIC_InitStructure);
}




RTC_TimeTypeDef  		RTC_TimeStructure;
RTC_DateTypeDef 		RTC_DateStructure;
RTC_AlarmTypeDef 		RTC_AlarmStructure;
/*RTCʵʱʱ�ӳ�ʼ�������������жϹ���*/
void rtc_wakeup_init(void)				
{
	 /* Enable the PWR clock ,ʹ�ܵ�Դʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Allow access to RTC ���������RTC*/
	PWR_BackupAccessCmd(ENABLE);
	
	/*ʹ��LSIʱ�ӣ�32KHz��*/
	RCC_LSICmd(ENABLE);
	/*�ȴ�LSTʱ��ʹ�ܳɹ� Wait till LSI is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
	/*ѡ��ʱ��ԴLSI*/
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	/*����RTC���Ƶ��1Hz-->����Ԥ��Ƶֵ*/
	/* RTCʱ�Ӽ��㷽ʽck_spre(1Hz) = RTCCLK(LSE 32Khz) /(uwAsynchPrediv + 1)/(uwSynchPrediv + 1)*/
	/* Enable the RTC Clock ��ʹ��RTCʱ��*/
	RCC_RTCCLKCmd(ENABLE);
	/* Wait for RTC APB registers synchronisation ���ȴ�RTC��ؼĴ�������*/
	RTC_WaitForSynchro();

	/* Configure the RTC data register and RTC prescaler������RTC���ݼĴ�����RTC�ķ�Ƶֵ */
	RTC_InitStructure.RTC_AsynchPrediv = 0x7F;		//�첽��Ƶϵ�� 128-1
	RTC_InitStructure.RTC_SynchPrediv = 0xF9;		//ͬ����Ƶϵ�� 250-1
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;//24Сʱ��ʽ
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

	
	//�رջ��ѹ���
	RTC_WakeUpCmd(DISABLE);	   

	//Ϊ���ѹ���ѡ��RTC���úõ�ʱ��Դ
	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
	
	//���û��Ѽ���ֵΪ�Զ����أ�д��Ĭ��ֵΪ0
	RTC_SetWakeUpCounter(1-1);
	//���RTC�����жϱ�־
	RTC_ClearITPendingBit(RTC_IT_WUT);
	//ʹ��RTC���Ѷ�ʱ���жϣ����Ǽ���1����ɺ�ͻᴥ��һ���ж�
	RTC_ITConfig(RTC_IT_WUT,ENABLE);
	
	RTC_WakeUpCmd(ENABLE);
	
	/*ʹ�ܻ��ѹ���,�����ⲿ�ж���22*/
	/* Configure EXTI Line22�������ⲿ�жϿ�����22 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line22;			//��ǰʹ���ⲿ�жϿ�����22
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//�ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//�����ش����ж� 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;			//ʹ���ⲿ�жϿ�����22
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;		//����RTC�����жϴ���
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//��ռ���ȼ�Ϊ0x3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		//��Ӧ���ȼ�Ϊ0x3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ʹ��
	NVIC_Init(&NVIC_InitStructure);	
	
}

/*�Լ���װ���ö�̬�������ӵĺ���*/
void rtc_alarm_init(uint8_t Hours,uint8_t Minutes,uint8_t Seconds,uint8_t AlarmDateWeekDay)
{
	//�ر����ӹ���
	RTC_AlarmCmd(RTC_Alarm_A,DISABLE); 
	/* Set the alarm 05h:20min:30s ��������*/
	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = Hours;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = Minutes;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = Seconds;
	RTC_AlarmStructure.RTC_AlarmDateWeekDay = AlarmDateWeekDay;
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_None;
	
	//ʹ�����ӹ���
	/* Configure the RTC Alarm A register ����ΪA����*/
	RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);

	/* Enable RTC Alarm A Interrupt ʹ�������ж�*/
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);

	/* Enable the alarm ʹ������A*/
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

	EXTI_InitStructure.EXTI_Line = EXTI_Line17;			//��ǰʹ���ⲿ�жϿ�����17
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//�ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//�����ش����ж� 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;			//ʹ���ⲿ�жϿ�����17
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;		//����RTC�����жϴ���
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//��ռ���ȼ�Ϊ0x3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//��Ӧ���ȼ�Ϊ0x3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ʹ��
	NVIC_Init(&NVIC_InitStructure);	
	
}

/*ADC3��ʼ���������������ѹֵ*/
void adc_init(void)
{
	/*ʹ��Ӳ��ʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	/*����GPIOF7����*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	/* ADC Common Init */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ADC����
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;//ADCӲ��ʱ��Ƶ��=84MHz/2=42MHz
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;//�ر�DMAģʽ
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//����������ļ��ʱ��=5*��1/42MHz��
	ADC_CommonInit(&ADC_CommonInitStructure);
	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//�ֱ���12bit���ֱ���Խ�ߣ��õ����ݸ�׼ȷ�������ȸ���
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//ADC1ֻת��1��ͨ��
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//ADC�������ϵĽ���ת��������DISABLE��ת��һ��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//����Ҫ���ش���������ʹ���������ADC����
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���洢
	ADC_InitStructure.ADC_NbrOfConversion = 1;//ֻת��1��ͨ������Ϊ1�����ADCת�����ͨ������ָ������
	ADC_Init(ADC3, &ADC_InitStructure);
	
	/* ADC3 regular channel 5 configuration��˳��Ϊ1������ʱ��Ϊ3*(1/42MHz)*/
	ADC_RegularChannelConfig(ADC3, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);

	ADC_Cmd(ADC3, ENABLE);
	ADC_SoftwareStartConv(ADC3);//ͨ���������ADC3��ת��
}

/*�������Ź���ʼ��*/
void iwdg_init(void)
{
	//������ʶ������Ź�
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	//���ö������Ź���Ԥ��ƵֵΪ256
	//�õ��������Ź�Ӳ��ʱ��Ϊ32KHz/256=125Hz
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	
	//���ö������Ź��Ķ�������ֵΪ 0 ~124
	IWDG_SetReload(125-1);//Ҳ����1sʱ�䵽��
	
	//���أ�ˢ�£�����ֵ
	IWDG_ReloadCounter();
	
	//ʹ�ܶ������Ź�����
	IWDG_Enable();
}

/*flash��ʼ��������������д������*/
void flash_init(void)
{
	//����FLASH�����ܸ�д���ݣ���������̣�
	FLASH_Unlock();
	//��������4����λʱ���ڲ���32bit
	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
					FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);	
	
	FLASH_EraseSector(FLASH_Sector_4,VoltageRange_3);
	
	//����FLASH��ֻ�ܶ�������д��Ҳ���ܲ���
	FLASH_Lock(); 
}


/*ģ��SPI flash��W25Q128����ʼ��*/
void w25qxx_simulate_init(void)
{
	//ʹ��Ӳ��PB4(��ԭ��ͼ)ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	//����PB3( /CS ),PB5(MOSI),PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 	//	����Ϊ���ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; 	// 	����ٶ�
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//	�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// 	��ʹ���ڲ�����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//����PB4(MISO)Ϊ����ģʽ����ΪMISO���Ŷ�ȡ����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;			//	����PB4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 	//	����ģʽ
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//������ģʽ3��ʱ��������PB3��ʼ��ƽΪ�ߵ�ƽ
	W25Q128_SCLK = 1;
	W25Q128_MOSI = 1;
	W25Q128_CS = 1;//����Ƭѡ����
}
/*SPI flash��W25Q128����ʼ��*/
void w25qxx_init(void)
{
	
#if 0
	//ʹ��Ӳ��SPI1��PB4(��ԭ��ͼ)ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	//����PB3,4,5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	 	//����ģʽ�����żȿ��Ա�Ӳ�����ƣ�Ҳ���Ա�������ƣ�
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; 	// ����ٶ�
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// ��ʹ���ڲ�����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// ��PB3��4��5���ŷֱ���ΪSPI1_SCK��SPI1_MISO��SPI1_MOSI
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
	//����PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;//����PB14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 	//���ģʽ
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	PBout(14)=1;//����
	
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//SPIһ��ʹ��˫��ȫ˫����SPI�����ܸ��ٸ�����豸ͨ�ţ��ٶȿ�
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//M4������������ɫ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//��ʱ��ͼ������С���ֽڵ�Ԫ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							//��ʱ��ͼ��CPOL=1��SPIʱ���߿���ʱΪ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						//��ʱ��ͼ,	CPHA=1,MISO����Ϊ�ڶ����أ�������Ϊ�����أ���������
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//ͨ������������
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;	//���ӻ��������ֲᣬ���ܳ��٣�SCLK��Ƶ��=84MHz/16=5.25MHz
	
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//���ӻ��������ֲ�
	SPI_Init(SPI1, &SPI_InitStructure);
	//ʹ��SPI1����
	SPI_Cmd(SPI1, ENABLE);
#else
	w25qxx_simulate_init();//ģ��spi��ʼ��
#endif
}



/*�Լ���װSPI�������ݺ�����ͬ��ͨ�ţ�������һ���ֽں�ͬʱҲ�᷵�����ݣ�*/
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
	
	//ѭ��8λbit�����ݣ���λ����(��λ�ȷ�)
	for(i=7;i>=0;i--)
	{
		//�жϸ�bitλ�Ƿ�Ϊ�ߵ�ƽ
		if(txd &(1<<i))
		{
			W25Q128_MOSI=1;//MOSI����Ϊ�ߵ�ƽ
		}
		else
		{
			W25Q128_MOSI=0;
		}
			
		//����ʱ����Ϊ�͵�ƽ��MOSI�����������
		W25Q128_SCLK=0;
		delay_us(1);
		//��������ݺ�ͬʱ�������ݵĽ��գ���λ���գ�����Ҫ����ʱ����
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

/*��ȡspi �豸id�ͳ���id����*/
void w25qxx_read_id(uint8_t *m_id,uint8_t *d_id)
{
	//Ƭѡ����PB14����͵�ƽ���ӻ���ʼ����
	PBout(14)=0;
	//����0x90(��ʱ��ͼ90h)
	SPI1_SendByte(0x90);
	//����24bit��ַ�������η���
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);
	
	//�������������ȡ����id
	*m_id = SPI1_SendByte(0xFF);
	*d_id = SPI1_SendByte(0xFF);
	//ͨ�Ž�����CS��PB14����������
	PBout(14)=1;
}

//spiдʹ�� WRITE ENABLE
void w25qxx_write_enable(void)
{
	//cs�������ͣ��ӻ���ʼ����
	W25Q128_CS=0;
	//����0x06
	SPI1_SendByte(0x06);
	//CS����Ϊ�ߵ�ƽ���ӻ�ֹͣ����
	W25Q128_CS=1;
}
//spiд��ʹ�� WRITE DISABLE
void w25qxx_write_disable(void)
{
	W25Q128_CS=0;
	SPI1_SendByte(0x04);
	W25Q128_CS=1;
}

//���Ĵ��� Read Status Register��״̬
uint8_t read_status_register(void)
{
	uint8_t sta;
	W25Q128_CS=0;
	SPI1_SendByte(0x05);
	//��������8λbit���ݣ���ȡ�Ĵ���״̬
	sta = SPI1_SendByte(0xFF);
	W25Q128_CS=1;
	return sta;	
}

//spi flash��������(sector erase)
void w25qxx_sector_erase(uint32_t sector_addr)//ѡ��������ַ
{
	uint8_t sta;
	//дʹ��
	w25qxx_write_enable();
	delay_us(1);
	W25Q128_CS=0;//�ӻ���ʼ����
	//����0x20��������ָ��
	SPI1_SendByte(0x20);
	//����24bits�ĵ�ַ����
	SPI1_SendByte((sector_addr >> 16) & 0xFF);
	SPI1_SendByte((sector_addr >> 8) & 0xFF);
	SPI1_SendByte(sector_addr & 0xFF);
	//���read status register�Ĵ�����BUSY ����λ�Ƿ�Ϊ0����ʾ���ݲ�����ϣ�
	while(1)
	{
		sta=read_status_register();
		if((sta & 0x01)==0x00)
			break;
		delay_us(1);
	}
		
	w25qxx_write_disable();
}

//spi flashҳ���(page program)�����������׵�ַ�������׵�ַ�����ݳ���
void w25qxx_page_program(uint32_t page_addr,uint8_t *buf,uint32_t len)
{
	uint8_t sta;
	uint8_t *p=buf;
	//дʹ��
	w25qxx_write_enable();
	//����0x02ҳ���ָ��
	SPI1_SendByte(0x02);
	//����24bits�ĵ�ַ����
	SPI1_SendByte((page_addr >> 16) & 0xFF);
	SPI1_SendByte((page_addr >> 8) & 0xFF);
	SPI1_SendByte(page_addr & 0xFF);
	
	//�ж�len�Ƿ񳬹�256.������Ҫ������һ��ҳ���
	len = len > 256 ? 256 : len;
	//��λд��buf�е�����
	while(len--)
	{
		SPI1_SendByte(*p);
		p++;
	}
	
	//CS����Ϊ�ߵ�ƽ���ӻ�ֹͣ����
	W25Q128_CS=1;

	delay_us(1);
	//���read status register�Ĵ�����BUSY ����λ�Ƿ�Ϊ0����ʾ���ݲ�����ϣ�
	while(1)
	{
		sta=read_status_register();
		if((sta & 0x01)==0x00)
			break;
		delay_us(1);
	}
	w25qxx_write_disable();
}
//��ȡspi���ݸ���W25Q128S�����ֲ�26ҳʱ��ͼ��д,��������ʼ��ַ����Ŷ�ȡ���ݵĻ�������Ҫ��ȡ�����ݳ���
void w25qxx_read_data(uint32_t addr,uint8_t *buf,uint32_t len)
{
	uint8_t *p = buf;
	
	//CS����Ϊ�͵�ƽ���ӻ���ʼ����
	W25Q128_CS=0;
	
	//�ȷ���0x03 ��ȡ����ָ��
	SPI1_SendByte(0x03);

	//����24bit��ַ���ߵ�ַ�ȷ��ͣ�����ķ���
	SPI1_SendByte((addr>>16)&0xFF);
	SPI1_SendByte((addr>>8)&0xFF);
	SPI1_SendByte(addr&0xFF);
	
	while(len--)
	{
		//ÿ�ζ�ȡ1byte������
		*p=SPI1_SendByte(0xFF);
		p++;
	}
	//CS����Ϊ�ߵ�ƽ���ӻ�ֹͣ����
	W25Q128_CS=1;	
}


/*������ģ���ʼ��*/
void sr04_init(void)
{
	// �򿪶˿�B��Ӳ��ʱ�ӣ����ǶԶ˿�B����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	// �򿪶˿�E��Ӳ��ʱ�ӣ����ǶԵ�ǰӲ������
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	// ����PB6�˿�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		 // ��6������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 // �������ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	 // �����������ǿ�������������ŵ������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;	 // ����ģʽ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // û��ʹ���ڲ���������
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// ����ʱ��ͼ��PB6��ǰΪ�͵�ƽ
	PBout(6) = 0;

	// ����PE6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		 // ��6������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 // ���빦��ģʽ���ȿ������������ƣ�Ҳ��������������ƣ�
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;	 // ����ģʽ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // û��ʹ���ڲ���������
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}
/*��ʪ��ģ�飬����PG9*/
void dht11_init(void)
 {
	// ��PG9��Ӳ���˿�ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	// ����PG9�˿�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		 // ��9������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 // �������ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	 // �����������ǿ�������������ŵ������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;	 // ����ģʽ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // û��ʹ���ڲ���������
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	PGout(9) = 1;
}

/*��ȡ��ʪ��ģ���ȡ������Ϣ*/
int32_t dht11_read(uint8_t *buf)
{
	uint32_t t = 0;
	int32_t i = 0, j = 0;
	uint8_t d = 0;
	uint8_t *p = buf;
	uint8_t check_sum = 0;
	int32_t flag = 0;

	// ����PG9�˿�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		 // ��9������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 // �������ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	 // �����������ǿ�������������ŵ������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;	 // ����ģʽ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // û��ʹ���ڲ���������
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// ����ʱ��ͼ��PB6��ǰΪ�͵�ƽ
	PGout(9) = 0;
	delay_ms(20);
	PGout(9) = 1;
	delay_us(30);

	// ����Ϊ����ģʽ
	// ����PG9�˿�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		 // ��9������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 // �������ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	 // �����������ǿ�������������ŵ������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;	 // ����ģʽ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // û��ʹ���ڲ���������
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// �ȴ��͵�ƽ��Ӧ�ź�
	t = 0;
	while (PGin(9))
	{
		t++;
		delay_us(1);
		if (t >= 4000)
			flag = -1;
	}
	// �����õ͵�ƽ��Ӧ�ź��Ƿ�Ϸ�
	t = 0;
	while (PGin(9) == 0)
	{
		t++;
		delay_us(1);
		if (t >= 100)
			flag = -2;
	}
	// �����øߵ�ƽ��Ӧ�ź��Ƿ�Ϸ�
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
			// �ȴ�����0��������1��ǰ�õ�ƽ����
			t = 0;
			while (PGin(9) == 0)
			{
				t++;
				delay_us(1);
				if (t >= 100)
					flag = -4;
			}
			// ��ʱ40us ����ʱʱ����28us ~ 70us��
			delay_us(40);

			if (PGin(9))
			{
				d |= 1 << i; // ��d������Ӧ��bit��1
				// �ȴ��ߵ�ƽ�������
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
		p[j] = d; // �õ�һ���ֽڣ�8bit��������
	}
	// ��ʱ50us�����Ժ���ͨѶ�����ĵ͵�ƽ
	delay_us(50);

	// ����У��ͣ������յ��������Ƿ�׼ȷ
	check_sum = (p[0] + p[1] + p[2] + p[3]) & 0xFF;

	if (check_sum == p[4])
		flag = 0;
	else
		flag = -6;
	return flag;
}

/*����ʱ��ͼ��װ��������ȡ������Ϣ�ĺ���*/
int sr04_get_distance(void)
{
	double distance = 0;
	int32_t save = 0;
	int32_t cnt = 0;
	PBout(6) = 1;
	delay_us(10); // ��ʱ10us֮��ʹ��PB6��ƽ��Ϊ�͵�ƽ
	PBout(6) = 0;
	// �ȴ������źű�Ϊ�ߵ�ƽ
	while (PEin(6) == 0)
		;

	while (PEin(6) == 1) // �������ƽ����ʱ�䣬ͨ������ת��Ϊ����
	{
		cnt++;
		delay_us(9);
	}
	distance = (double)(3 * cnt / 2);
	if (distance >= 20 && distance <= 1000)
	{
		printf("distance = %.3lfmm \r\n", distance);
		save = 3; // Σ�յȼ�3
	}
	else if (distance > 1000 && distance <= 2000)
	{
		printf("distance = %.3lfmm \r\n", distance);
		save = 2; // Σ�յȼ�3
	}
	else
	{
		printf("distance = %.3lfmm \r\n", distance);
		save = 1; // Σ�յȼ�3
	}

	return save;
}

/*�Լ���װ�ķ����ַ�������*/
void my_USART_SendData(USART_TypeDef *USARTx, uint8_t *data)
{
	while (*data != '\0')
	{
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
			; // �жϷ������ݻ������Ƿ�Ϊ��
		USART_SendData(USARTx, *data);
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
			; // �ж������Ƿ������
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

//��int���͵�������0~59��ת��Ϊ8λ16����(���磺23->0x23)
uint8_t to_hex(uint32_t num)
{
    uint8_t hex_num = 0;
    
    hex_num = (num / 10) * 16 + (num % 10);//��֧��2λʮ��������ת��
	
    return hex_num;
}

//�˲��㷨�õ���ƽ����ADCת������
uint32_t filter_out(void)
{
	uint32_t adc_val;	//adcת���õ���12bit������ֵ
	uint32_t adc_vol;	//��������ֵת���ɵ�ʵ�ʵ�ѹֵ
	uint32_t i;		//һ������ֵ
	for(adc_val=0,i=0;i<1000;i++)
	{
		while(ADC_GetFlagStatus(ADC3,ADC_FLAG_EOC)==RESET);
		ADC_ClearFlag(ADC3,ADC_FLAG_EOC);
		
		adc_val+=ADC_GetConversionValue(ADC3);
	}
	adc_val/=1000;//��β�����ƽ��ֵ
	adc_vol=adc_val*3300/4095;
	printf("adc_vol=%dmv\r\n",adc_vol);
	return adc_vol;
}

/*�Լ���װ�Ľ�����д��flash�ĺ���*/
void flash_write(flash_t *flash_write_buf )
{
	int i = 0;
	FLASH_Unlock();

	for(i = 0; i<sizeof(flash_t)/4;i++)//����д��,һ����4���ֽڣ�����Ҫ����4
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

//����flash����4��δ������ݵĵ�ַ���׵�ַ��ƫ����
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
	return count;//���ص�ǰ���ݿ�
}


int main(void)
{
	char *p;			//һ��ָ��
	uint8_t date;		//���ӣ���
	uint8_t hours;		//���ӣ�ʱ
	uint8_t minutes;	//���ӣ���
	uint8_t seconds;	//���ӣ���
	uint8_t d_id;		//spi�豸id
	uint8_t m_id;		//spi flash����id
	uint32_t duty;		//PWMռ�ձ�	
	int32_t distance_save = 0;//��������లȫ�ȼ�
	int32_t i;					//һ������ֵ
	uint8_t w25qxx_buf[64]={0};
	// int pwm_cmp;
	// �򿪶˿�Ӳ��ʱ�ӣ����ǶԵ�ǰӲ������
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	/* ����PF10Ϊ���ģʽ����������ž�������ߵ͵�ƽ�Ĺ��� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;		   // ��10������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // �������ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // �����������ǿ�������������ŵ������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // ���ŵ��ٶ����Ϊ100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   // û��ʹ���ڲ���������
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	exti_init();
	tim1_init();
	tim3_init();
	tim4_init();
	tim14_init();
	tim14_set_freq(100);
	tim1_set_freq(100);
	
	w25qxx_init();//SPI���ߣ�w25qxx����ʼ��
	
	adc_init();	//ADC��ʼ��
	iwdg_init();//�������Ź���ʼ��
	sr04_init();  // ������ģ���ʼ��
	dht11_init(); // ��ʪ��ģ���ʼ��
	
	delay_ms(100);
	Init_Usart1(115200);//����1��ʼ����������115200
	delay_ms(100);
	Init_Usart3(9600);//����3��ʼ������������ģ��ͨ�ţ�������9600
	
	PFout(9) = PFout(10) = PEout(13) = PEout(14) = 1;
	
	my_USART_SendData(USART3, "------���ڲ��Կ�ʼ------\r\n");
	my_USART_SendData(USART3, "------����  start 					������ʪ�����ݼ�¼ģʽ------\n");
	my_USART_SendData(USART3, "------����  show 					��ʾ��ǰ���е���ʪ�ȼ�¼------\n");
	my_USART_SendData(USART3, "------����  stop 					ֹͣ��ʪ�ȼ�¼------\n");
	my_USART_SendData(USART3, "------����  reset 					�������е���ʪ�ȼ�¼------\n");
	my_USART_SendData(USART3, "------����  Alarm set-d-h-m-s# 		��������A����ʱ��------\n");
	

	if (RTC_ReadBackupRegister(RTC_BKP_DR0) != 4455)
	{
		rtc_wakeup_init();//ʵʱʱ��RTC��ʼ��
		flash_init();	//�������4
		RTC_WriteBackupRegister(RTC_BKP_DR0, 4455);
	}
	else
	{
		g_rtc_reset=1;//��ֹRTCʱ�临λ
		rtc_wakeup_init();//ʵʱʱ��RTC��ʼ��
	}
	
	w25qxx_read_id(&m_id,&d_id);//��ȡspi�豸id�ͳ���id
	printf("m_id:%x,d_id:%x\r\n",m_id,d_id);
	w25qxx_sector_erase(0);//��������0
	memset(w25qxx_buf,'a',sizeof(w25qxx_buf));
	//��0��ַд��w25qxx_buf���������
	w25qxx_page_program(0,w25qxx_buf,sizeof(w25qxx_buf));
	memset(w25qxx_buf,0,sizeof(w25qxx_buf));
	//��0��ַ��ʼ��ȡ����
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
		if (g_usart1_event)//����1����������ɣ���ʼ����
		{
			printf("usart1 recv data:%s\r\n", g_usart1_buf);
			my_USART_SendData(USART3, (uint8_t*)g_usart1_buf);
			// my_USART_SendData(USART3,g_usart1_buf);
			g_usart1_event = 0;
			g_usart1_cnt = 0;
			memset((void *)g_usart1_buf, 0, sizeof g_usart1_buf);
		}
		
		if (g_usart3_event)//��������������ɣ���ʼ����
		{
			printf("usart3 recv data:%s\r\n", g_usart3_buf);
			if (strstr((char *)g_usart3_buf, "duty"))
			{
				p = strtok((char *)g_usart3_buf, "=");
				p = strtok(NULL, "=");
				duty = atoi(p);
				printf("atoi(p):%d\n", duty); // �������ַ���const char *��ת��Ϊ�������Σ�int��
				tim14_set_duty(duty);
			}
			//������ʪ�����ݼ�¼ģʽ
			if(strstr((char *)g_usart3_buf,"temp?#")||strstr((char *)g_usart3_buf,"start"))
			{
				my_USART_SendData(USART3,"---��ʼ��ʪ�ȼ�¼---\r\n");
				g_tim3_event=1;
			}
			if(strstr((char *)g_usart3_buf,"Alarm set-"))
			{
				printf("����ʱ��\n");
				//��ȡ ��-ʱ-��-��
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
				rtc_alarm_init(hours,minutes,seconds,date);//����ʱ��	
			}
			if(strstr((char *)g_usart3_buf,"stop"))//������ʪ�����ݼ�¼ģʽ
			{
				my_USART_SendData(USART3,"---ֹͣ��ʪ�ȼ�¼---\r\n");
				g_tim3_event=0;
			}
			if(strstr((char *)g_usart3_buf,"show"))
			{
				my_USART_SendData(USART3,"---�鿴��ʪ�ȼ�¼---\r\n");
				flash_read(10);
			}
			if(strstr((char *)g_usart3_buf,"reset"))
			{
				my_USART_SendData(USART3,"---���flash����4�ɹ�---\r\n");
				flash_init();
			}
			g_usart3_event = 0;
			g_usart3_cnt = 0;
			memset((void *)g_usart3_buf, 0, sizeof g_usart3_buf);
		}
		
		if (g_tim3_event)//���������¶��¼���
		{	
			dht11_read_ret= dht11_read(dht11_buf);
			if(dht11_read_ret==0)
			{
				sprintf((char *)g_temp_buf,"Temp=%d.%d Humi=%d.%d",dht11_buf[2], dht11_buf[3], dht11_buf[0], dht11_buf[1]);
				
				if(dht11_buf[2]>=29)
				{
					my_USART_SendData(USART3,"---�¶ȳ�����ֵ,ֹͣι��---\r\n");
					g_feed_event=1;
				}	
			}				
			else
			{
				printf("dht11 read error code is %d\r\n", dht11_read_ret);
			}
	
			PFout(10) ^= 1;
			delay_ms(6000);//��ʱ6�룬��־λ����λ��ÿ6���¼һ��
			PFout(10) ^= 1;
		}
		
		if(g_ble_connect_event)//���������¼�
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
		
		if(g_rtc_wakup_event)//RTC�����¼�
		{
			//��ȡʱ��
			RTC_GetTime(RTC_Format_BCD,&RTC_TimeStructure);
			//��ȡ����
			RTC_GetDate(RTC_Format_BCD, &RTC_DateStructure);
		
			if(g_tim3_event)
			{
				flash_buf.offset=flash_read_offset((uint32_t *)(0x08010000));
				if(flash_buf.offset>=10)
				{
					printf("---���������ɹ�---\r\n");
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
			g_rtc_wakup_event=0;//��λ
		}
		
		if(g_rtc_alarm_event)
		{
			my_USART_SendData(USART3,"---����A����---\r\n");
			g_rtc_alarm_event=0;
		}
		
		distance_save = sr04_get_distance();
		
		switch (distance_save)
		{
		case 0:
			printf("distance error\r\n");
			break;
		case 1:
			printf("��ȫ�ȼ�1,��ȫ��\r\n");
			tim1_set_duty(100);
			break;
		case 2:
			printf("��ȫ�ȼ�2,ע�⣡��\r\n");
			tim1_set_duty(60);
			break;
		case 3:
			printf("��ȫ�ȼ�3,Σ�գ�����\r\n");
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
	// ���EXTI0�Ƿ����ж�����
	if (EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		/* �رն�ʱ��3 */
		TIM_Cmd(TIM3, DISABLE);

		/* ��յ�ǰ����ֵ */
		TIM_SetCounter(TIM3, 0);

		/* ������ʱ��3��������һЩ�Ʋ����ڣ�Ҳ�����ö�ʱ�� */
		TIM_Cmd(TIM3, ENABLE);

		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void EXTI2_IRQHandler(void)
{
	// ���EXTI2�Ƿ����ж�����
	if (EXTI_GetITStatus(EXTI_Line2) == SET)
	{
		/* �رն�ʱ��3 */
		TIM_Cmd(TIM3, DISABLE);

		/* ��յ�ǰ����ֵ */
		TIM_SetCounter(TIM3, 0);

		/* ������ʱ��3��������һЩ�Ʋ����ڣ�Ҳ�����ö�ʱ�� */
		TIM_Cmd(TIM3, ENABLE);

		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

void EXTI9_5_IRQHandler(void)//�ⲿ�ж���5���������������Ͽ�
{
	/* ����ж��Ƿ��д��� */
	if(EXTI_GetITStatus(EXTI_Line5) == SET)
	{
		/* ����û����� */
		g_ble_connect_event=1;

		/* ����жϱ�־λ������CPU��ǰ�¼��Ѿ��������
           ˼���⣺�������ձ�־λ�������ʲô����
		*/
		EXTI_ClearITPendingBit(EXTI_Line5);
				
	}
}
void TIM3_IRQHandler(void)
{
	// ����־λ
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		if (PEin(2) == 0)
		{
			g_tim3_event = 1;//����2���´����¶Ȳ����¼�
		}
		TIM_Cmd(TIM3, DISABLE); // �رն�ʱ��
		// ��ձ�־λ
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

void TIM4_IRQHandler(void)//��⴮��1�ʹ���3�������Ƿ������
{
	static uint32_t cnt = 0;
	static uint32_t cnt3 = 0;
	if(g_feed_event == 0)
		IWDG_ReloadCounter();//ˢ�¼���ֵ����ֹ����ֵ������0ʹ��cpu��λ���׳�ι����
	// ����־λ
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		if (cnt != g_usart1_cnt)//���USART1�����Ƿ�������
		{
			cnt = g_usart1_cnt;
		}
		else if ((cnt == g_usart1_cnt) && cnt)
		{
			g_usart1_event = 1; // ֪ͨ�������ݷ������
			cnt = 0;			// ��λ
		}
		
		if (cnt3 != g_usart3_cnt)//���USART3�����Ƿ�������
		{
			cnt3 = g_usart3_cnt;
		}
		else if ((cnt3 == g_usart3_cnt) && cnt3)
		{
			g_usart3_event = 1;
			cnt3 = 0;
		}
		// ��ձ�־λ
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

// USART1�жϷ�����(���մ��ڷ�����������)
void USART1_IRQHandler(void) // �мɲ��ܼ�printf������ӡ����������ʱ��̫�࣬���ݽ��ղ�����
{
	// ���USART1�Ƿ�����USART_IT_RXNE�жϣ�Ҳ���Ǵ����˽��������ж��¼�
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		// ����û�����
		redata = USART_ReceiveData(USART1);
	
		if (g_usart1_cnt < sizeof(g_usart1_buf))
		{
			g_usart1_buf[g_usart1_cnt++] = redata;
		}

		// ��ձ�־λ
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
 
void USART3_IRQHandler(void)
{
	uint8_t bdata;
	// ���USART3�Ƿ�����USART_IT_RXNE�жϣ�Ҳ���Ǵ����˽��������ж��¼�
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		// ����û�����
		bdata = USART_ReceiveData(USART3);
		
		if (g_usart3_cnt < sizeof(g_usart3_buf))
		{
			g_usart3_buf[g_usart3_cnt++] = bdata;
		}

		// ��ձ�־λ
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}

//ʵʱʱ��RTC�����ж�
void RTC_WKUP_IRQHandler(void)
{
	//����־λ
	if(RTC_GetITStatus(RTC_IT_WUT) != RESET)
	{
		//����û�����
		g_rtc_wakup_event=1;
	
		//��ձ�־λ	
		RTC_ClearITPendingBit(RTC_IT_WUT);
		EXTI_ClearITPendingBit(EXTI_Line22);

	}
}

//ʵʱʱ��RTC�����ж�
void RTC_Alarm_IRQHandler(void)
{
	//����־λ������A��
	if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
	{
		//����û�����
		g_rtc_alarm_event=1;
	
		//��ձ�־λ	
		RTC_ClearITPendingBit(RTC_IT_ALRA);
		EXTI_ClearITPendingBit(EXTI_Line17);

	}
}
	
