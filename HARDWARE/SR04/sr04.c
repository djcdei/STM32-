#include "sr04.h" 
#include <stdio.h>
static GPIO_InitTypeDef GPIO_InitStructure;
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
		save = 2; // Σ�յȼ�2
	}
	else
	{
		printf("distance = %.3lfmm \r\n", distance);
		save = 1; // Σ�յȼ�1
	}

	return save;
}







