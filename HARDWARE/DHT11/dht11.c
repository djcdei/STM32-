#include "dht11.h"

static GPIO_InitTypeDef GPIO_InitStructure;/*��ʪ��ģ�飬����PG9*/
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
	// �����õ͵�ƽ��Ӧ�ź��Ƿ�Ϸ�������80us�ĵ͵�ƽ
	t = 0;
	while (PGin(9) == 0)
	{
		t++;
		delay_us(1);
		if (t >= 100)
			flag = -2;
	}
	// �����øߵ�ƽ��Ӧ�ź��Ƿ�Ϸ�������80us�ĸߵ�ƽ
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

