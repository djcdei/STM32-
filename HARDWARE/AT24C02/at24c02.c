/*EEPROM*/

#include"at24c02.h"

static GPIO_InitTypeDef GPIO_InitStructure;
/*ģ��I2C��ʼ��*/
void at24c02_init(void)
{
	// �򿪶˿�B��ʱ�ӣ���Ӳ��ԭ��ͼ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	// ���ö˿�B��8��9�����ţ�����Ϊ�������ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; // 8 9������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		   // ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		   // ���� Push Pull����© Open Drain
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	   // ��ʹ������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;		   // ���٣����ĵͣ�����������Ӧʱ�����
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// �������ŵ�ƽ����ʱ��ͼ��
	AT24C02_SCL_W= 1;//PB8
	AT24C02_SDA_W = 1;//PB9
}

//PB9����ģʽ���ú���
void at24c02_sda_pin_mode(GPIOMode_TypeDef pin_mode)
{
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		 // 9������
	GPIO_InitStructure.GPIO_Mode = pin_mode;		 // ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	 // ���� Push Pull����© Open Drain
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��ʹ������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;	 // ���٣����ĵͣ�����������Ӧʱ�����
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// i2c��������(�ź�)
void at24c02_i2c_start(void)
{
	// ����PB9Ϊ���ģʽ
	sda_pin_mode(GPIO_Mode_OUT);
	// ���������ʼ�źŵķ�������ʼ�źţ�ʱ��Ϊ�ߵ�ƽ��ʱ��������SDA�ɸߵ��͵ı仯������Ϊ��ʼ�źţ����ĵ���
	AT24C02_SCL_W= 1;	 // ʱ�Ӹߵ�ƽ
	AT24C02_SDA_W= 1;	 // ����������������ߴӸߵ��ͷ�������
	delay_us(5); // ��ʱ5us�����ʱ�����ڣ���I2Cʱ��Ϊ100KHzҲ��������10us
	PAT24C02_SDA_W = 0;
	delay_us(5); // ��ʱ5us
	// ʱ������
	AT24C02_SCL_W = 0;
	delay_us(5);
}

// i2cֹͣ����(�ź�)
void at24c02_i2c_stop(void)
{
	// ����PB9Ϊ���ģʽ
	sda_pin_mode(GPIO_Mode_OUT);
	AT24C02_SCL_W = 1; // ʱ�Ӹߵ�ƽ
	AT24C02_SDA_W = 0; // ����������������ߴӵ͵��߷�������
	delay_us(5);
	AT24C02_SDA_W = 1;
	delay_us(5);
}

//i2c�������ݺ���
void at24c02_i2c_send_byte(uint8_t byte)
{
	int32_t i;
	at24c02_sda_pin_mode(GPIO_Mode_OUT);//
	AT24C02_SCL_W=0;
	AT24C02_SDA_W=0;
	delay_us(5);
	
	for(i=7; i>=0; i--)
	{
		if(byte & (1<<i))
			AT24C02_SDA_W=1;
		else
			AT24C02_SDA_W=0;
	
		delay_us(5);
		//��ǰSDA���ŵ�ƽ�ǲ���ģ�Ȼ��ӻ����Կɿ�����

		AT24C02_SCL_W=1;	
		delay_us(5);

		//��ǰSDA���ŵ�ƽ���ܻᷢ�������Ȼ��ӻ������ǲ��ɿ�
		AT24C02_SCL_W=0;	
		delay_us(5);
	}
}

//i2cӦ���ź�
uint8_t at24c02_i2c_wait_ack(void)
{
	uint8_t ack=0;
	at24c02_sda_pin_mode(GPIO_Mode_IN);	//��PB(9)ת��Ϊ����ģʽ
	
	AT24C02_SCL_W=1;
	delay_us(5);
	
	//��⵽SDA����Ϊ�͵�ƽ�����Ǵӻ���Ӧ��
	if(AT24C02_SDA_R==0)
		ack=0;
	else
		ack=1;
	
	AT24C02_SCL_W=0;
	delay_us(5);	
	return ack;
}