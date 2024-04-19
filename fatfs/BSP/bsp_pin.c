#include "bsp_pin.h"
#include "marlin.h"


//����˵��: IO�ڵ�ʱ������
//���������GPIO_TypeDef* GPIOx:IO������ 
//�����������
//��ע��
void GPIO_ClockSet(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, FunctionalState enable)
{
	uint32_t rcc_apb2periph = 0;
	
	if(GPIOx==GPIOA)
	{
		rcc_apb2periph = RCC_APB2Periph_GPIOA;
	}
	else if(GPIOx==GPIOB)
	{
		rcc_apb2periph = RCC_APB2Periph_GPIOB;
	}
	else if(GPIOx==GPIOC)
	{
		rcc_apb2periph = RCC_APB2Periph_GPIOC;
	}
	else if(GPIOx==GPIOD)
	{
		rcc_apb2periph = RCC_APB2Periph_GPIOD;
	}
	else if(GPIOx==GPIOE)
	{
		rcc_apb2periph = RCC_APB2Periph_GPIOE;
	}
	RCC_APB2PeriphClockCmd(rcc_apb2periph, enable);	 //PD2---LED3
}

//����˵��: IO��д����
//���������GPIO_TypeDef* GPIOx:IO������ uint16_t GPIO_Pin :IO�ڹ̺ܽ�
//�����������
//��ע��
void GPIO_WriteBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t BitVal)
{ 
  if (BitVal)
  {
    GPIOx->BSHR = GPIO_Pin;
  }
  else
  {
    GPIOx->BCR = GPIO_Pin;
  }
}

//����˵��: IO��ģ�����ú���
//���������GPIO_TypeDef* GPIOx:IO������ uint16_t GPIO_Pin :IO�ڹ̺ܽ�,GPIOMode_TypeDef mode:IO��ģ��
//�����������
//��ע��
void GPIO_ModeBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef mode)
{	
	GPIO_ClockSet(GPIOx,GPIO_Pin,ENABLE); 
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin;				     //LED1
  GPIO_InitStructure.GPIO_Mode = mode;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOx, &GPIO_InitStructure);	
}
//����˵��: IO��ģ�����ó����50M����ģʽ
//���������GPIO_TypeDef* GPIOx:IO������ uint16_t GPIO_Pin :IO�ڹ̺ܽ�
//�����������
//��ע��
void GPIO_OutputBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_ClockSet(GPIOx,GPIO_Pin,ENABLE); 
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOx, &GPIO_InitStructure);	
}
//����˵��: IO��ģ�����ó�����50M����ģʽ
//���������GPIO_TypeDef* GPIOx:IO������ uint16_t GPIO_Pin :IO�ڹ̺ܽ�
//�����������
//��ע��
void GPIO_InputBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_ClockSet(GPIOx,GPIO_Pin,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin;				     //LED1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOx, &GPIO_InitStructure);	
	
//	GPIO_SetBits(GPIOx,GPIO_Pin);
}


