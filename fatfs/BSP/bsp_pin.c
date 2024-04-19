#include "bsp_pin.h"
#include "marlin.h"


//函数说明: IO口的时钟设置
//输入参数：GPIO_TypeDef* GPIOx:IO口类型 
//输出参数：无
//备注：
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

//函数说明: IO口写函数
//输入参数：GPIO_TypeDef* GPIOx:IO口类型 uint16_t GPIO_Pin :IO口管教号
//输出参数：无
//备注：
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

//函数说明: IO口模型设置函数
//输入参数：GPIO_TypeDef* GPIOx:IO口类型 uint16_t GPIO_Pin :IO口管教号,GPIOMode_TypeDef mode:IO口模型
//输出参数：无
//备注：
void GPIO_ModeBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef mode)
{	
	GPIO_ClockSet(GPIOx,GPIO_Pin,ENABLE); 
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin;				     //LED1
  GPIO_InitStructure.GPIO_Mode = mode;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOx, &GPIO_InitStructure);	
}
//函数说明: IO口模型设置成输出50M推挽模式
//输入参数：GPIO_TypeDef* GPIOx:IO口类型 uint16_t GPIO_Pin :IO口管教号
//输出参数：无
//备注：
void GPIO_OutputBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_ClockSet(GPIOx,GPIO_Pin,ENABLE); 
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOx, &GPIO_InitStructure);	
}
//函数说明: IO口模型设置成输入50M上拉模式
//输入参数：GPIO_TypeDef* GPIOx:IO口类型 uint16_t GPIO_Pin :IO口管教号
//输出参数：无
//备注：
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


