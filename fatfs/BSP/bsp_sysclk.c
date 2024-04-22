#include "bsp_sysclk.h"

volatile uint32_t system_ms = 0;
volatile uint32_t system_minute = 0;

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

#ifdef __cplusplus
extern "C" {
#endif

//stm32����ϵͳʱ��
//void SysTick_Handler(void)     __attribute__((interrupt()));

void SysTick_Handler(void)
{
	static int ms;
    SysTick->SR  = 0;//
	system_ms++;
	ms++;
	if(ms==6000)
	{
			system_minute++;
			ms=0;

	}
}

#ifdef __cplusplus
}
#endif

//static RV_STATIC_INLINE uint32_t SysTick_Config(uint32_t ticks)
//{
//  if (ticks > SysTick_LOAD_RELOAD_Msk)  return (1);            /* Reload value impossible */
//
//  SysTick->LOAD  = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;      /* set reload register */
//  NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Cortex-M0 System Interrupts */
//  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
//  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
//                   SysTick_CTRL_TICKINT_Msk   |
//                   SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
//  return (0);                                                  /* Function successful */
//}




RV_STATIC_INLINE uint32_t SysTick_Config(uint64_t ticks)
{
//  if (ticks > 0xffffffffffffffff)     return (1);          /* Reload value impossible */
//
//  SysTick->CMP  = (ticks & 0xffffffffffffffff) - 1;      /* set reload register */
//  NVIC_SetPriority(SysTicK_IRQn, (1<<3) - 1);  /* set Priority for Cortex-M0 System Interrupts */
//  SysTick->CNT   = 0;     /* Load the SysTick Counter Value */
//  SysTick->SR = 0;
//
//  NVIC_EnableIRQ(SysTicK_IRQn);
//
//  SysTick->CTLR  = 0x0000002f;
//
//  return (0);            /* Function successful */
    SysTick->CTLR = 0x00000000;             //���ƼĴ�����λ

       SysTick->SR = 0x00000000;               //״̬�Ĵ�����λ

       SysTick->CNT = 0x00000000;              //��������λ�����ó�ʼֵΪ0

       SysTick->CMP = ticks;                   //���ؼ��ؼĴ�����ֵ

       NVIC_SetPriority(SysTicK_IRQn, 15);     //����SysTick�ж����ȼ�

       NVIC_EnableIRQ(SysTicK_IRQn);           //ʹ�ܿ���Systick�ж�

       SysTick->CTLR |= 0x0000000B;            //����ϵͳ������STK��HCLK/8ʱ����

       return (0);

}

//ϵͳʱ�Ӳ���1ms�Ķ�ʱ�ж�

void  SystemTick_Init(void)
{
//	SysTick_Config(71999);
	 SysTick_Config(SystemCoreClock / 8000);//1ms            72M/8000/9000000 = 1/1000 = 1ms
} 

//��ʱ��3�ĳ�ʼ��,�������ݲ�ѯ�¶Ⱥ�PID�Ĵ�����ʱ�ж�1ms
void timer3_init()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);                   
	NVIC_InitStructure.NVIC_IRQChannel =TIM3_IRQn ;           
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;      
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;           
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;   
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);  
	TIM_DeInit(TIM3);
	
	TIM_TimeBaseStructure.TIM_Period=71;
	TIM_TimeBaseStructure.TIM_Prescaler=1000-1;     
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;   
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);     
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);           
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);     
	TIM_Cmd(TIM3, ENABLE);                       
}
