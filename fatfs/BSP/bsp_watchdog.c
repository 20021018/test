#include "../marlin/Marlin.h"

#ifdef USE_WATCHDOG


#include "bsp_watchdog.h"
//#include "ultralcd.h"
#include "language.h"

//===========================================================================
//=============================private variables  ============================
//===========================================================================

//===========================================================================
//=============================functinos         ============================
//===========================================================================
//���ڹ�
int WWDG_CNT = 0x7f;

//���ڹ����ж�����
void WWDG_NVIC_Init()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn; //WWDG ??
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //?? 2 ???? 3 ? 2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //?? 2,???? 3,? 2
    NVIC_Init(&NVIC_InitStructure); //NVIC ???
}

//���ڹ��ĳ�ʼ��
void WWDG_Init(u8 tr,u8 wr,u32 fprer)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE); // WWDGʱ��ʹ��
    WWDG_CNT=tr&WWDG_CNT; //��ʼ��WWDG_CNT.
    WWDG_SetPrescaler(fprer); //����WWDG_CNTԤ��Ƶֵ
    WWDG_SetWindowValue(wr); //���ô���ֵ
    WWDG_Enable(WWDG_CNT);
     //ʹ�ܿ��Ź�,����counter
    WWDG_ClearFlag(); //�����ǰ�����жϱ�־λ
    WWDG_NVIC_Init(); //��ʼ�����Ź�NVIC
    WWDG_EnableIT(); //�������ڿ��Ź��ж�
}
//������WWDG��������ֵ
void WWDG_Set_Counter(u8 cnt)
{
	WWDG_Enable(cnt); //ʹ�ܿ��Ź�,����counter .
}
 
//���ڹ����жϺ���
#ifdef __cplusplus
extern "C"{
#endif
void WWDG_IRQHandler()
{ 
	WWDG_SetCounter(WWDG_CNT);//���贰�ڿ��Ź�ֵ
	WWDG_ClearFlag(); //�����ǰ�����жϱ�־λ
    //TODO: This message gets overwritten by the kill() call
    printf(MSG_ERR);
    printf("Something is wrong, Watchdog will Reset your print,please turn off the printer.\n\r");
    kill(); //kill blocks
    while(1); //wait for user or serial reset
}
#ifdef __cplusplus
}
#endif

/*************�������Ź�,û���ж�***********/
//������ι������
void IWDG_Feed(void)
{
    IWDG_ReloadCounter();
}
//�������Ź���ʼ��,���ó�4S��ι��
void watchdog_init()
{
#ifdef WATCHDOG_RESET_MANUAL
    //We enable the watchdog timer, but only for the interrupt.
    //Take care, as this requires the correct order of operation, with interrupts disabled. See the datasheet of any AVR chip for details.
    wdt_reset();
#else
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ȡ���Ĵ���д����
    IWDG_SetPrescaler(IWDG_Prescaler_64); //���Ź��ķ�Ƶֵ,���Ź���ʱ����40k,��Ƶ64,���Ź�һ�μ���ʱ��1.6ms
    IWDG_SetReload(1875); //���Ź��ļ������ֵ,4s/1.6=1875�μ���,���ֵ��0xfff
    IWDG_ReloadCounter(); //ι��
    IWDG_Enable();    //ʹ�ܿ��Ź� 
	
		//WWDG_Init(0X7F,0X5F,WWDG_Prescaler_8);
		printf("WatchDog start!\n\r");

#endif
}

#endif//USE_WATCHDOG

