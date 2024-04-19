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
//窗口狗
int WWDG_CNT = 0x7f;

//窗口狗的中断配置
void WWDG_NVIC_Init()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn; //WWDG ??
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //?? 2 ???? 3 ? 2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //?? 2,???? 3,? 2
    NVIC_Init(&NVIC_InitStructure); //NVIC ???
}

//窗口狗的初始化
void WWDG_Init(u8 tr,u8 wr,u32 fprer)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE); // WWDG时钟使能
    WWDG_CNT=tr&WWDG_CNT; //初始化WWDG_CNT.
    WWDG_SetPrescaler(fprer); //设置WWDG_CNT预分频值
    WWDG_SetWindowValue(wr); //设置窗口值
    WWDG_Enable(WWDG_CNT);
     //使能看门狗,设置counter
    WWDG_ClearFlag(); //清除提前唤醒中断标志位
    WWDG_NVIC_Init(); //初始化看门狗NVIC
    WWDG_EnableIT(); //开启窗口看门狗中断
}
//重设置WWDG计数器的值
void WWDG_Set_Counter(u8 cnt)
{
	WWDG_Enable(cnt); //使能看门狗,设置counter .
}
 
//窗口狗的中断函数
#ifdef __cplusplus
extern "C"{
#endif
void WWDG_IRQHandler()
{ 
	WWDG_SetCounter(WWDG_CNT);//重设窗口看门狗值
	WWDG_ClearFlag(); //清除提前唤醒中断标志位
    //TODO: This message gets overwritten by the kill() call
    printf(MSG_ERR);
    printf("Something is wrong, Watchdog will Reset your print,please turn off the printer.\n\r");
    kill(); //kill blocks
    while(1); //wait for user or serial reset
}
#ifdef __cplusplus
}
#endif

/*************独立看门狗,没有中断***********/
//独立狗喂狗程序
void IWDG_Feed(void)
{
    IWDG_ReloadCounter();
}
//独立看门狗初始化,设置成4S的喂狗
void watchdog_init()
{
#ifdef WATCHDOG_RESET_MANUAL
    //We enable the watchdog timer, but only for the interrupt.
    //Take care, as this requires the correct order of operation, with interrupts disabled. See the datasheet of any AVR chip for details.
    wdt_reset();
#else
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //取消寄存器写保护
    IWDG_SetPrescaler(IWDG_Prescaler_64); //看门狗的分频值,看门狗的时钟是40k,分频64,看门狗一次计数时间1.6ms
    IWDG_SetReload(1875); //看门狗的计数溢出值,4s/1.6=1875次计数,最大值是0xfff
    IWDG_ReloadCounter(); //喂狗
    IWDG_Enable();    //使能看门狗 
	
		//WWDG_Init(0X7F,0X5F,WWDG_Prescaler_8);
		printf("WatchDog start!\n\r");

#endif
}

#endif//USE_WATCHDOG

