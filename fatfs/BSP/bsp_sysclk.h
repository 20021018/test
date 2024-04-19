#ifndef BSP_SYSCLK_H
#define BSP_SYSCLK_H

#include "ch32v30x.h"

#ifdef __cplusplus
 extern "C" {
#endif


void  SystemTick_Init(void);



extern volatile uint32_t system_ms;
extern volatile uint32_t system_minute;



extern void timer3_init();




#ifdef __cplusplus
 }
#endif




#endif

