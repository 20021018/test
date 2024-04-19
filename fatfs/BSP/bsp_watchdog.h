#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "Marlin.h"

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef USE_WATCHDOG

  // intialise watch dog with a 1 sec interrupt time
  void watchdog_init();
	void IWDG_Feed(void);
  //If we do not have a watchdog, then we can have empty functions which are optimized away.	
	void WWDG_Init(u8 tr,u8 wr,u32 fprer);
#endif 

#ifdef __cplusplus
 }
#endif

#endif 

		
		

   
   
   
	 
