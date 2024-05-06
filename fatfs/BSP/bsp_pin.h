#ifndef PIN_H
#define PIN_H

#include "ch32v30x.h"

#include "../marlin/Marlin.h"

#define MASK(N)  (1<<N)

//#define HEATER_0_PIN        GPIOC, GPIO_Pin_0	 //挤出机1
//#define LED_PIN            GPIOC, GPIO_Pin_2	 //MOS外接LED

//#define HEATER_BED_PIN      GPIOC, GPIO_Pin_3	 //热床              没有热床

#define HEATER_0_PIN        GPIOC, GPIO_Pin_2	 //挤出机1
//#define FAN_PIN             GPIOC, GPIO_Pin_1	 //挤出机风扇
#define LED_PIN            GPIOE, GPIO_Pin_0	 //MOS外接LED ->PE0

#define BUZ_PIN            GPIOA, GPIO_Pin_12 	//蜂鸣器

#define POWER_CHK       	GPIOA, GPIO_Pin_11	 //电源掉电检测

//#define X_MIN_PIN           GPIOB, GPIO_Pin_7	   //XMIN限位开关
//#define X_MAX_PIN         	GPIOB, GPIO_Pin_8
#define X_MAX_PIN           GPIOB, GPIO_Pin_7      // 只需要XMAX,调换
#define X_MIN_PIN           GPIOB, GPIO_Pin_8

#define Y_MIN_PIN           GPIOB, GPIO_Pin_9	
#define Y_MAX_PIN         	GPIOC, GPIO_Pin_13	

#define Z_MAX_PIN           GPIOB, GPIO_Pin_10  // 只需要ZMAX,调换
#define Z_MIN_PIN           GPIOC, GPIO_Pin_15  //
//#define Z_MIN_PIN           GPIOB, GPIO_Pin_10  //
//#define Z_MAX_PIN         	GPIOC, GPIO_Pin_15  //


#define X_STEP_PIN          GPIOB, GPIO_Pin_2	  //X轴电机
#define X_DIR_PIN           GPIOC, GPIO_Pin_5	
#define X_ENABLE_PIN        GPIOC, GPIO_Pin_4
#define X_STOP_PIN          GPIOC, GPIO_Pin_4

#define Y_STEP_PIN          GPIOA, GPIO_Pin_7	  //Y轴电机
#define Y_DIR_PIN           GPIOA, GPIO_Pin_6	
#define Y_ENABLE_PIN        GPIOC, GPIO_Pin_4	
#define Y_STOP_PIN          GPIOC, GPIO_Pin_4	

#define Z_STEP_PIN          GPIOA, GPIO_Pin_15	  //Z轴电机
#define Z_DIR_PIN           GPIOA, GPIO_Pin_9	  //_从PA3 改到了PA14 ,再修改为PA9
#define Z_ENABLE_PIN        GPIOC, GPIO_Pin_4
#define Z_STOP_PIN          GPIOC, GPIO_Pin_4

#define E0_STEP_PIN         GPIOA, GPIO_Pin_1	 //挤出电机
#define E0_DIR_PIN          GPIOA, GPIO_Pin_0	
#define E0_ENABLE_PIN       GPIOC, GPIO_Pin_4

// #define EE_SCL_PIN    			GPIOA, GPIO_Pin_3	
// #define EE_SDA_PIN     			GPIOA, GPIO_Pin_2	


//没有热床温度传感器
//#define TEMP_BED_PIN        GPIOB, GPIO_Pin_0
//#define TEMP_BED_CHANNEL    ADC_Channel_8   //对应的ADC采样通道

#define TEMP_0_PIN          GPIOC, GPIO_Pin_1	     
#define TEMP_0_CHANNEL      ADC_Channel_11  //对应的ADC采样通道

//#define PLATFORM_PIN    GPIOA, GPIO_Pin_15	//_平台调平检测




//#define SDPOWER_PIN          GPIOA, GPIO_Pin_0	

#define SDSS_PIN             GPIOC, GPIO_Pin_7	
#define SDCD_PIN             GPIOC, GPIO_Pin_6	
#define SDCLK_PIN            GPIOB, GPIO_Pin_13	
#define SDMISO_PIN           GPIOB, GPIO_Pin_14	
#define SDMOSI_PIN           GPIOB, GPIO_Pin_15	


//#define PS_ON_PIN           GPIOA, GPIO_Pin_0	
//#define KILL_PIN            GPIOA, GPIO_Pin_0	





#define SET(PIN)          GPIO_SetBits(PIN)
#define CLR(PIN)          GPIO_ResetBits(PIN)

#define WRITE(PIN,VALUE)  GPIO_WriteBits(PIN,VALUE)
#define READ(PIN)         GPIO_ReadInputDataBit(PIN)

#define SET_OUTPUT(PIN)   GPIO_OutputBits(PIN)
#define SET_INPUT(PIN)    GPIO_InputBits(PIN)

#define MODE(PIN,MODE)    GPIO_ModeBits(PIN,MODE)

#ifdef __cplusplus
 extern "C" {
#endif

void GPIO_ClockSet(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, FunctionalState enable);
void GPIO_ModeBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef mode);
void GPIO_OutputBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_InputBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t BitVal);
void pwm_init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,int val);

#ifdef __cplusplus
 }
#endif

#endif

