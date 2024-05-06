#ifndef PIN_H
#define PIN_H

#include "ch32v30x.h"

#include "../marlin/Marlin.h"

#define MASK(N)  (1<<N)

//#define HEATER_0_PIN        GPIOC, GPIO_Pin_0	 //������1
//#define LED_PIN            GPIOC, GPIO_Pin_2	 //MOS���LED

//#define HEATER_BED_PIN      GPIOC, GPIO_Pin_3	 //�ȴ�              û���ȴ�

#define HEATER_0_PIN        GPIOC, GPIO_Pin_2	 //������1
//#define FAN_PIN             GPIOC, GPIO_Pin_1	 //����������
#define LED_PIN            GPIOE, GPIO_Pin_0	 //MOS���LED ->PE0

#define BUZ_PIN            GPIOA, GPIO_Pin_12 	//������

#define POWER_CHK       	GPIOA, GPIO_Pin_11	 //��Դ������

//#define X_MIN_PIN           GPIOB, GPIO_Pin_7	   //XMIN��λ����
//#define X_MAX_PIN         	GPIOB, GPIO_Pin_8
#define X_MAX_PIN           GPIOB, GPIO_Pin_7      // ֻ��ҪXMAX,����
#define X_MIN_PIN           GPIOB, GPIO_Pin_8

#define Y_MIN_PIN           GPIOB, GPIO_Pin_9	
#define Y_MAX_PIN         	GPIOC, GPIO_Pin_13	

#define Z_MAX_PIN           GPIOB, GPIO_Pin_10  // ֻ��ҪZMAX,����
#define Z_MIN_PIN           GPIOC, GPIO_Pin_15  //
//#define Z_MIN_PIN           GPIOB, GPIO_Pin_10  //
//#define Z_MAX_PIN         	GPIOC, GPIO_Pin_15  //


#define X_STEP_PIN          GPIOB, GPIO_Pin_2	  //X����
#define X_DIR_PIN           GPIOC, GPIO_Pin_5	
#define X_ENABLE_PIN        GPIOC, GPIO_Pin_4
#define X_STOP_PIN          GPIOC, GPIO_Pin_4

#define Y_STEP_PIN          GPIOA, GPIO_Pin_7	  //Y����
#define Y_DIR_PIN           GPIOA, GPIO_Pin_6	
#define Y_ENABLE_PIN        GPIOC, GPIO_Pin_4	
#define Y_STOP_PIN          GPIOC, GPIO_Pin_4	

#define Z_STEP_PIN          GPIOA, GPIO_Pin_15	  //Z����
#define Z_DIR_PIN           GPIOA, GPIO_Pin_9	  //_��PA3 �ĵ���PA14 ,���޸�ΪPA9
#define Z_ENABLE_PIN        GPIOC, GPIO_Pin_4
#define Z_STOP_PIN          GPIOC, GPIO_Pin_4

#define E0_STEP_PIN         GPIOA, GPIO_Pin_1	 //�������
#define E0_DIR_PIN          GPIOA, GPIO_Pin_0	
#define E0_ENABLE_PIN       GPIOC, GPIO_Pin_4

// #define EE_SCL_PIN    			GPIOA, GPIO_Pin_3	
// #define EE_SDA_PIN     			GPIOA, GPIO_Pin_2	


//û���ȴ��¶ȴ�����
//#define TEMP_BED_PIN        GPIOB, GPIO_Pin_0
//#define TEMP_BED_CHANNEL    ADC_Channel_8   //��Ӧ��ADC����ͨ��

#define TEMP_0_PIN          GPIOC, GPIO_Pin_1	     
#define TEMP_0_CHANNEL      ADC_Channel_11  //��Ӧ��ADC����ͨ��

//#define PLATFORM_PIN    GPIOA, GPIO_Pin_15	//_ƽ̨��ƽ���




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

