#ifndef BSP_ADC_H
#define BSP_ADC_H


#include "ch32v30x.h"
#ifdef __cplusplus
 extern "C" {
#endif




void Adc1_Init(uint16_t ADC_Channel);



u16 get_ADC_Result(void);



#ifdef __cplusplus
}
#endif


#endif
