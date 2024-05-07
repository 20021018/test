#ifndef BSB_USART_H
#define BSB_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "ch32v30x.h"
#include "../marlin/Marlin.h"

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define CHAR 0

#define SERIAL_PORT 2

#define test_PORT 12
//#if SERIAL_PORT == 1
//#define USART1 USART1
//#elif SERIAL_PORT == 2
//#define USART1 USART2
//#elif SERIAL_PORT == 3
//#define USART1 USART3
//#endif


#define RX_BUFFER_SIZE 128



typedef struct 
{
  unsigned char buffer[RX_BUFFER_SIZE];
  int head;
  int tail;
}ring_buffer;

extern ring_buffer rx_buffer;


extern volatile unsigned long last_print_time; //为了配合上位机
//#define LAST_PRINT_TIME last_print_time


void USART2_IRQHandler(void);

#ifdef __cplusplus
}
#endif





#ifdef __cplusplus
class  MarlinSerial
{
public:
  MarlinSerial(){};
  void begin(uint32_t baud);
  void end();
  int peek(void);
  int read(void);
  void flush(void);
    
		//串口接收数据个数
  FORCE_INLINE int available(void)
  {
   return (unsigned int)(RX_BUFFER_SIZE + rx_buffer.head - rx_buffer.tail) % RX_BUFFER_SIZE;
  }
    
	//串口数据发送函数
	FORCE_INLINE void write(uint8_t c)
	{
#if test_PORT==1
		while(!(USART1->STATR&USART_STATR_TXE)){};
		USART1->DATAR = c;
#endif 
#if SERIAL_PORT==2
	    while(!(USART2->STATR&USART_STATR_TXE)){};
	    USART2->DATAR = c;

#endif
	}
    
    
  FORCE_INLINE void checkRx(void)
  {
#if test_PORT==1
		if((USART1->STATR&USART_STATR_RXNE)!=0)
		{
			unsigned char c = USART1->DATAR;
			int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;
			if (i != rx_buffer.tail) 
			{
				rx_buffer.buffer[rx_buffer.head] = c;
				rx_buffer.head = i;
			}
     }
#endif 

#if SERIAL_PORT==2
	    if((USART2->STATR&USART_STATR_RXNE)!=0)
	        {
	            unsigned char c = USART2->DATAR;
	            int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;
	            if (i != rx_buffer.tail)
	            {
	                rx_buffer.buffer[rx_buffer.head] = c;
	                rx_buffer.head = i;
	            }
	     }
#endif
 }    
 
public:
   FORCE_INLINE void write(const char *str)
   {
     while (*str)
       write(*str++);
   }

    FORCE_INLINE void write(const uint8_t *buffer, uint32_t size)
    {
      while (size--)
        write(*buffer++);
    }
    
} ;

extern MarlinSerial MSerial;
#endif

#endif










