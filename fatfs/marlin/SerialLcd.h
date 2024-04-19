/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  Modified 28 September 2010 by Mark Sproul
*/

#ifndef SerialLcd_h
#define SerialLcd_h

#include "Marlin.h"
#include "cardreader.h"
#include "stepper.h"
#include "temperature.h"
#include "Configuration.h"
#include "Configuration.h"

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.
#define LCD_RX_BUFFER_SIZE 64


int int_to_str(int uint,char *pchar,char *plen);

struct uart_buffer
{
  unsigned char buffer[RX_BUFFER_SIZE];
  int head;
  int tail;
};


extern uart_buffer lcd_buffer;


class SerialLcd //: public Stream
{
  public:
  
	  uint16_t current_page;
	
    SerialLcd();
    void begin(long);
    void end();
    int peek(void);
    int read(void);
    void flush(void);
    
    FORCE_INLINE int available(void)
    {
      return (unsigned int)(RX_BUFFER_SIZE + lcd_buffer.head - lcd_buffer.tail) % RX_BUFFER_SIZE;
    }
    
    FORCE_INLINE void write(uint8_t c)
    {
				while(!(UART4->STATR&USART_STATR_TXE)){};
				UART4->DATAR = c;
    }
    
    
    FORCE_INLINE void checkRx(void)
    {
        unsigned char c  =  0;//M_UDRx;
        int i = (unsigned int)(lcd_buffer.head + 1) % RX_BUFFER_SIZE;

        // if we should be storing the received character into the location
        // just before the tail (meaning that the head would advance to the
        // current location of the tail), we're about to overflow the buffer
        // and so we don't write the character or advance the head.
        if (i != lcd_buffer.tail) {
          lcd_buffer.buffer[lcd_buffer.head] = c;
          lcd_buffer.head = i;
        }
      }

     
    private:
    void printNumber(unsigned long, uint8_t);
    void printFloat(double, uint8_t);
    
    
  public:
    FORCE_INLINE void write(const char *str)
    {
		char i=0;
     // while (*str)
       // write(*str++);
	   while(*(str+i)!='\0')
	   {
		   write(*(str+i));
		   i++;
	   }
    }
    FORCE_INLINE void write(const uint8_t *buffer, size_t size)
    {
      while (size--)
        write(*buffer++);
    }
	
	//LCD串口屏选择，例如 page main或者page0
	void Set_Page(const char *page,uint16_t pagen);
	uint16_t LCD_Get_Value(void);
	// 发送读取当前页面的指令
	void Get_Page(void);
	void Get_Value(char *pval,char n);
	void Set_Txt(char t,char *txt);
	void Set_Txt(char t,const char *txt);
	void Set_Txt(char t,uint16_t u_int);
	void Set_Txt(char t,float fdata);
	void Get_Num(char num);
		
	void Set_Txt_Color(uint8_t t,char* color);
	void Set_Num(char n,int val);
	void Get_Txt(char *t);
	uint8_t	LCD_Run(void);
	void Set_Display(char *w,char flag);
	void Set_PIC(char n,int npic);
	uint8_t Search_Date_Tail(void);
	uint8_t Search_Date_Head(void);
	uint8_t LCD_Key_Swtich(void);
	uint8_t LCD_Cmd_Invalid(void);
	uint8_t LCD_Parameter_Invalid(void);
	uint8_t LCD_Page_Invalid(void);
	void Update(void);
	uint16_t LCD_Press_Key(char page,char vaule);
	void Set_HValue(char vn,int vdata);

	//LCD屏串口数据读取
	
	int writecmd(uint8_t *pcmd,uint8_t cmdlen);	
	void Display_Status(void);
	
	void Set_Progress(uint8_t jn,uint8_t pro);
	
	void uinttostr(uint16_t uuint,char *str);
};


extern SerialLcd MySerialLcd;

#endif // !AT90USB



























