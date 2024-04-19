#include "SerialLcd.h"
#include "cardreader.h"
#include "Marlin.h"
#include "bsp_pin.h"

#define LCD_ACK_LENGTH      0x07   //串口屏返回数据长度是7,第一个字符是0x65,最后3个字符是0xff,0xff,0xff
//////////////////////////////main页面///////////////////////////////////////////
#define P0BT0_SDPRINT_CN    "打印"   //页面1--SD打印按键
#define P0BT1_CONFIG_CN     "配置"  //页面1--配置按键
#define P0BT2_PLATFORM_CN   "平台"   //页面1--手动调平按键
#define P0BT3_MOVE_CN       "移动"  //页面1--移动按键
#define P0BT4_HEAT_CN       "加热"   //页面1--加热按键
#define P0BT5_SYS_CN        "系统"  //页面1--系统按键

#define P0BT0_SDPRINT_EN    "print"   //页面1--SD打印按键
#define P0BT1_CONFIG_EN     "Configure"  //页面1--配置按键
#define P0BT2_PLATFORM_EN   "Platform"   //页面1--手动调平按键
#define P0BT3_MOVE_EN       "Move"  //页面1--移动按键
#define P0BT4_HEAT_EN       "Heat"   //页面1--加热按键
#define P0BT5_SYS_EN        "System"  //页面1--系统按键

#define P0BT0_SDPRINT    0x65000101   //页面1--SD打印按键返回值
#define P0BT1_CONFIG     0x65000201  //页面1--配置按键返回值
#define P0BT2_PLATFORM   0x65000301   //页面1--手动调平按键返回值
#define P0BT3_MOVE       0x65000401  //页面1--移动按键返回值
#define P0BT4_HEAT       0x65000501   //页面1--加热按键返回值
#define P0BT5_SYS        0x65000601  //页面1--系统按键返回值

////////////////////////////sd页面//////////////////////////////////
#define P4BT3_LAST_CN       "上一页"
#define P4BT4_MAIN_CN       "主页"
#define P4BT5_NEXT_CN       "下一页"

#define P4BT3_LAST_EN       "Last"   
#define P4BT4_MAIN_EN       "Main"     
#define P4BT5_NEXT_EN       "Next"   

#define P4BT3_LAST       0x65040501   
#define P4BT4_MAIN       0x65040701     
#define P4BT5_NEXT       0x65040601
#define P4T0_TXT         0x65040101
#define P4T1_TXT         0x65040201
#define P4T2_TXT         0x65040301
#define P4T3_TXT         0x65040401
////////////////////////////print页面///////////////////////////////////
#define P7BT0_PAUSE       0x65070a01   
//#define P7BT0_CONTINUE    0x650701 
#define P7BT11_CANCEL     0x65070901      
#define P7BT10_MORE       0x65070801
//#define P7T4_TXT          0x65040101
//#define P7T5_TXT          0x65040201
#define P7N0_TXT          0x65040301
#define P7N1_TXT          0x65040401
//////////////////////sdsure///////////////////////////////////////////
#define P17BT0_SURE          0x65110101
#define P17BT1_CANCEL        0x65110201


////////////////////////sdcancel/////////////////////////////////
#define P18BT0_YES       0x65120101
#define P18BT1_NO        0x65120201
//////////////////////move////////////////////////
#define P5BT9_LAST   0x65051601
#define P5BT11_MAIN   0x65051701
#define P5BT10_NEXT   0x65051e01

#define P5BT0_FRONT   0x65050101
#define P5BT1_BACK    0x65050201
#define P5BT2_LEFT    0x65050301
#define P5BT3_RIGHT   0x65050401

#define P5BT4_UP     0x65050501
#define P5BT5_DOWN   0x65050601

#define P5CK0_10     0x65050701
#define P5CK1_1      0x65050801
#define P5CK2_01     0x65050901

uart_buffer lcd_buffer  =  { { 0 }, 0, 0 };
uint8_t read_page = 0;

long int timeout=0;
SerialLcd MySerialLcd;
volatile uint16_t get_value=0;

int int_to_str(int uint,char *pchar,char *plen)
{
		uint8_t i,j,k,len;
		char xx[5]={'0','\0','\0','\0','\0'};
		if(uint)
		{
			xx[0] = (uint/10000)%10 + '0';
			xx[1] = (uint/1000)%10 + '0';
			xx[2] = (uint/100)%10 + '0';
			xx[3] = (uint/10)%10 + '0';
			xx[4] = uint%10 + '0';
			
			for(i=0;i<5;i++) //寻找到第一个不是0的数据
			{
				if(xx[i]!='0')
				{
					break;
				}
			}
			k=i;
			for(j=0;j<5;j++) //重新拷贝数据
			{
				if(j<5-k)
				xx[j] = xx[i++];
				else
				xx[j] = '\0';
			}
			len = 0;
			for(i=0;i<5;i++) //把数据拷贝出去
			{
					if(xx[i] == '\0')
					{
						break;
					}
					else
					{
						*pchar = xx[i];
						len++;
					}
			}
			*plen = len;
		}
		return 0;
}



FORCE_INLINE void lcd_store_char(unsigned char c)
{
  int i = (unsigned int)(lcd_buffer.head + 1) % LCD_RX_BUFFER_SIZE;
  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != lcd_buffer.tail) {
    lcd_buffer.buffer[lcd_buffer.head] = c;
    lcd_buffer.head = i;
  }
	
}

#define PLATFOMR_SCREW  0.7    //平台调平螺丝的螺纹距，（M4的大号螺纹距是0.7，M3的螺纹距是0.5）

volatile uint16_t which_value=0;

static float move_scale=10.0;
static uint8_t heat_page = 0;	//温度设置页面
static uint8_t status_page = 0; //打印状态页面


static int16_t filepage = 0;
static uint16_t fileCnt=0;	//显示当前目录下又几个文件（包括文件夹）
static uint16_t file_i=0;	//记录当前目录下的第几个文件

static uint32_t printtime = 0;

float pause_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };


static uint8_t autocalibration_step = 0; //用来表示调整平台的步骤
static uint8_t autocalibration_ok = 0;
static float screw1_height=0;
static float screw2_height=0;
static float screw3_height=0;
static float screw4_height=0;
static float center_height=0;
static float average_height=0;

float ffabs(float a,float b)
{
	if(a>b)
	{
		return a-b;
	}
	else
	{
		return b-a;
	}
}


void lcd_delay_nus(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=10;  
      while(i--) ;    
   }
}


SerialLcd::SerialLcd()
{

}

// Public Methods //////////////////////////////////////////////////////////////

void SerialLcd::begin(long baud)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC , ENABLE);
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART4, ENABLE);
		
	//GPIO_PinRemapConfig(GPIO_PartialRemap_UART4,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);		    
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	/* USART1 configuration ------------------------------------------------------*/
	/* USART and USART2 configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure USART1 */
  USART_Init(UART4, &USART_InitStructure);

	USART_ClearITPendingBit(UART4, USART_IT_RXNE);  
	USART_ClearITPendingBit(UART4, USART_IT_TC);  
	
  /* Enable USART1 Receive and Transmit interrupts */
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
//  USART_ITConfig(USARTx, USART_IT_TC, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  

  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;

  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  /* Enable the USART1 */
  USART_Cmd(UART4, ENABLE);	
}


#ifdef __cplusplus
extern "C" {
#endif
/***************中断接收数据****************/
void UART4_IRQHandler(void)  
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE)!= RESET)  
	{
		unsigned char c = UART4->DATAR;
		lcd_store_char(c);     
		timeout = millis()+100;
		
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);  
	}	
	if(USART_GetITStatus(UART4, USART_IT_TC)!= RESET)  
	{
		USART_ClearITPendingBit(UART4, USART_IT_TC);  
	}	
}

#ifdef __cplusplus
	}
#endif

void SerialLcd::end()
{

	
}



int SerialLcd::peek(void)
{
  if (lcd_buffer.head == lcd_buffer.tail) {
    return -1;
  } else {
    return lcd_buffer.buffer[lcd_buffer.tail];
  }
}

int SerialLcd::read(void)
{
  //if the head isn't ahead of the tail, we don't have any characters
  if (lcd_buffer.head == lcd_buffer.tail) {
    return -1;
  } else {
    unsigned char c = lcd_buffer.buffer[lcd_buffer.tail];
    lcd_buffer.tail = (unsigned int)(lcd_buffer.tail + 1) % LCD_RX_BUFFER_SIZE;
    return c;
  }
}



int SerialLcd::writecmd(uint8_t *pcmd,uint8_t cmdlen)
{
	
	if ((RX_BUFFER_SIZE - (lcd_buffer.head + RX_BUFFER_SIZE - lcd_buffer.tail)%RX_BUFFER_SIZE) > cmdlen)
	{	 	 
		for(char i=0;i<cmdlen;i++)
		{
			 lcd_buffer.buffer[lcd_buffer.head]=*(pcmd+i);
			 lcd_buffer.head = (unsigned int)(lcd_buffer.head + 1) % LCD_RX_BUFFER_SIZE;
		}
		return 1;
	}
	 return 0;
}


void SerialLcd::flush()
{
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // were full, not empty.
  lcd_buffer.head = lcd_buffer.tail;
}

//LCD串口屏选择，例如 page main或者page0
void SerialLcd::Set_Page(const char *page,uint16_t pagen)
{
	write("page");
	write(" ");
	write(page);
	write(0xff);
	write(0xff);
	write(0xff);
	this->current_page = pagen;
}
	// 发送读取当前页面的指令
void SerialLcd::Get_Page(void)
{
	write("sendme");
	write(0xff);
	write(0xff);
	write(0xff);
}

	// 发送读取当前页面的指令
void SerialLcd::Get_Num(char num)
{
	write("get ");
	write('n');
	if(num<=9)
	{
		write(num+'0');
	}
	else
	{
		write(num/10+'0');	//十位
		write(num%10+'0');	//个位
	}
	write(".val");
	write(0xff);
	write(0xff);
	write(0xff);
}
	// 发送读取当前页面的指令
void SerialLcd::Get_Value(char *pval,char n)
{	
	write("get");
	write(" ");
	write(pval);
	write(0xff);
	write(0xff);
	write(0xff);
	which_value=n;
}

void SerialLcd::Set_Display(char *w,char flag)
{
	write("vis");
	write(" ");
	write(w);
	write(",");
	if(flag)
		write("1");
	else
		write("0");
	write(0xff);
	write(0xff);
	write(0xff);	
}


//设置图片
void SerialLcd::Set_PIC(char n,int npic)
{
		write("p");
		if(n<=9)
		{
			write(n+'0');
		}
		else
		{
			write(n/10+'0');	//十位
			write(n%10+'0');	//个位
		}
		write(".pic=");
		if(npic<=9)
		{
			write(npic+'0');
		}
		else
		{
			write(npic/10+'0');	//十位
			write(npic%10+'0');	//个位
		}
		write(0xff);
		write(0xff);
		write(0xff);
}

//设置数字显示,范围是0-999
void SerialLcd::Set_Num(char n,int val)
{
	char i,k,j;
	char xx[3]={'0','0','0'};
	xx[0] = (val/100)%10 + '0';		
	xx[1] = (val/10)%10 + '0';
	xx[2] = val%10 + '0';		//各位
	
	for(i=0;i<3;i++)
	{
		if(xx[i]!='0')
		{
			break;
		}
	}
	k=i;
	for(j=0;j<3;j++)
	{
		if(j<3-k)
		xx[j] = xx[i++];
		else
		xx[j] = '\0';
	}		
	
	if(n>100) 
	{
		return;
	}
	
	write('n');
	if(n<=9)
	{
		write(n+'0');
	}
	else
	{
		write(n/10+'0');	//十位
		write(n%10+'0');	//个位
	}
	write(".val=");
	write(xx);
	write(0xff);
	write(0xff);
	write(0xff);
}

// 设置滑动条的数据
void SerialLcd::Set_HValue(char t,int uint)
{
		uint8_t i,j,k;
		char xx[5]={'0','\0','\0','\0','\0'};
		if(uint)
		{
			xx[0] = (uint/10000)%10 + '0';
			xx[1] = (uint/1000)%10 + '0';
			xx[2] = (uint/100)%10 + '0';
			xx[3] = (uint/10)%10 + '0';
			xx[4] = uint%10 + '0';
			
			for(i=0;i<5;i++)
			{
				if(xx[i]!='0')
				{
					break;
				}
			}
			k=i;
			for(j=0;j<5;j++)
			{
				if(j<5-k)
				xx[j] = xx[i++];
				else
				xx[j] = '\0';
			}
		}
		write('h');
		if(t<=9)
		{
			write(t+'0');
		}
		else
		{
			write(t/10+'0');	//十位
			write(t%10+'0');	//个位
		}
		write(".val=");
		write(xx);
		write(0xff);
		write(0xff);
		write(0xff);
}

//t的范围是0~99；
void SerialLcd::Set_Txt(char t,char *txt)
{
	write('t');
	if(t<=9)
	{
		write(t+'0');
	}
	else
	{
		write(t/10+'0');	//十位
		write(t%10+'0');	//个位
	}
	write(".txt=");
	write('"');
	write(txt);
	write('"');
	write(0xff);
	write(0xff);
	write(0xff);
}


void SerialLcd::Set_Txt(char t,const char *txt)
{
	write('t');
	if(t<=9)
	{
		write(t+'0');
	}
	else
	{
		write(t/10+'0');	//十位
		write(t%10+'0');	//个位
	}
	write(".txt=");
	write('"');
	write(txt);
	write('"');
	write(0xff);
	write(0xff);
	write(0xff);
}

//t的范围是0~99；pro的范围是0-100
void SerialLcd::Set_Progress(uint8_t n,uint8_t pro)
{
	char i,k,j;
	char xx[3]={'0','0','0'};
	xx[0] = (pro/100)%10 + '0';		
	xx[1] = (pro/10)%10 + '0';
	xx[2] = pro%10 + '0';		//各位
	
	for(i=0;i<3;i++)
	{
		if(xx[i]!='0')
		{
			break;
		}
	}
	k=i;
	for(j=0;j<3;j++)
	{
		if(j<3-k)
		xx[j] = xx[i++];
		else
		xx[j] = '\0';
	}		
	
	if(pro>100) 
	{
		return;
	}
	
	write('j');
	if(n<=9)
	{
		write(n+'0');
	}
	else
	{
		write(n/10+'0');	//十位
		write(n%10+'0');	//个位
	}
	write(".val=");
	write(xx);
	write(0xff);
	write(0xff);
	write(0xff);
}

//t的范围是0~99；
void SerialLcd::Set_Txt(char t,uint16_t uint)
{
	uint8_t i,j,k;
	char xx[5]={'0','\0','\0','\0','\0'};
	if(uint)
	{
		xx[0] = (uint/10000)%10 + '0';
		xx[1] = (uint/1000)%10 + '0';
		xx[2] = (uint/100)%10 + '0';
		xx[3] = (uint/10)%10 + '0';
		xx[4] = uint%10 + '0';
	
		for(i=0;i<5;i++)
		{
			if(xx[i]!='0')
			{
				break;
			}
		}
		k=i;
		for(j=0;j<5;j++)
		{
			if(j<5-k)
				xx[j] = xx[i++];
			else
				xx[j] = '\0';
		}
	}
	
	write('t');
	if(t<=9)
	{
		write(t+'0');
	}
	else
	{
		write(t/10+'0');	//十位
		write(t%10+'0');	//个位
	}
	write(".txt=");
	write('"');
	write(xx);
	write('"');
	write(0xff);
	write(0xff);
	write(0xff);
}


//t的范围是0~99；
//fdata的范围是：-99999.99~99999.99
void SerialLcd::Set_Txt(char t,float fdata)
{
	uint8_t i,j,k,nflag=0;
	char xx[9]={'0','0','0','0','0','0','.','\0','\0'};
	
	if((fdata<-99999.99)||(fdata>99999.99))
	{
		return;
	}
	if(fdata<0) //如果显示是负数
	{
		fdata = -fdata;
		nflag = 1;
	}
	uint32_t uint= (uint32_t)(fdata*100);	//整数部分
	if(uint)
	{
		xx[0] = (uint/10000000)%10 + '0';	
		xx[1] = (uint/1000000)%10 + '0';
		xx[2] = (uint/100000)%10 + '0';
		xx[3] = (uint/10000)%10 + '0';
		xx[4] = (uint/1000)%10 + '0';
		xx[5] = (uint/100)%10 + '0';
		xx[6] = '.';
		xx[7] = (uint/10)%10 + '0';		
		xx[8] = uint%10 + '0';		

		for(i=0;i<5;i++) //检查整数部分，非零的最高位
		{
			if(xx[i]!='0')
			{
				break;
			}
		}
		k=i; 
		for(j=0;j<9;j++)
		{
			if(nflag)	//如果是负数
			{
				if(j==0) 
				{
					xx[j]='-';
				}
				else if(j<9-k+1) //把非零的整数从地址0开始拷贝
				{
					xx[j] = xx[i++];
				}
				else
				{
					xx[j] = '\0';
				}
			}
			else //数据是正数
			{
				if(j<9-k) //把非零的整数从地址0开始拷贝
				{
					xx[j] = xx[i++];
				}
				else
				{
					xx[j] = '\0';
				}
			}
		}
	}
	else //如果数据
	{
		xx[0] = '0';
		xx[1] = '.';
		xx[2] = '0';
		xx[3] = '\0';
		xx[4] = '\0';
		xx[5] = '\0';
		xx[6] = '\0';
	}
	
	write('t');
	if(t<=9)
	{
		write(t+'0');
	}
	else
	{
		write(t/10+'0');	//十位
		write(t%10+'0');	//个位
	}
	write(".txt=");
	write('"');
	write(xx);
	write('"');
	write(0xff);
	write(0xff);
	write(0xff);		
	
	write('t');
	if(t<=9)
	{
		write(t+'0');
	}
	else
	{
		write(t/10+'0');	//十位
		write(t%10+'0');	//个位
	}
	write(".txt=");
	write('"');
	write(xx);
	write('"');
	write(0xff);
	write(0xff);
	write(0xff);
}

void SerialLcd::Set_Txt_Color(uint8_t t,char* color)
{
	write(t);
	write(".pco=");
	write(color);
	write(0xff);
	write(0xff);
	write(0xff);
}

void SerialLcd::Get_Txt(char *t)
{
	write("get");
	write(" ");
	write(t);
	write(".txt");
	write(0xff);
	write(0xff);
	write(0xff);
}

//无效指令：0x00 0xff 0xff 0xff
//无效参数：0x1e 0xff 0xff 0xff
//无效页面：0x03 0xff 0xff 0xff
//按键返回：0x65 0x00（第几页） 0x00（第几个按键） 0x00 0xff 0xff 0xff
uint8_t SerialLcd::Search_Date_Head(void)
{
	char c;
	while(this->available())
	{
		c =this->read();
		switch(c)
		{
			case 0x00:	//无效指令;
			case 0x02:
			case 0x03:
			case 0x1e:
			case 0x65: //返回按键值
			case 0x66: //返回当前页面值
			case 0x71: //数据返回
				return c;
			default:
				break;
		}
	}
	return 0xff;
}

//数据尾是3个连续的0xff,0xff,0xff;
uint8_t SerialLcd::Search_Date_Tail(void)
{
	char c,flag=0;
	while(this->available())
	{
		c = this->read();
		if(c==0xff)
		{
			flag++;
			if(flag==3)
			{
				return 0;
			}
		}
		else
		{
			flag=0;
		}
	}
	return 0xff;
}

//无效指令：0x00 0xff 0xff 0xff
//无效参数：0x1e 0xff 0xff 0xff
//无效页面：0x03 0xff 0xff 0xff
//按键返回：0x65 0x00（第几页） 0x00（第几个按键） 0x00 0xff 0xff 0xff


//
//LCD的输入页面无效
uint8_t SerialLcd::LCD_Page_Invalid(void)
{
		return 0;	
}

//
//LCD的输入参数无效
uint8_t SerialLcd::LCD_Parameter_Invalid(void)
{
		return 0;
}

//LCD的返回无效指令处理函数
uint8_t SerialLcd::LCD_Cmd_Invalid(void)
{	
	return 0;
}
//LCD的返回无效指令处理函数
uint16_t SerialLcd::LCD_Get_Value(void)
{
	uint8_t ch;
	uint8_t v1,v2;
	uint16_t value=0;
	ch = MySerialLcd.Search_Date_Head();
	if(ch!=0x71) return 0xffff;
	
	v1 = MySerialLcd.read();	//读数据地位
	v2 = MySerialLcd.read();  //读数据高位
	value = v2;	//读按键值
	value<<=8;
	value|=v1;
	return value;
}

//无效指令：0x00 0xff 0xff 0xff
//无效参数：0x1e 0xff 0xff 0xff
//无效页面：0x03 0xff 0xff 0xff
//按键返回：0x65 0x00（第几页） 0x00（第几个按键） 0x00 0xff 0xff 0xff
uint8_t SerialLcd::LCD_Run(void)
{
	static uint32_t lcd_trap_time;	//触摸屏的处理间隔时间是100ms
	if(lcd_trap_time<millis())
	{
		lcd_trap_time = millis() + 100;
	}
	else
	{
		return 0;
	}
	if(timeout > millis())return 0; //等到数据串口接收数据超时，表示一帧数据结束

	char c = Search_Date_Head();
	switch(c)
	{
		case 0x00:	//无效指令;
			LCD_Cmd_Invalid();
		break;
		case 0x03: //无效页面
			LCD_Page_Invalid();
		break;
		case 0x1e: //无效参数
			LCD_Parameter_Invalid();
		break;
		case 0x65:	//返回按键值;
			LCD_Key_Swtich();
		break;
		case 0x66: //返回页面值
			read_page = MySerialLcd.read();
		break;
		break;
		default:
		break;
	}
	//c = Search_Date_Tail(); //读三个0xff,0xff,0xff
	MySerialLcd.Update(); //刷新页面显示
	return 0;
}
//LCD的按键返回处理函数
uint8_t SerialLcd::LCD_Key_Swtich(void)
{
	static long int key;
	static uint8_t disable_stepper_flag = 0;
	char buff[4]={0x65};
	int i,j,read_data;

	if(available()<4) return -1; //数据不够
	
	for(j=1;j<4;j++)//(3)复制数据
	{
			read_data=MySerialLcd.read();
			buff[j] = read_data;
	}
	
	key = buff[0];
	key <<= 8;
	key |= buff[1];
	key <<= 8;
	key |= buff[2];
	key <<= 8;
	key |= buff[3];
	
	switch(key)
	{
/////////////////////////////SD卡打印///////////////////////////////////
		case P0BT0_SDPRINT:	 
		//	card.initsd();
			static uint8_t lcd_oldcardstatus=0;
			if((IS_SD_INSERTED != lcd_oldcardstatus))	
			{
			    lcd_oldcardstatus = IS_SD_INSERTED;
				    
			    if(lcd_oldcardstatus)
			    {
				    card.initsd();
			    }
			    else
			    {
				    card.release();
			    }
			}
			if(!card.cardOK)//SD卡初始化错误
			{
				MySerialLcd.Set_Page("sderror",4);
				return 0;
			}	
			
			MySerialLcd.Set_Page("sd",4);	//切换到file文件名显示页面,一页只能显示5个文件名
			
			MySerialLcd.Set_Txt(0," ");
			MySerialLcd.Set_Txt(1," ");
			MySerialLcd.Set_Txt(2," ");
			MySerialLcd.Set_Txt(3," ");
			
			fileCnt = card.getfilecount(card.path); //总的文件数
				
			char i;
			file_i = 0;
			for(i=0;i<min(fileCnt,4);i++,file_i++)
			{
					card.getfilename(file_i+1,card.path);
					MySerialLcd.Set_Txt(i,FILE_VALID_NAME);
			}
			filepage = 0;
		break;
///////////////////P0BT1_CONFIG/////////////////////////////////////////
		case P0BT1_CONFIG:	//main界面的模型选择按键
			MySerialLcd.Set_Page("configure",12);
		break;
////////////////////////////P0BT2_PLATFORM/////////////////////////////////////////////		
		case P0BT2_PLATFORM: //设置功能
			MySerialLcd.Set_Page("level",13);	
		break;	
//////////////////////////P0BT3_MOVE/////////////////////////////////////////////
		case P0BT3_MOVE:
			MySerialLcd.Set_Page("move",5);
		break;
///////////////////////////P0BT4_HEAT////////////////////////////////		
		case P0BT4_HEAT:
			MySerialLcd.Set_Page("heat",6);
		break;
/////////////////////////P0BT5_SYS//////////////////////////////				
		case P0BT5_SYS:
			MySerialLcd.Set_Page("sys",10);
		break;

/////////////////////////SD卡页////////////////////////////////////
		case P4T0_TXT://打印第一行
			file_i = 4*filepage+1;
			if(file_i>fileCnt) return 0;
		
			card.getfilename(file_i,card.path);
			if(FILE_IS_DIR) //如果选择的是文件夹
			{
				card.downdir(FILE_VALID_NAME);
				MySerialLcd.LCD_Press_Key(1,1);
				return 0;
			}
			if(strstr(FILE_VALID_NAME,".gcode")==NULL)//如果选择的不是gcode文件
			{
				return 0;
			}
			MySerialLcd.Set_Page("sdsure",14);	//打印确认
			MySerialLcd.Set_Txt(1,FILE_VALID_NAME); //显示文件名称

		break;
		case P4T1_TXT://打印第二行
			file_i = 4*filepage+2;
			if(file_i>fileCnt) return 0;
		
			card.getfilename(file_i,card.path);
			if(FILE_IS_DIR) //如果选择的是文件夹
			{
				card.downdir(FILE_VALID_NAME);
				MySerialLcd.LCD_Press_Key(1,1);
				return 0;
			}
			if(strstr(FILE_VALID_NAME,".gcode")==NULL)//如果选择的不是gcode文件
			{
				return 0;
			}
			MySerialLcd.Set_Page("sdsure",14);	//打印确认
			MySerialLcd.Set_Txt(1,FILE_VALID_NAME); //显示文件名称
		break;
		case P4T2_TXT://打印第三行
			file_i = 4*filepage+3;
			if(file_i>fileCnt) return 0;
		
			card.getfilename(file_i,card.path);
			if(FILE_IS_DIR) //如果选择的是文件夹
			{
				card.downdir(FILE_VALID_NAME);
				MySerialLcd.LCD_Press_Key(1,1);
				return 0;
			}
			if(strstr(FILE_VALID_NAME,".gcode")==NULL)//如果选择的不是gcode文件
			{
				return 0;
			}
			MySerialLcd.Set_Page("sdsure",14);	//打印确认
			MySerialLcd.Set_Txt(1,FILE_VALID_NAME); //显示文件名称
			
		break;
		case P4T3_TXT://打印第4行
			file_i = 4*filepage+4;
			if(file_i>fileCnt) return 0;
		
			card.getfilename(file_i,card.path);
			if(FILE_IS_DIR) //如果选择的是文件夹
			{
				card.downdir(FILE_VALID_NAME);
				MySerialLcd.LCD_Press_Key(1,1);
				return 0;
			}
			if(strstr(FILE_VALID_NAME,".gcode")==NULL)//如果选择的不是gcode文件
			{
				return 0;
			}
			MySerialLcd.Set_Page("sdsure",14);	//打印确认
			MySerialLcd.Set_Txt(1,FILE_VALID_NAME); //显示文件名称
			
		break;
		case P4BT3_LAST://上一页
			if(filepage>0)
			{
				filepage--;
				file_i = 4*filepage;
				char i;
				for(i=0;i<min(fileCnt,4);i++,file_i++)
				{
						card.getfilename(file_i+1,card.path);
						MySerialLcd.Set_Txt(i,FILE_VALID_NAME);
				}
			}
			else
			{
					if(card.isroot())	//如果在根目录下返回main页面
					{
						MySerialLcd.Set_Page("main",0);
					}
					else
					{
						MySerialLcd.LCD_Press_Key(1,1);
						card.updir();
					}
			}
		break;
		case P4BT4_MAIN://返回主页
			MySerialLcd.Set_Page("main",0);
		break;
		case P4BT5_NEXT://下一页
			if((filepage+1)*4<=fileCnt)
			{
				MySerialLcd.Set_Txt(0," ");
				MySerialLcd.Set_Txt(1," ");
				MySerialLcd.Set_Txt(2," ");
				MySerialLcd.Set_Txt(3," ");
				
				filepage++;
				file_i = 4*filepage;
				char i;
				for(i=0;i<=min(fileCnt-file_i,4);i++,file_i++)
				{
						card.getfilename(file_i+1,card.path);
						MySerialLcd.Set_Txt(i,FILE_VALID_NAME);
				}
			}
		break;
////////////////sdsure///////////////////////////			
		case 	P17BT0_SURE:
			status_page = 0;  //打印页面状态
			print_total_layer=0;
			print_current_layer=0;
			print_total_time=0;
			stoptime = 0;
			MySerialLcd.Set_Page("print",7);
		 // MySerialLcd.Set_Num(0,(uint16_t)target_temperature[0]); //挤出设定温度
		//	MySerialLcd.Set_Num(1,(uint16_t)target_temperature[1]); //热床设定温度
			MySerialLcd.Set_Txt(7,card.fno.lfname);	
			card.openFile(card.fno.lfname,true);
			card.startFileprint();
			starttime=millis();	//开始打印的时间
		break;			
		case P17BT1_CANCEL: //取消打印
			this->Set_Page("sd",4);
		break;
///////////////////////print页面////////////////////////////////
		case P7BT0_PAUSE:
			
			static char pause=0;
			if(pause==0)
			{
					pause = 1;
					card.pauseSDPrint();
			}
			else
			{
					pause = 0;
					card.startFileprint();
			}
		break;
		case P7BT11_CANCEL:
				MySerialLcd.Set_Page("sdcancel",18);
		break;
		case P7BT10_MORE:
		break;
		
/////////////sdcacel//////////////////////////////////
		case P18BT0_YES:
			  card.sdprinting = false;
		    card.closefile();
		    quickStop();
				autotempShutdown();	
				MySerialLcd.Set_Page("main",0);
		break;
		case P18BT1_NO:
			MySerialLcd.Set_Page("print",7);
		break;
///////////////////////move/////////////////////////////		
		case P5BT9_LAST:
		case P5BT11_MAIN:
		case P5BT10_NEXT:
			MySerialLcd.Set_Page("main",0);
		break;	
		
		case P5BT0_FRONT:
			enquecommand("G91");
			enquecommand("G0 X10");
			enquecommand("G90");
		break;
		case P5BT1_BACK:
			enquecommand("G91");
			enquecommand("G0 X-10");
			enquecommand("G90");
		break;
		case P5BT2_LEFT:
			enquecommand("G91");
			enquecommand("G0 Y10");
			enquecommand("G90");	
		break;
		case P5BT3_RIGHT:
			enquecommand("G91");
			enquecommand("G0 Y-10");
			enquecommand("G90");	
		break;	
		case P5BT4_UP:
			enquecommand("G91");
			enquecommand("G0 Z10");
			enquecommand("G90");
		break;
		case P5BT5_DOWN:
			enquecommand("G91");
			enquecommand("G0 Z-10");
			enquecommand("G90");
		break;		
		case P5CK0_10:
		break;
		case P5CK1_1:
		break;	
		case P5CK2_01:
		break;
		
		
		default:
		break;
	}
	
	return 0;
}


uint16_t SerialLcd::LCD_Press_Key(char page,char value)
{
	uint8_t buff[]={0x65,0x00,0x00,0x00,0xff,0xff,0xff};
	
	buff[1] = page;
	buff[2] = value;
	
	writecmd(buff,sizeof(buff));
	return 0;
}



float get_platform_position(float x,float y)
{
	
	
	return 0;
}

void SerialLcd::Update(void)
{
	static uint32_t lcd_trap_time;	//触摸屏的处理间隔时间是500ms
	static uint8_t lcd_dis_step=0;
	
	if(lcd_trap_time<millis())
	{
		lcd_trap_time = millis() + 1000;
	}
	else
	{
		return;
	}
	if(MySerialLcd.current_page==0)
	{
		
	}
	else if(MySerialLcd.current_page==7)	//如果是print页面，则要刷新温度，时间，进度等等！
	{
			MySerialLcd.Set_Txt(7,card.fno.lfname);//显示正在打印的文件名称
			
			MySerialLcd.Set_Txt(4,(uint16_t)current_temperature[0]);//挤出机当前温度
		//	MySerialLcd.Set_Num(0,(uint16_t)target_temperature[0]);
		
			MySerialLcd.Set_Txt(5,(uint16_t)current_temperature[1]);//热床当前温度
		//	MySerialLcd.Set_Num(1,(uint16_t)target_temperature[1]);
			
			int val;
			MySerialLcd.Get_Num(0);
			lcd_delay_nus(2000);
			val = LCD_Get_Value();
			static int val_last;
			if(val!=0xffff)
			{
				if(val_last != val) //有设置温度
				{
					val_last = val;
					if(val>HEATER_0_MAXTEMP) //操作最大温度
					{
						MySerialLcd.Set_Num(0,HEATER_0_MAXTEMP);
					}	
					else
					{
						target_temperature[0]=val;
					}
				}
				else //没有设置温度
				{
					MySerialLcd.Set_Num(0,target_temperature[0]);
				}
			}
			MySerialLcd.Get_Num(1);
			lcd_delay_nus(2000);
			val = LCD_Get_Value();
			static int val_last1;
			if(val!=0xffff)
			{
				if(val_last1!=val)
				{
					val_last1=val;
					if(val>HEATER_1_MAXTEMP)//操作最大温度
					{
						MySerialLcd.Set_Num(1,HEATER_1_MAXTEMP);
					}
					else
					{
						target_temperature[1]=val;
					}
				}
				else
				{
					MySerialLcd.Set_Num(1,target_temperature[1]);
				}
			}
			uint16_t shours, sminutes;
			unsigned long t;
			stoptime=millis();	
			t=(stoptime-starttime)/1000;
			
			sminutes=(t/60)%60;
			shours=t/60/60;
			
			MySerialLcd.Set_Txt(10,(uint16_t)(shours));//总时间xx小时
			MySerialLcd.Set_Txt(11,(uint16_t)(sminutes));//总余时间xx分钟
			
			MySerialLcd.Set_Txt(10,(uint16_t)(print_total_time/60));//总时间xx小时
			MySerialLcd.Set_Txt(11,(uint16_t)(print_total_time%60));//总余时间xx分钟
			
			MySerialLcd.Set_Progress(0,(uint16_t)card.percentDone());	
	}
	else if(MySerialLcd.current_page==5)	//如果是status页面，则要刷新温度，时间，进度等等！
	{
		uint16_t shours, sminutes;
		unsigned long t;
		
		t=(stoptime-starttime)/1000;
		sminutes=(t/60)%60;
		shours=t/60/60;
		MySerialLcd.Set_Txt(0,FILE_VALID_NAME);//显示正在打印的文件名称
		MySerialLcd.Set_Txt(1,shours);//显示正在打印的文件名称
		MySerialLcd.Set_Txt(2,sminutes);//显示正在打印的文件名称
	}
	else if((MySerialLcd.current_page==8)||(MySerialLcd.current_page==17))
	{
		MySerialLcd.Set_Txt(0,(uint16_t)current_temperature[0]);//挤出机当前温度
		MySerialLcd.Set_Txt(1,(uint16_t)target_temperature[0]);//挤出机设定温度
	}
	else if(MySerialLcd.current_page==15) //温度设定页面，需要刷新当前温度和设定温度，还有读出温度设定值
	{
		
		MySerialLcd.Set_Txt(0,(uint16_t)current_temperature[0]);//挤出机当前温度
		MySerialLcd.Set_Txt(2,(uint16_t)current_temperature_bed);//热床当前温度
		
		if(which_value==0)
		{
			if((get_value>=180)&&(get_value<=240))
			{
				setTargetHotend0(get_value);	//设定挤出温度
			}
		}
		else if(which_value==1)
		{
			if((get_value>=40)&&(get_value<=120))
			{
				setTargetBed(get_value);
			}
		}
		else if(which_value==2)
		{
			if(get_value<=255)
			{
				fanSpeed = get_value;
			}
		}

	}
	else if(MySerialLcd.current_page==11) //手动控制页面
	{
		MySerialLcd.Set_Txt(0,current_position[X_AXIS]);
		MySerialLcd.Set_Txt(1,current_position[Y_AXIS]);
		MySerialLcd.Set_Txt(2,current_position[Z_AXIS]);
		
		if(which_value==0)
		{
			if(get_value == 1)
			{
				move_scale = 10.0;
				Set_HValue(0,1);
			}
			else if(get_value == 2)
			{
				move_scale = 1.0;
				Set_HValue(0,2);
			}
			else if(get_value == 3)
			{
				move_scale = 0.1;	
				Set_HValue(0,3);
			}		
		}		
		MySerialLcd.Set_Txt(3,move_scale);	
	}
	else if(MySerialLcd.current_page==13) //进入自动校平台的页面
	{
		static uint8_t platform_time = 0;
		static uint8_t time_max = 0; //最大的循环次数
		if(autocalibration_step == 0x00) //步骤1: XY轴归零，显示清0
		{
			platform_time = 0;
		//if((Stopped == false)&&(READ(PLATFORM_PIN) != PLATFORM_INVERTING))
			{
				current_position[Z_AXIS] += 10.0f;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 5, active_extruder);	
			}
			enquecommand("G28 X0 Y0");    //先让XY归零
			MySerialLcd.Set_Txt(0,"0");   //平台误差
			MySerialLcd.Set_Txt(1,"0");   //第几颗螺丝
			MySerialLcd.Set_Txt(3,"0");   //旋转多少度
			MySerialLcd.Set_Display("p0",0);	
			MySerialLcd.Set_Display("p1",0);
			MySerialLcd.Set_Display("p2",0);
			MySerialLcd.Set_Display("p3",0);
			autocalibration_ok = 0;
			autocalibration_step=0x01;
		}	
		else if(autocalibration_step == 0x01) //步骤2：测量第一个平台螺丝的高度
		{	
			screw1_height = get_platform_position(0,190);
			MySerialLcd.Set_Txt(0,screw1_height);
			autocalibration_step=0x02;
		}
		else if(autocalibration_step == 0x02)//步骤3：测量第二个平台螺丝的高度
		{
			screw2_height = get_platform_position(160,190);
			MySerialLcd.Set_Txt(0,screw2_height);
			autocalibration_step=0x03;
		}
		else if(autocalibration_step == 0x03)//步骤3：测量第三个平台螺丝的高度
		{
			screw3_height = get_platform_position(160,20);
			MySerialLcd.Set_Txt(0,screw3_height);
			autocalibration_step=0x04;
		}	
		else if(autocalibration_step == 0x04)//步骤3：测量第四个平台螺丝的高度
		{
			screw4_height = get_platform_position(0,20);
			MySerialLcd.Set_Txt(0,screw4_height);
			autocalibration_step=0x05;
		}	
		else if(autocalibration_step == 0x05) //步骤4：取第一个平台螺丝和第二个平台螺丝和第三个平台螺丝和第四个平台螺丝的平均值
		{
			average_height = (screw1_height+screw2_height+screw3_height+screw4_height)/4;
			autocalibration_step=0x06;
		}
		else if(autocalibration_step == 0x06) //步骤5：重新测量第一颗螺丝的高度
		{
			screw1_height = get_platform_position(0,170);
			MySerialLcd.Set_Txt(0,screw1_height);
			autocalibration_step=0x07;
		}
		else if(autocalibration_step == 0x07) //步骤6：调整第一个螺丝的高度，当第一螺丝高度和标准值误差在0.1mm以内后，则测量第二个螺丝
		{
			MySerialLcd.Set_Txt(0,screw1_height);
			if(ffabs(screw1_height,average_height)<=0.10) //误差小于等于0.1，调整第二个螺丝
			{
				MySerialLcd.Set_Txt(1,"1"); 
				MySerialLcd.Set_Txt(3,"0");
				MySerialLcd.Set_Display("p0",0); //不显示方向图标
				autocalibration_step=0x08;
			}
			else //误差大于0.1，等待用户调整平台螺丝，点击确定后，重新测量
			{
				platform_time = 0;
				if(screw1_height<average_height)
				{
					float truns = (average_height-screw1_height)/PLATFOMR_SCREW;
					MySerialLcd.Set_Txt(1,"1");
					MySerialLcd.Set_Txt(3,truns); //显示螺丝号
					MySerialLcd.Set_PIC(0,25); //第一颗螺丝CW方向
					MySerialLcd.Set_Display("p0",1);
	
				}
				else
				{
					float truns = (screw1_height-average_height)/PLATFOMR_SCREW;
					MySerialLcd.Set_Txt(1,"1");
					MySerialLcd.Set_Txt(3,truns); //显示螺丝号
					MySerialLcd.Set_PIC(0,24);
					MySerialLcd.Set_Display("p0",1);
				}
				if(autocalibration_ok)
				{
					autocalibration_ok = 0;
					autocalibration_step=0x06;	
				}
			}
		}	
		else if(autocalibration_step == 0x08) //步骤7：重新测量第二螺丝的高度
		{
			screw2_height = get_platform_position(160,170);
			MySerialLcd.Set_Txt(0,screw2_height);
			autocalibration_step=0x09;	
		}
		else if(autocalibration_step == 0x09) //步骤8：调整第二个螺丝的高度，当第二螺丝高度和标准值误差在0.1mm以内后，则测量第三个螺丝
		{
			if(ffabs(screw2_height,average_height)<=0.10) //误差小于等于0.1，调整第二个螺丝
			{
				autocalibration_step=10;
				MySerialLcd.Set_Txt(1,"0");
				MySerialLcd.Set_Txt(3,"0");  	
				MySerialLcd.Set_PIC(1,23);
				MySerialLcd.Set_Display("p1",0); //不显示螺丝2的图标
			}
			else //误差大于0.1，等待用户调整平台螺丝，点击确定后，重新测量
			{
				platform_time = 0;
				if(screw2_height<average_height)
				{
					float truns = (average_height-screw2_height)/PLATFOMR_SCREW;
					MySerialLcd.Set_Txt(1,"2");
					MySerialLcd.Set_Txt(3,truns); //显示螺丝号
					MySerialLcd.Set_PIC(1,23);
					MySerialLcd.Set_Display("p1",1); //显示螺丝2的图标
				}
				else
				{
					float truns = (screw2_height-average_height)/PLATFOMR_SCREW;	
					MySerialLcd.Set_Txt(1,"2");
					MySerialLcd.Set_Txt(3,truns); //显示螺丝号
					MySerialLcd.Set_PIC(1,24);
					MySerialLcd.Set_Display("p1",1);	
				}
				if(autocalibration_ok)
				{
					autocalibration_ok = 0;
					autocalibration_step=0x08;
				}
			}				
		}
		else if(autocalibration_step == 10) //步骤9：测量第三颗螺丝的高度
		{
			screw3_height = get_platform_position(160,20);	
			MySerialLcd.Set_Txt(0,screw3_height);
			autocalibration_step=11;
		}
		else if(autocalibration_step == 11) //步骤10：调整第三个螺丝
		{
			if(ffabs(screw3_height,average_height)<=0.10) //误差小于等于0.1，调整结束
			{
				autocalibration_step=12;
				MySerialLcd.Set_Txt(1,"0");
				MySerialLcd.Set_Txt(3,"0");
				MySerialLcd.Set_PIC(2,21);	
				MySerialLcd.Set_Display("p2",0);	
			}
			else //误差大于0.1，等待用户调整平台螺丝，点击确定后，重新测量
			{
				platform_time = 0;
				if(screw3_height<average_height)
				{
					float truns = (average_height-screw3_height)/PLATFOMR_SCREW;
					MySerialLcd.Set_Txt(1,"3");
					MySerialLcd.Set_Txt(3,truns); //显示螺丝号
					MySerialLcd.Set_PIC(2,21);
					MySerialLcd.Set_Display("p2",1);	
				}
				else
				{	
					float truns = (screw3_height-average_height)/PLATFOMR_SCREW;
					MySerialLcd.Set_Txt(1,"3");
					MySerialLcd.Set_Txt(3,truns); //显示螺丝号
					MySerialLcd.Set_PIC(2,20);
					MySerialLcd.Set_Display("p2",1);	
				}				
				if(autocalibration_ok)
				{
					autocalibration_ok = 0;
					autocalibration_step=10;
				}
			}				
		}
		else if(autocalibration_step == 12) //步骤11：测量第四颗螺丝的高度
		{
			screw4_height = get_platform_position(0,20);
			MySerialLcd.Set_Txt(0,screw4_height);
			autocalibration_step=13;
		}
		else if(autocalibration_step == 13) //步骤12：调整第四个螺丝
		{
			if(ffabs(screw4_height,average_height)<=0.10) //误差小于等于0.1，调整结束
			{
				time_max++;
				if(platform_time==0)
				{
					platform_time = 1;
					autocalibration_step=0x06;
				}
				else
				{
					platform_time = 0;
					autocalibration_step=14;
				}
				if(time_max>=4)
				{
					time_max = 0;
					autocalibration_step=14;
				}
				MySerialLcd.Set_Txt(1,"4");
				MySerialLcd.Set_Txt(3,"0");
				MySerialLcd.Set_PIC(3,21);
				MySerialLcd.Set_Display("p3",0);
			}
			else //误差大于0.1，等待用户调整平台螺丝，点击确定后，重新测量
			{
				platform_time = 0;
				if(screw4_height<average_height)
				{
					float truns = (average_height-screw4_height)/PLATFOMR_SCREW;
					MySerialLcd.Set_Txt(1,"4");
					MySerialLcd.Set_Txt(3,truns); //显示螺丝号
					MySerialLcd.Set_PIC(3,27);
					MySerialLcd.Set_Display("p3",1);
					
				}
				else
				{
					float truns = (screw4_height-average_height)/PLATFOMR_SCREW;
					MySerialLcd.Set_Txt(1,"4");
					MySerialLcd.Set_Txt(3,truns); //显示螺丝号
					MySerialLcd.Set_PIC(3,26);
					MySerialLcd.Set_Display("p3",1);
				}
				if(autocalibration_ok)
				{
					autocalibration_ok = 0;
					autocalibration_step=12;
				}
			}
		}		
		
		else if(autocalibration_step == 14) //步骤13：移动到平台中心点上
		{
			center_height = get_platform_position(100,100);		
			autocalibration_step = 15;		
		}
		else if(autocalibration_step == 15) //步骤14：移动到平台中心点上
		{
		//	if(autocalibration_ok) //平台调平完成，回到主页面
		//	{
				MySerialLcd.Set_Page("platform4",14);
				autocalibration_ok = 0;
				autocalibration_step = 0x00;
	//		}
		}
		else
		{
			
		}
	}
	else
	{
		
	}

}


