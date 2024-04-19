#include "bsp_usart.h"
#include <stdio.h>
#include "../marlin/Marlin.h"

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf  __enable_irq();
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

#endif /* __GNUC__ */
#ifdef __cplusplus
extern "C" {
#endif
///�ض���c�⺯��printf��USARTdebug.h11111

//int fputc(int ch, FILE *f)
//{
//	/* ����һ���ֽ����ݵ�USART1 */
//	USART_SendData(USART1, (uint8_t) ch);
//    USART_SendData(USART2, (uint8_t) ch);
//	/* �ȴ�������� */
//	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//	return (ch);
//}


void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      _write
 *
 * @brief �ض���c�⺯��printf�� _write(int fd, char *buf, int size)
 *
 * @param   *buf - UART send Data.
 *          size - Data length
 *
 * @return  size: Data length
 */
__attribute__((used)) int _write(int fd, char *buf, int size)
{
    int i;

    for(i = 0; i < size; i++)
    {

if (SERIAL_PORT == 2||test_PORT==1)
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
        USART_SendData(USART2, *buf++);
       // USART_SendData(USART1, *buf++);
    }

    return size;
}


///�ض���c�⺯��scanf��USART1
int fgetc(FILE *f)
{
	/* �ȴ�����1�������� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	return (int)USART_ReceiveData(USART1);
}
#ifdef __cplusplus

	}
#endif

ring_buffer rx_buffer  =  { { 0 }, 0, 0 };


void store_char(unsigned char c)
{

  int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;
  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != rx_buffer.tail) { //�������û������д�룬�������
    rx_buffer.buffer[rx_buffer.head] = c;
    rx_buffer.head = i;
  }
}

#ifdef __cplusplus
extern "C" {
//����1�ж��������մ�������
#endif

#if test_PORT == 1
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE)!= RESET)
	{
		unsigned char c = USART1->DATAR;
		store_char(c);
		//USART_SendData(USART1, c);
    //while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){};
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	if(USART_GetITStatus(USART1, USART_IT_TC)!= RESET)
	{

		USART_ClearITPendingBit(USART1, USART_IT_TC);
	}
}
#endif


#if SERIAL_PORT == 2


void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE)!= RESET)
    {
        printf("hello\r\n");

        unsigned char c = USART2->DATAR;
//      USART_SendData(USART2, c);
        store_char(c);
        //USART_SendData(USART1, c);
    //while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){};
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);

    }
    if(USART_GetITStatus(USART2, USART_IT_TC)!= RESET)
    {

        USART_ClearITPendingBit(USART2, USART_IT_TC);
    }

}
#endif

#ifdef __cplusplus
	}
#endif
#if test_PORT == 1
void USART1_IOConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 |RCC_APB2Periph_GPIOA , ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         //USART1 TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);		    //A�˿�

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         //USART1 RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //���ÿ�©����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);



}
#endif

#if SERIAL_PORT == 2

void USART2_IOConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
        RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2 , ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;            //USART1 TX
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //�����������
        GPIO_Init(GPIOA, &GPIO_InitStructure);          //A�˿�

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;           //USART1 RX
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //���ÿ�©����
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

}


#endif
// Public Methods //////////////////////////////////////////////////////////////
void MarlinSerial::begin(uint32_t baud)
{

	GPIO_InitTypeDef GPIO_InitStructure= {0};
	USART_InitTypeDef USART_InitStructure= {0};
	NVIC_InitTypeDef NVIC_InitStructure= {0};
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
#if test_PORT == 1
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 |RCC_APB2Periph_GPIOA , ENABLE);
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = baud;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1

#endif 
#if SERIAL_PORT == 2

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2 , ENABLE);
   //USART1_TX   GPIOA.9
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //�����������
 GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9

 //USART1_RX     GPIOA.10��ʼ��
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA10
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
 GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10

 //Usart1 NVIC ����
 NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      //�����ȼ�3
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQͨ��ʹ��
   NVIC_Init(&NVIC_InitStructure); //����ָ���Ĳ�����ʼ��VIC�Ĵ���

  //USART ��ʼ������

   USART_InitStructure.USART_BaudRate = baud;//���ڲ�����
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
   USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
   USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //�շ�ģʽ

 USART_Init(USART2, &USART_InitStructure); //��ʼ������1
 USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
 USART_Cmd(USART2, ENABLE);
 printf("  uart2_begin\r\n");
#endif




}

void MarlinSerial::end()
{
  USART_Cmd(USART1, DISABLE);
  USART_Cmd(USART2, DISABLE);
	rx_buffer.head=0;
	rx_buffer.tail=0;
}



int MarlinSerial::peek(void)
{
  if (rx_buffer.head == rx_buffer.tail) {
    return -1;
  } else {
    return rx_buffer.buffer[rx_buffer.tail];
  }
}

int MarlinSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (rx_buffer.head == rx_buffer.tail) { //������ڻ�������,�򷵻�-1
    return -1;
  } else {
    unsigned char c = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (unsigned int)(rx_buffer.tail + 1) % RX_BUFFER_SIZE;
    return c;
//    printf("uart2_read\r\n");
  }
}

void MarlinSerial::flush() //��մ��ڻ�����
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
  rx_buffer.head = rx_buffer.tail;
}


//__attribute__((used)) int _write(int fd, char *buf, int size)
//{
//    int i;
//
//    for(i = 0; i < size; i++)
//    {
//#if(DEBUG == DEBUG_UART1)
//        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//        USART_SendData(USART1, *buf++);
//#elif(SERIAL_PORT == 2)
//        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//        USART_SendData(USART2, *buf++);
//#elif(DEBUG == DEBUG_UART3)
//        while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
//        USART_SendData(USART3, *buf++);
//#endif
//    }
//
//    return size;
//}


MarlinSerial MSerial;

