#include "planner.h"
#include "temperature.h"
//#include "ultralcd.h"
#include "ConfigurationStore.h"
#include "language.h"


#include "bsp_pin.h"
#include "Marlin.h"

#define EE_DELAY()    __NOP() //do{for(uint8_t i=0;i<3;i++){__NOP();}}while(0)
#define EE24C02_WRITE_ADDR  0xa0
#define EE24C02_READ_ADDR   0xa1
#define EE24C02_PAGE_BYTES  8  //24C02一页是8个byte,跨页需要重新写地址
#define EE24C02_TOTAL_BYTES  256 

//IIC的start,SCL在高电平是，SDA上的上升沿触发
static uint8_t I2C_Start(void)
{
// 	SET_OUTPUT(EE_SCL_PIN);	
// 	CLR(EE_SCL_PIN); 
// 	EE_DELAY();
// 	
// 	SET_OUTPUT(EE_SDA_PIN);
// 	SET(EE_SDA_PIN);
// 	EE_DELAY();
// 	SET(EE_SCL_PIN);
// 	EE_DELAY();
// 	
// 	CLR(EE_SDA_PIN);
// 	EE_DELAY();
// 	
// 	CLR(EE_SCL_PIN);
// 	EE_DELAY();
	return 0;
}
//IIC的stop,SCL在高电平是，SDA上的下升沿触发
static void I2C_Stop(void)
{
// 	SET_OUTPUT(EE_SCL_PIN);	
// 	CLR(EE_SCL_PIN);
// 	EE_DELAY();
// 	
// 	SET_OUTPUT(EE_SDA_PIN);	
// 	CLR(EE_SDA_PIN);
// 	EE_DELAY();
// 	
// 	SET(EE_SCL_PIN);
// 	EE_DELAY();
// 	
// 	SET(EE_SDA_PIN);
// 	EE_DELAY();
// 	
// 	CLR(EE_SCL_PIN);
// 	EE_DELAY();
// 	CLR(EE_SDA_PIN);
}

//IIC的接收8个字节要发送应答标志,0应答，1不应答
static void I2C_SendACK(uint8_t ack)
{
// 	SET_OUTPUT(EE_SCL_PIN);	
// 	CLR(EE_SCL_PIN);
// 	EE_DELAY();
// 	
// 	SET_OUTPUT(EE_SDA_PIN);	
// 	if(ack)
// 	{
// 		SET(EE_SDA_PIN);
// 	}
// 	else
// 	{
// 		CLR(EE_SDA_PIN);
// 	}
// 	EE_DELAY();
// 	
// 	SET(EE_SCL_PIN);
// 	EE_DELAY();
// 	
// 	CLR(EE_SCL_PIN);
	EE_DELAY();
}
//IIC的发送8个字节后的的应答
static uint8_t I2C_ReceiveACK(void)
{
	uint8_t ack = 0;
// 	SET_OUTPUT(EE_SCL_PIN);	
// 	CLR(EE_SCL_PIN);
// 	EE_DELAY();
// 	
// 	SET_INPUT(EE_SDA_PIN);
// 	EE_DELAY();
// 	
// 	SET(EE_SCL_PIN);
// 	EE_DELAY();
// 	
// 	if(READ(EE_SDA_PIN))
// 	{
// 		ack = 1;
// 	}
// 	else
// 	{
// 		ack = 0;
// 	}
// 	CLR(EE_SCL_PIN);
// 	EE_DELAY();
	
	return ack;
}
//IIC的写一个8字节的数据
static void I2C_WriteByte(uint8_t Data)
{
	uint8_t i;
	
// 	SET_OUTPUT(EE_SCL_PIN);	
// 	CLR(EE_SCL_PIN);
// 	EE_DELAY();
// 	
// 	SET_OUTPUT(EE_SDA_PIN);	
// 	EE_DELAY();
// 	
// 	for(i = 0; i < 8; i++)
// 	{
// 		CLR(EE_SCL_PIN);
// 		EE_DELAY();
// 		
// 		if(Data & 0x80)
// 		{
// 			SET(EE_SDA_PIN);
// 		}
// 		else
// 		{
// 			CLR(EE_SDA_PIN);
// 		} 
// 		Data <<= 1;
// 		EE_DELAY();
// 		
// 		SET(EE_SCL_PIN);
// 		EE_DELAY();
// 	}
// 	
// 	CLR(EE_SCL_PIN);
	EE_DELAY();
}
//IIC接收一个8字节的数据
static uint8_t I2C_ReadByte(void)
{
 	uint8_t i, Dat = 0;

// 	SET_OUTPUT(EE_SCL_PIN);	
// 	CLR(EE_SCL_PIN);
// 	EE_DELAY();
// 	
// 	SET_INPUT(EE_SDA_PIN);	
// 	EE_DELAY();
// 	for(i = 0; i < 8; i++)
// 	{
// 		CLR(EE_SCL_PIN);
// 		EE_DELAY();
// 		
// 		SET(EE_SCL_PIN);
// 		EE_DELAY();
// 		
// 		Dat <<= 1;
// 		if(READ(EE_SDA_PIN))  //读IO数据
// 		{
// 			Dat |= 0x01;
// 		}   
// 	}
// // 	CLR(EE_SCL_PIN);
// 	EE_DELAY();
 	return Dat;
}



//24C02在特定的地址上写入一个字节的数据
uint8_t EE24C02_Write_Byte(uint16_t addr,uint8_t byte)
{
	uint8_t ack = 0;

	I2C_Start();
	EE_DELAY();
	
	I2C_WriteByte(0xa0); //24C02的设备地址
	EE_DELAY();
	
	ack = I2C_ReceiveACK();
	if(ack){return 1;}

	I2C_WriteByte(addr); //24C02的设备地址
	EE_DELAY();
	
	ack = I2C_ReceiveACK();
	if(ack){return 1;}

	I2C_WriteByte(byte); //24C02要写入的数据
	EE_DELAY();
	
	ack = I2C_ReceiveACK();
	if(ack){return 1;}
	 
	I2C_Stop();
	return 0;
}

void delay_ms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  
      while(i--) ;    
   }
}
void delay_us(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=10;  
      while(i--) ;    
   }
}
//24C02字节写构成的跨页连续写
uint8_t EE24C02_Write_Bytes1(uint16_t addr,uint8_t *pbyte,uint16_t length)
{
	uint16_t i;
	if(addr+length > EE24C02_TOTAL_BYTES) return 0;
	for(i=0;i<length;i++)
	{

		EE24C02_Write_Byte(addr+i,*(pbyte+i));
		delay_us(1500); //这个延时不能少,至少1.5ms
	}
	return 1;
}

//24C02写非跨页
uint8_t EE24C02_Write_Page(uint16_t addr,uint8_t *pbyte,uint16_t length)
{
	uint16_t i;
	uint8_t ack = 0;
			
	I2C_Start();
	EE_DELAY();
		
	I2C_WriteByte(0xa0); //24C02的设备地址
	ack = I2C_ReceiveACK();
	if(ack){return 1;}
	EE_DELAY();	

	I2C_WriteByte(addr); //24C02的设备地址
	ack = I2C_ReceiveACK();
	if(ack){return 1;}
	EE_DELAY();
			
	for(i=0;i<length;i++)
	{	
		I2C_WriteByte(*(pbyte+i)); //24C02要写入的数据
		ack = I2C_ReceiveACK();
		if(ack){return 1;}
		EE_DELAY();
		EE_DELAY();
	}	
	I2C_Stop();

	return 1;
}


//24C02跨页连续写
uint8_t EE24C02_Write_Bytes(uint16_t addr,uint8_t *pbyte,uint16_t length)
{
	uint8_t ack = 0;
	uint16_t i;
	
	for(i=0;i<length;i++)
	{
		if(i==0||(addr+i)%EE24C02_PAGE_BYTES==0)
		{			
			I2C_Start();
			EE_DELAY();
			
			I2C_WriteByte(0xa0); //24C02的设备地址
			ack = I2C_ReceiveACK();
			if(ack){return 1;}
			EE_DELAY();
			
			I2C_WriteByte(addr+i); //24C02的设备地址
			ack = I2C_ReceiveACK();
			if(ack){return 1;}
			EE_DELAY();
		}

		I2C_WriteByte(*(pbyte+i)); //24C02要写入的数据
		ack = I2C_ReceiveACK();
		if(ack){return 1;}
		EE_DELAY();
		EE_DELAY();

		if((i==length-1)||((addr+i)%EE24C02_PAGE_BYTES==EE24C02_PAGE_BYTES-1))
		{
			I2C_Stop();
			delay_us(1500);  //至少1.5ms
		}	
	}
	return 0;
}
//在24C02上的特定地址上读一个数据
uint8_t EE24C02_Read_Byte(uint8_t addr)
{
	uint8_t rbyte = 0;
	uint8_t ack = 0;

	I2C_Start();
	EE_DELAY();
		
	I2C_WriteByte(0xa0); //24C02的设备地址
	EE_DELAY();

	ack = I2C_ReceiveACK();
	if(ack){return 1;}
	

	I2C_WriteByte(addr); //24C02的内存地址
	EE_DELAY();
	ack = I2C_ReceiveACK();
	if(ack){return 1;}


	I2C_Start();
	EE_DELAY();
			
	I2C_WriteByte(0xa1); //24C02的设备地址
	EE_DELAY();
	ack = I2C_ReceiveACK();
	if(ack){return 1;}

	
	rbyte = I2C_ReadByte(); //24C02读数据
	I2C_SendACK(1); //单独读不应答,连续读才应答
	
	I2C_Stop();
	EE_DELAY();
	return rbyte;
}

//读可以自由的跨页
uint8_t EE24C02_Read_Bytes(uint16_t addr,uint8_t *pdata,uint8_t length)
{
	uint16_t i;
	uint8_t rbyte = 0;
	uint8_t ack = 0;

	I2C_Start();
	EE_DELAY();
	I2C_WriteByte(0xa0); //24C02的设备地址
	EE_DELAY();
	ack = I2C_ReceiveACK();
	if(ack){return 1;}
	
	I2C_WriteByte(addr); //24C02的内存地址
	EE_DELAY();
	ack = I2C_ReceiveACK();
	if(ack){return 1;}
	
	I2C_Start();
	EE_DELAY();
	I2C_WriteByte(0xa1); //24C02的设备地址
	EE_DELAY();
	ack = I2C_ReceiveACK();
	if(ack){return 1;}
			
	for(i=0;i<length;i++)
	{				
		*(pdata+i) = I2C_ReadByte(); //24C02读数据
		I2C_SendACK(0);
		EE_DELAY();
	}
	I2C_Stop();
	return rbyte;
}


//从EE24C02里连续读
uint8_t EE24C02_Read_Bytes1(uint16_t addr,uint8_t *pdata,uint8_t length)
{
	uint16_t i;
	for(i=0;i<length;i++)
	{
		*(pdata+i) = EE24C02_Read_Byte(addr+i);
	}
	return 1;
}



//======================================================================================
#define MSG_ERR_EEPROM_WRITE                "Error writing to EEPROM!"


void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size) {
	uint8_t c;
	while (size--) {
		EE24C02_Write_Byte(pos, *value);
		delay_us(1500);  //至少1.5ms
		//EE24C02_Write_Byte(uint16_t addr,uint8_t byte)
		//eeprom_write_byte((unsigned char*)pos, *value);
		//c = eeprom_read_byte((unsigned char*)pos);
		c = EE24C02_Read_Byte(pos);
		if (c != *value) {
			printf(MSG_START);
			printf(MSG_ERR_EEPROM_WRITE);
		}
		pos++;
		value++;
	};
}
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size) {
	do {
		*value = EE24C02_Read_Byte(pos);
		pos++;
		value++;
	} while (--size);
}

#define EEPROM_OFFSET 20


#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))

// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.
#define EEPROM_VERSION "V07"

#ifdef EEPROM_SETTINGS
void Config_StoreSettings() 
{
  char ver[4]= EEPROM_VERSION;
  int i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver); // invalidate data first 
  EEPROM_WRITE_VAR(i,axis_steps_per_unit);  
  EEPROM_WRITE_VAR(i,max_feedrate);  
  EEPROM_WRITE_VAR(i,max_acceleration_units_per_sq_second);
  EEPROM_WRITE_VAR(i,acceleration);
  EEPROM_WRITE_VAR(i,retract_acceleration);
  EEPROM_WRITE_VAR(i,minimumfeedrate);
  EEPROM_WRITE_VAR(i,mintravelfeedrate);
  EEPROM_WRITE_VAR(i,minsegmenttime);
  EEPROM_WRITE_VAR(i,max_xy_jerk);
  EEPROM_WRITE_VAR(i,max_z_jerk);
  EEPROM_WRITE_VAR(i,max_e_jerk);
  EEPROM_WRITE_VAR(i,add_homeing);
  #ifndef ULTIPANEL
  int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP, plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP, plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
  int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP, absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP, absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
  #endif
  EEPROM_WRITE_VAR(i,plaPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,plaPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,plaPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,absPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,absPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,absPreheatFanSpeed);
  #ifdef PIDTEMP
    EEPROM_WRITE_VAR(i,Kp);
    EEPROM_WRITE_VAR(i,Ki);
    EEPROM_WRITE_VAR(i,Kd);
  #else
		float dummy = 3000.0f;
    EEPROM_WRITE_VAR(i,dummy);
		dummy = 0.0f;
    EEPROM_WRITE_VAR(i,dummy);
    EEPROM_WRITE_VAR(i,dummy);
  #endif
  char ver2[4]=EEPROM_VERSION;
  i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver2); // validate data
  printf(MSG_START);
  printf("Settings Stored\n\r");
}
#endif //EEPROM_SETTINGS


#ifdef EEPROM_CHITCHAT
void Config_PrintSettings()
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
	printf(MSG_START);
  printf("Steps per unit:");
  printf(MSG_START);
  printf("  M92 X%f",axis_steps_per_unit[0]);
  printf(" Y%f",axis_steps_per_unit[1]);
  printf(" Z%f",axis_steps_per_unit[2]);
  printf(" E%f",axis_steps_per_unit[3]);
  printf("\r\n");
      
  printf(MSG_START);
  printf("Maximum feedrates (mm/s):");
  printf(MSG_START);
  printf("  M203 X%f",max_feedrate[0]);
  printf(" Y%f",max_feedrate[1] ); 
  printf(" Z%f", max_feedrate[2] ); 
  printf(" E%f", max_feedrate[3]);
  printf("\r\n");

  printf(MSG_START);
  printf("Maximum Acceleration (mm/s2):");
  printf(MSG_START);
  printf("  M201 X%ld" ,max_acceleration_units_per_sq_second[0] ); 
  printf(" Y%ld" , max_acceleration_units_per_sq_second[1] ); 
  printf(" Z%ld" ,max_acceleration_units_per_sq_second[2] );
  printf(" E%ld" ,max_acceleration_units_per_sq_second[3]);
  printf("\r\n");
  printf(MSG_START);
  printf("Acceleration: S=acceleration, T=retract acceleration");
  printf(MSG_START);
  printf("  M204 S%f",acceleration ); 
  printf(" T%f" ,retract_acceleration);
  printf("\r\n");

  printf(MSG_START);
  printf("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
  printf(MSG_START);
  printf("  M205 S%f",minimumfeedrate ); 
  printf(" T%f" ,mintravelfeedrate ); 
  printf(" B%f" ,minsegmenttime ); 
  printf(" X%f" ,max_xy_jerk ); 
  printf(" Z%f" ,max_z_jerk);
  printf(" E%f" ,max_e_jerk);
  printf("\r\n");

  printf(MSG_START);
  printf("Home offset (mm):");
  printf(MSG_START);
  printf("  M206 X%f",add_homeing[0] );
  printf(" Y%f" ,add_homeing[1] );
  printf(" Z%f" ,add_homeing[2] );
  printf("\r\n");
#ifdef PIDTEMP
  printf(MSG_START);
  printf("PID settings:");
  printf(MSG_START);
  printf("   M301 P%f",Kp); 
  printf(" I%f" ,unscalePID_i(Ki)); 
  printf(" D%f" ,unscalePID_d(Kd));
  printf("\r\n");
#endif
} 
#endif


#ifdef EEPROM_SETTINGS
void Config_RetrieveSettings()
{
    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    EEPROM_READ_VAR(i,stored_ver); //read stored version
    //  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
    if (strncmp(ver,stored_ver,3) == 0)
    {
        // version number match
        EEPROM_READ_VAR(i,axis_steps_per_unit);  
        EEPROM_READ_VAR(i,max_feedrate);  
        EEPROM_READ_VAR(i,max_acceleration_units_per_sq_second);
        
        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
				reset_acceleration_rates();
        
        EEPROM_READ_VAR(i,acceleration);
        EEPROM_READ_VAR(i,retract_acceleration);
        EEPROM_READ_VAR(i,minimumfeedrate);
        EEPROM_READ_VAR(i,mintravelfeedrate);
        EEPROM_READ_VAR(i,minsegmenttime);
        EEPROM_READ_VAR(i,max_xy_jerk);
        EEPROM_READ_VAR(i,max_z_jerk);
        EEPROM_READ_VAR(i,max_e_jerk);
        EEPROM_READ_VAR(i,add_homeing);
        #ifndef ULTIPANEL
        int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
        int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
        #endif
        EEPROM_READ_VAR(i,plaPreheatHotendTemp);
        EEPROM_READ_VAR(i,plaPreheatHPBTemp);
        EEPROM_READ_VAR(i,plaPreheatFanSpeed);
        EEPROM_READ_VAR(i,absPreheatHotendTemp);
        EEPROM_READ_VAR(i,absPreheatHPBTemp);
        EEPROM_READ_VAR(i,absPreheatFanSpeed);
        #ifndef PIDTEMP
        float Kp,Ki,Kd;
        #endif
        // do not need to scale PID values as the values in EEPROM are already scaled		
        EEPROM_READ_VAR(i,Kp);
        EEPROM_READ_VAR(i,Ki);
        EEPROM_READ_VAR(i,Kd);

		// Call updatePID (similar to when we have processed M301)
		updatePID();
        printf(MSG_START);
        printf("Stored settings retrieved\n\r");
    }
    else
    {
        Config_ResetDefault();
    }
    Config_PrintSettings();
}
#endif

void Config_ResetDefault()
{
    float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[]=DEFAULT_MAX_FEEDRATE;
    long tmp3[]=DEFAULT_MAX_ACCELERATION;
    for (short i=0;i<4;i++) 
    {
        axis_steps_per_unit[i]=tmp1[i];  
        max_feedrate[i]=tmp2[i];  
        max_acceleration_units_per_sq_second[i]=tmp3[i];
    }
    
    // steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();
    
    acceleration=DEFAULT_ACCELERATION;
    retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
    minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
    minsegmenttime=DEFAULT_MINSEGMENTTIME;       
    mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
    max_xy_jerk=DEFAULT_XYJERK;
    max_z_jerk=DEFAULT_ZJERK;
    max_e_jerk=DEFAULT_EJERK;
    add_homeing[0] = add_homeing[1] = add_homeing[2] = 0;
#ifdef ULTIPANEL
    plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
    plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
    plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
    absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
    absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
#endif
#ifdef PIDTEMP
    Kp = DEFAULT_Kp;
    Ki = scalePID_i(DEFAULT_Ki);
    Kd = scalePID_d(DEFAULT_Kd);
    
    // call updatePID (similar to when we have processed M301)
    updatePID();
    
#ifdef PID_ADD_EXTRUSION_RATE
    Kc = DEFAULT_Kc;
#endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP

printf(MSG_START);
printf("Hardcoded Default Settings Loaded\n\r");

}

