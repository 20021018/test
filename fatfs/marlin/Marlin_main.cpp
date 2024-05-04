/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */
#include "Marlin.h"
#include "bsp_pin.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "bsp_watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "cardreader.h"
//#include "seriallcd.h"

#include "mmc_sd.h"
#include "bsp_usart.h"

#include"debug.h"


#define VERSION_STRING  "SardineBoard V1.0"

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for extruder current temp to reach target temp.
// M114 - Display current position

//Custom M Codes
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84

// M20  - List SD card //完成
// M21  - Init SD card //完成
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial

// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M114 - Output current position to serial port
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M200 - Set filament diameter
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homeing offset
// M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M240 - Trigger a camera to take a photograph
// M280 - set servo position absolute. P: servo index, S: angle or microseconds
// M300 - Play beepsound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from eeprom)
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M928 - Start SD logging (M928 filename.g) - ended by M29
// M999 - Restart after being stopped by error

//Stepper Movement Variables
void All_Axis_Go_Home(void);


//===========================================================================
//=============================imported variables============================
//===========================================================================
//===========================================================================
//=============================public variables=============================
//===========================================================================

#ifdef SDSUPPORT
CardReader card;
#endif

float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float destination[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0};
float add_homeing[3]={ 0, 0, 0}; 
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
// Extruder offset, only in XY plane
#if EXTRUDERS > 1
float extruder_offset[2][EXTRUDERS] = {
#if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y)
  EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y
#endif
};
#endif
uint8_t active_extruder = 0;
int fanSpeed=0;
#ifdef BARICUDA
int ValvePressure=0;
int EtoPPressure=0;
#endif

#ifdef FWRETRACT
  bool autoretract_enabled=true;
  bool retracted=false;
  float retract_length=3, retract_feedrate=17*60, retract_zlift=0.8;
  float retract_recover_length=0, retract_recover_feedrate=8*60;
#endif

/******************限位开关的接法,false是常闭,true常开,公共端C接信号,常开(NO)或者常闭(NC)接GND***********************/
// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.（解决多次重复定义，在Configuration.h中声明）
const bool X_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Y_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Z_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool LEVEL_INVERTING  = false;

//===========================================================================
//=============================private variables=============================
//===========================================================================

const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

volatile bool relative_mode = false;  //Determines Absolute or Relative Coordinates



static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static bool comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

//const int sensitive_pins[] = {1,2,3};//SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables


static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;

static uint8_t tmp_extruder;


bool Stopped=false;

#if NUM_SERVOS > 0
  Servo servos[NUM_SERVOS];
#endif

volatile unsigned long previous_millis_cmd = 0;

volatile uint16_t print_total_time = 0;	      
volatile uint16_t print_total_layer = 0;      
volatile uint16_t print_current_layer = 0;    
volatile uint16_t print_start_layer = 0;	  


//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

void get_arc_coordinates();
bool setTargetedHotend(int code);

void serial_echopair_P(const char *s_P, float v)
    { printf("%s",s_P); printf("%f",v); }
void serial_echopair_P(const char *s_P, double v)
    { printf("%s",s_P); printf("%f",v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { printf("%s",s_P); printf("%f",v); }

#if __cplusplus
extern "C"{
#endif
//  extern unsigned int __Vectors_End;
//  extern unsigned int __heap_start;
//  extern void *__brkval;
//计算剩余内存空间
int freeMemory() 
{
//     int free_memory;

//     if((int)__brkval == 0)
//       free_memory = ((int)&free_memory) - ((int)&__bss_end);
//     else
//       free_memory = ((int)&free_memory) - ((int)__brkval);

    return 0;
}
#if __cplusplus
}
#endif
//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
    strcpy(&(cmdbuffer[bufindw][0]),cmd);
    printf(MSG_START);
    printf("enqueing \"");
    printf("%s",cmdbuffer[bufindw]);
    printf("\"\n\r");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void enquecommand_P(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
   //// strcpy_P(&(cmdbuffer[bufindw][0]),cmd);
		
		strcpy(&(cmdbuffer[bufindw][0]),cmd);
    printf(MSG_START);
    printf("enqueing \"");
    printf("%s",cmdbuffer[bufindw]);
    printf("\"\n\r");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void setup_killpin()
{
  #if defined(KILL_PIN)
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN,HIGH);
  #endif
}

void setup_photpin()
{
  #if defined(PHOTOGRAPH_PIN)
    SET_OUTPUT(PHOTOGRAPH_PIN);
    WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_powerhold()
{
  #if defined(SUICIDE_PIN)
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if defined(PS_ON_PIN)
    SET_OUTPUT(PS_ON_PIN);
    WRITE(PS_ON_PIN, PS_ON_AWAKE);
  #endif
}

//复位自己
void suicide()
{
  #if defined(SUICIDE_PIN)
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, LOW);
  #endif
	printf("System will Reset itself\n\r");
	//__set_FAULTMASK(1);
	__disable_irq();//关闭全局中断
	NVIC_SystemReset();
}

void servo_init()
{
  #if (NUM_SERVOS >= 1) && defined(SERVO0_PIN) && (SERVO0_PIN > -1)
    servos[0].attach(SERVO0_PIN);
  #endif
  #if (NUM_SERVOS >= 2) && defined(SERVO1_PIN) && (SERVO1_PIN > -1)
    servos[1].attach(SERVO1_PIN);
  #endif
  #if (NUM_SERVOS >= 3) && defined(SERVO2_PIN) && (SERVO2_PIN > -1)
    servos[2].attach(SERVO2_PIN);
  #endif
  #if (NUM_SERVOS >= 4) && defined(SERVO3_PIN) && (SERVO3_PIN > -1)
    servos[3].attach(SERVO3_PIN);
  #endif
  #if (NUM_SERVOS >= 5)
    #error "TODO: enter initalisation code for more servos"
  #endif
}


void delay_mms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  
      while(i--) ;    
   }
}

void setup()
{
   // SystemCoreClockUpdate();
    //

	SET_OUTPUT(BUZ_PIN);
	WRITE(BUZ_PIN,HIGH);
//
//	setup_killpin();
//    setup_powerhold();
	//MySerialLcd.begin(9600);//暂时把串口屏初始化搁置
    MYSERIAL.begin(115200);
 	SET_OUTPUT(LED_PIN);
// 	WRITE(LED_PIN,HIGH);

	#ifdef SDSUPPORT
	SPI2_Init(); //sd卡用的SPI2初始化
	#endif
	printf(MSG_MARLIN);
	printf(VERSION_STRING);

  #ifdef STRING_VERSION_CONFIG_H
    #ifdef STRING_CONFIG_H_AUTHOR
      printf(MSG_START);
      printf(MSG_CONFIGURATION_VER);
      printf(STRING_VERSION_CONFIG_H);
      printf(MSG_AUTHOR);
      printf(STRING_CONFIG_H_AUTHOR);
      printf("Compiled: ");
      printf(__DATE__);
    #endif
  #endif
	printf(MSG_START);
  printf(MSG_FREE_MEMORY);
  printf("%d",freeMemory());
  printf(MSG_PLANNER_BUFFER_BYTES);
  printf("%d\n",(int)(sizeof(block_t)*BLOCK_BUFFER_SIZE));

  for(int8_t i = 0; i < BUFSIZE; i++) //用来标记命令缓冲区的数据是否需要保存在SD卡里
  {
    fromsd[i] = false;
  }
//  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
// // Config_RetrieveSettings();  //从eeprom和configure.h中载入设定的参数
	Config_ResetDefault();
    tp_init();    //ADC初始化,并打开定时器3,产生1ms的中断用来读取ADC值和输出软PWM
    plan_init();  //运动缓存区数据清空
////	#ifdef USE_WATCHDOG
//////  watchdog_init(); //看门狗初始化
////	#endif
    st_init();    //电机IO初始化，定时器2中断初始化，限位开关初始化
//  //setup_photpin(); //触发外部相机拍照
//  //servo_init();    //伺服舵机
    SystemTick_Init();  //系统时钟初始化,产生1ms的中断,用来计时   有问题的函数
//
   #if defined(CONTROLLERFAN_PIN)
//    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
   #endif
//
////	MySerialLcd.Set_Page("main",1);	//串口屏进入主页面
 	WRITE(BUZ_PIN,LOW);

}

void delay_nms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  
      while(i--) ;    
   }
}
int main()
{		
	setup(); //初始化
//	SET_OUTPUT(HEATER_BED_PIN);


	while(1)
	{

//	      printf("T:");//for test
//	      printf("%.1f ",degHotend(tmp_extruder));//for test
//
		if(buflen < (BUFSIZE-1))
	 	get_command(); //把串口缓冲区的数据解析出来保存到GM码命令缓冲区中
		#ifdef SDSUPPORT
		card.checkautostart(false); //初始化SD卡,以及M23,	M24支持G码写入SD卡
		#endif
		if(buflen)
		{
			#ifdef SDSUPPORT
			card.savefile(cmdbuffer[bufindr]); //命令数据是否写入SD卡
			#endif
			process_commands(); //命令的执行
			buflen = (buflen-1);
			bufindr = (bufindr + 1)%BUFSIZE;
		}
		//check heater every n milliseconds
		manage_heater(); 			//温度转换,PID调温,软PWM输出
		manage_inactivity(); //加热故障管理
		checkHitEndstops();	 //限位开关触发后发送信息
//		MySerialLcd.LCD_Run(); //串行触摸屏的数据处理
	//	IWDG_Feed(); //看门狗喂狗

	}
	return 0;
}

uint16_t get_explaination_data(char *pbuf,char c)
{
	char i;
	uint16_t d=0;
	for(i=0;i<MAX_CMD_SIZE;i++) 
	{
		if(*(pbuf+i)==c)
		{
			break;
		}
	}
	for(;i<MAX_CMD_SIZE;i++) 
	{
		if(*(pbuf+i)>='0' && *(pbuf+i)<='9')
		{
			break;
		}
	}
	for(;i<MAX_CMD_SIZE;i++) 
	{
		if(*(pbuf+i)>='0' && *(pbuf+i)<='9')
		{
			d=10*d;
			d += *(pbuf+i)-'0';
		}
		else
		{
			break;
		}
	}	
	return d;
}


void get_explaination(char *pbuf)
{
	char c = *(pbuf+1);
	switch(c)
	{
		case 'L':
			if(strstr(pbuf,"Layer count:")!=NULL)
			{
				print_total_layer = get_explaination_data(pbuf,':');
			}
			if(strstr(pbuf,"LAYER:")!=NULL)	
			{
				print_current_layer=get_explaination_data(pbuf,':');
			}
		break;
		case 'P':
			if(strstr(pbuf,"Print time:")!=NULL)
			{
				print_total_time = get_explaination_data(pbuf,'s'); 
				if(print_total_time)	
				{
					print_total_time += 60*get_explaination_data(pbuf,':');
				}
				else
				{
					print_total_time = get_explaination_data(pbuf,':');
				}
			}
		break;
		default:
		break;
	}
}

//在串口缓存区中取出GCODE的命令
void get_command()
{	
	//如果串口中数据,并且命令缓存区中数据没有满
//printf("mmm:%s\n",MYSERIAL.available());
  while( MYSERIAL.available() > 0  && buflen < BUFSIZE)
//    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
  {
    serial_char = MYSERIAL.read(); //串口缓存区中取出字符
   // printf("串口缓冲区：%s\n",serial_char);
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1) ) //字符是换行或注释或长度过大
    {
      if(!serial_count) { //if empty line
        comment_mode = false; //for new command
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if(!comment_mode){ //如果不是注释
        comment_mode = false; //for new command
        fromsd[bufindw] = false; //改行命令是否要保存到SD卡中
        if(strchr(cmdbuffer[bufindw], 'N') != NULL) 	//cmdbuffer中有'N'
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N'); //或者'\N'的地址
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10)); //获取N后面的行号
          if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {
            printf(MSG_ERR);
            printf(MSG_ERR_LINE_NO);
            printf("%ld",gcode_LastN);
            printf("%ld",gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
          if(strchr(cmdbuffer[bufindw], '*') != NULL) //如果没有'*'
          {
            char checksum = 0;
            char count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');

            if((int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) { 
              printf(MSG_ERR);
              printf(MSG_ERR_CHECKSUM_MISMATCH);
              printf("%ld\n\r",gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }
            //if no errors, continue parsinghello
          }
          else
          {
            printf(MSG_ERR);
            printf(MSG_ERR_NO_CHECKSUM);
            printf("%ld",gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if((strchr(cmdbuffer[bufindw], '*') != NULL))
          {
            printf(MSG_ERR);
            printf(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            printf("%ld",gcode_LastN);
            serial_count = 0;
            return;
          }
        }
        if((strchr(cmdbuffer[bufindw], 'G') != NULL)){  //判断命令行是否是G码
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
            if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
          #ifdef SDSUPPORT
              if(card.saving)
                break;
          #endif //SDSUPPORT
              printf(MSG_OK);
            }
            else {
              printf(MSG_ERR_STOPPED);
            ////  LCD_MESSAGEPGM(MSG_STOPPED);
            }
            break;
          default:
            break;
          }

        }
        bufindw = (bufindw + 1)%BUFSIZE;  //循环读命令条数中的命令
        buflen += 1; //命令条数累加
      }
      serial_count = 0; //clear buffer //命令长度0开始保存
    }
    else
    {
      if(serial_char == ';') comment_mode = true; //GCODE里注释的信息
      if(!comment_mode)  //不是注释命令，进行保存
			{
				cmdbuffer[bufindw][serial_count++] = serial_char; //把串口缓存区的命令保存到命令缓存区中
			}
    }
  }

  #ifdef SDSUPPORT //从SD卡写入数据
//  printf("mmm:%s\n",MYSERIAL.available());
  char explain_buf[MAX_CMD_SIZE]={0};	//SD卡数据缓存区
  char explation_buf_len = 0;
  if(!card.sdprinting || serial_count!=0){ 
    return;
  }
  while(!card.eof()  && buflen < BUFSIZE) {  //数据没有读完
    int16_t n=card.get(); //读出SD中的GCODE数据
    serial_char = (char)n;
    if(serial_char == '\n' || //SD卡中的命令结束
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1)||n==-1)
    { 

				if(card.eof()){	 //如果SD卡数据读完
        printf(MSG_FILE_PRINTED);
				printf("sd print finish!!!");
				stoptime=millis();
        char time[30];
        unsigned long t=(stoptime-starttime)/1000;
        uint16_t hours, minutes;
        minutes=(t/60)%60;
        hours=t/60/60;
				sprintf(time, PSTR("%i hours %i minutes"),hours, minutes);
        printf(MSG_START);
        printf("%d",time);
				card.printingHasFinished();
        card.checkautostart(true);
      }
			if(comment_mode==true) //GCODE中的数据注释信息
			{
				explation_buf_len = 0;
				get_explaination(explain_buf);  //解析GCODE中注释信息
				for(char i=0;i<MAX_CMD_SIZE;i++)explain_buf[i]=0;
			}
      if(!serial_count)
      {
        comment_mode = false; //for new command
        return; //if empty line
      }
				cmdbuffer[bufindw][serial_count] = 0; //terminate string
        fromsd[bufindw] = true;
        buflen += 1;
        bufindw = (bufindw + 1)%BUFSIZE;
				comment_mode = false;		//for new command
				serial_count = 0;			//clear buffer
    }
    else //保存SD数据到cmdbuffer中
    {
			if(serial_char == ';') comment_mode = true;
				if(!comment_mode) 
			{
				cmdbuffer[bufindw][serial_count++] = serial_char; //保存SD卡中的命令道缓存区中
			  MYSERIAL.write(serial_char);
			}
			else 
			{  
				explain_buf[explation_buf_len++] = serial_char;  //获取注释信息
			}
    }
  }
  #endif //SDSUPPORT
}

float code_value() //将G或者M后面的字符串转换成浮点数
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()//将G或者M后面的字符串转换成长整数
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)//在G或者M后面寻找某一个字符
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

bool an_seen(char code) //同code_seen
{
	strchr_pointer = strchr(cmdbuffer[bufindr], code);
	return (strchr_pointer != NULL);  //Return True if a character was found
}


float base_min_pos[] = {X_MIN_POS, Y_MIN_POS, Z_MIN_POS};
float base_max_pos[] = {X_MAX_POS, Y_MAX_POS, Z_MAX_POS};
float base_home_pos[]= {X_HOME_POS,Y_HOME_POS,Z_HOME_POS};
float max_length[] =   {X_MAX_LENGTH,Y_MAX_LENGTH,Z_MAX_LENGTH};
float home_retract_mm[] = {X_HOME_RETRACT_MM,Y_HOME_RETRACT_MM,Z_HOME_RETRACT_MM};
signed char home_dir[] = {X_HOME_DIR,Y_HOME_DIR,Z_HOME_DIR};


static void axis_is_at_home(int axis) {
  current_position[axis] = base_home_pos[axis] + add_homeing[axis];
  min_pos[axis] =          base_min_pos[axis] + add_homeing[axis];
  max_pos[axis] =          base_max_pos[axis] + add_homeing[axis];
}



static void homeaxis(int axis) 
{
#define HOMEAXIS_DO(LETTER) ((LETTER##_HOME_DIR==-1) || (LETTER##_HOME_DIR==1))

  if (axis==X_AXIS ? HOMEAXIS_DO(X) : axis==Y_AXIS ? HOMEAXIS_DO(Y) :axis==Z_AXIS ? HOMEAXIS_DO(Z) : 0) 
	{
		current_position[axis] = 0; 
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]); 
		destination[axis] = 1.5 * max_length[axis] * home_dir[axis]; 
		feedrate = homing_feedrate[axis]; 
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
		destination[axis]=0;
		st_synchronize(); 

		current_position[axis] = 0; 
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		destination[axis] = -home_retract_mm[axis] * home_dir[axis]; 
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
		destination[axis]=0;
		st_synchronize();

		destination[axis] = 2*home_retract_mm[axis] * home_dir[axis];  
		feedrate = homing_feedrate[axis]/2 ;
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
		destination[axis]=0;
		st_synchronize(); 

		axis_is_at_home(axis); 
		destination[axis] = current_position[axis]; 
		feedrate = 0.0;
		endstops_hit_on_purpose(); 
  }
}


#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

 
void All_Axis_Go_Home(void)
{
    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;
	  
    previous_millis_cmd = millis();
	  
	  st_synchronize();

    enable_endstops(true);  
	  //check_platform = true;

			for(int8_t i=0; i < NUM_AXIS; i++) 
			{
        destination[i] = current_position[i];
      }
      feedrate = 0.0;
      home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

      #if Z_HOME_DIR > 0                      // If homing away from BED do Z first
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        HOMEAXIS(Z);
      }
      #endif

      #ifdef QUICK_HOME
      if((home_all_axis)||( code_seen(axis_codes[X_AXIS]) && code_seen(axis_codes[Y_AXIS])) )  //first diagonal move
      {
        current_position[X_AXIS] = 0;current_position[Y_AXIS] = 0;

        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        destination[X_AXIS] = 1.5 * X_MAX_LENGTH * X_HOME_DIR;destination[Y_AXIS] = 1.5 * Y_MAX_LENGTH * Y_HOME_DIR;
        feedrate = homing_feedrate[X_AXIS];
        if(homing_feedrate[Y_AXIS]<feedrate)
          feedrate =homing_feedrate[Y_AXIS];
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
        st_synchronize();

        axis_is_at_home(X_AXIS);
        axis_is_at_home(Y_AXIS);
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        destination[X_AXIS] = current_position[X_AXIS];
        destination[Y_AXIS] = current_position[Y_AXIS];
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose();
      }
      #endif

      if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
      {
        HOMEAXIS(X);
      }

      if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
        HOMEAXIS(Y);
      }

      #if Z_HOME_DIR < 0                      // If homing towards BED do Z last
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        HOMEAXIS(Z);
      }
      #endif

      if(code_seen(axis_codes[X_AXIS])) 
      {
        if(code_value_long() != 0) {
          current_position[X_AXIS]=code_value()+add_homeing[0];
        }
      }

      if(code_seen(axis_codes[Y_AXIS])) {
        if(code_value_long() != 0) {
          current_position[Y_AXIS]=code_value()+add_homeing[1];
        }
      }

    if(code_seen(axis_codes[Z_AXIS])) {		  
        if(code_value_long() != 0) {
          current_position[Z_AXIS]=code_value()+add_homeing[2];
        }
      }

      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif

      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_cmd = millis();
      endstops_hit_on_purpose();
	//		check_platform = false;
 }

 //Gcode的命令的执行
void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;

  if(code_seen('G'))
  {
    switch((int)code_value())
    {
    case 0: // G0 -> G1
	   
    case 1: // G1     
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E F
        prepare_move();
        moving_flag=true;//上位机  TIM2_I
        //ClearToSend();
        return;
      }
      //break;
    case 2: // G2  - CW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(true);
        return;
      }
    case 3: // G3  - CCW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(false);
        return;
      }
    case 4: // G4 dwell
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      st_synchronize();
      codenum += millis();  // keep track of when we started waiting
      previous_millis_cmd = millis();
      while(millis()  < codenum ){
				IWDG_Feed();
        manage_heater();
        manage_inactivity();
      //  lcd_update();
      }
      break;
      #ifdef FWRETRACT
      case 10: // G10 retract
      if(!retracted)
      {
        destination[X_AXIS]=current_position[X_AXIS];
        destination[Y_AXIS]=current_position[Y_AXIS];
        destination[Z_AXIS]=current_position[Z_AXIS];
        current_position[Z_AXIS]+=-retract_zlift;
        destination[E_AXIS]=current_position[E_AXIS]-retract_length;
        feedrate=retract_feedrate;
        retracted=true;
        prepare_move();
      }

      break;
      case 11: // G10 retract_recover
      if(!retracted)
      {
        destination[X_AXIS]=current_position[X_AXIS];
        destination[Y_AXIS]=current_position[Y_AXIS];
        destination[Z_AXIS]=current_position[Z_AXIS];

        current_position[Z_AXIS]+=retract_zlift;
        current_position[E_AXIS]+=-retract_recover_length;
        feedrate=retract_recover_feedrate;
        retracted=false;
        prepare_move();
      }
      break;
      #endif //FWRETRACT
    case 28: //G28 Home all Axis one at a time
      All_Axis_Go_Home();
      moving_flag=true;//上位机
      break;
    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
      break;
    case 92: // G92
      if(!code_seen(axis_codes[E_AXIS]))
        st_synchronize();
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
           if(i == E_AXIS) {
             current_position[i] = code_value();
             plan_set_e_position(current_position[E_AXIS]);
           }
           else {
					current_position[i] = code_value()+add_homeing[i];
					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
				}
        }
      }
      break;
    }
  }
  else if(code_seen('M'))
  {
    switch( (int)code_value() )
    {
#ifdef ULTIPANEL
    case 0: // M0 - Unconditional stop - Wait for user button press on LCD
    case 1: // M1 - Conditional stop - Wait for user button press on LCD
    {
      LCD_MESSAGEPGM(MSG_USERWAIT);
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      st_synchronize();
      previous_millis_cmd = millis();
      if (codenum > 0){
        codenum += millis();  // keep track of when we started waiting
        while(millis()  < codenum && !lcd_clicked()){
					IWDG_Feed();
          manage_heater();
          manage_inactivity();
        }
      }else{
        while(!lcd_clicked()){
					IWDG_Feed();
          manage_heater();
          manage_inactivity();
        }
      }
    }
    break;
#endif
    case 17:
        enable_x();
        enable_y();
        enable_z();
        enable_e0();
        enable_e1();
        enable_e2();
      break;

#ifdef SDSUPPORT
    case 20: // M20 - list SD card
      printf(MSG_BEGIN_FILE_LIST);
      card.ls();
      printf(MSG_END_FILE_LIST);
	  break;
    case 21: // M21 - init SD card
      card.initsd();
      break;
    case 22: //M22 - release SD card
      card.release();
      break;
    case 23: //M23 - Select file
   //   starpos = (strchr(strchr_pointer + 4,'*'));
    //  if(starpos!=NULL)
     //   *(starpos-1)='\0';
     // card.openFile(strchr_pointer + 4,true);   
			//	uint32_t a = millis();
				if(strstr(cmdbuffer[bufindr],".gcode")!=NULL)
				{
					char filename[64];
					strcpy(filename,&cmdbuffer[bufindr][4]);
					card.openFile(filename,1);
					printf("%s",cmdbuffer[bufindr]);
				}
      break;
    case 24: //M24 - Start SD print
      card.startFileprint();
      starttime=millis();
	  break;
    case 25: //M25 - Pause SD print
      card.pauseSDPrint();
      break;
    case 26: //M26 - Set SD index
      if(card.cardOK && code_seen('S')) {
        card.setIndex(code_value_long());
      }
      break;
    case 27: //M27 - Get SD status
      card.getStatus();
      break;
    case 28: //M28 - Start SD write
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos-1) = '\0';
      }
      card.openFile(strchr_pointer+4,false);
      break;
    case 29: //M29 - Stop SD write
      //processed in write to file routine above
      card.saving = false;
      break;
    case 30: //M30 <filename> Delete File
      if (card.cardOK){
        card.closefile();
        starpos = (strchr(strchr_pointer + 4,'*'));
        if(starpos != NULL){
          char* npos = strchr(cmdbuffer[bufindr], 'N');
          strchr_pointer = strchr(npos,' ') + 1;
          *(starpos-1) = '\0';
        }
        card.removeFile(strchr_pointer + 4);
      }
      break;
    case 928: //M928 - Start SD write
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos-1) = '\0';
      }
      card.openLogFile(strchr_pointer+5);
      break;

#endif //SDSUPPORT

    case 31: //M31 take time since the start of the SD print or an M109 command
      {
				stoptime=millis();
				char time[30];
				unsigned long t=(stoptime-starttime)/1000;
				int sec,min;
				min=t/60;
				sec=t%60;
				////sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
				printf(MSG_START);
				printf("%s",time);
				autotempShutdown();
      }
      break;
    case 42: //M42 -Change pin status via gcode
  /*    if (code_seen('S'))
      {
        int pin_status = code_value();
        int pin_number = LED_PIN;
        if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
          pin_number = code_value();
        for(int8_t i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
        {
          if (sensitive_pins[i] == pin_number)
          {
            pin_number = -1;
            break;
          }
        }
      #if defined(FAN_PIN) && FAN_PIN > -1
        if (pin_number == FAN_PIN)
          fanSpeed = pin_status;
      #endif
        if (pin_number > -1)
        {
          SET_OUTPUT(pin_number);
          digitalWrite(pin_number, pin_status);
          analogWrite(pin_number, pin_status);
        }
      }*/
     break;
    case 104: // M104
      if(setTargetedHotend(104)){
        break;
      }
      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
      setWatch();
      break;
    case 140: // M140 set bed temp
      if (code_seen('S')) setTargetBed(code_value());
      break;
    case 105 : // M105
      if(setTargetedHotend(105)){
        break;
      }
      #if defined(TEMP_0_PIN)
          printf(MSG_OK);
          printf("\r\n");
        printf("ok T:");
        printf("%.1f",degHotend(tmp_extruder));
        printf(" /");
        printf("%.1f",degTargetHotend(tmp_extruder));
        #if defined(TEMP_BED_PIN)
          printf(" B:");
          printf("%.1f",degBed());
          printf(" /");
          printf("%.1f \n\r",degTargetBed());
        #endif //TEMP_BED_PIN
      #else
        printf(MSG_ERR);
        printf(MSG_ERR_NO_THERMISTORS);
				printf("\n\r");
      #endif

        printf(" @:");
        printf("%d ",getHeaterPower(tmp_extruder));

        printf(" B@:");
        printf("%d",getHeaterPower(-1));
        printf(" \n\r");
      return;
      break;
    case 109:
    {// M109 - Wait for extruder heater to reach target.
      if(setTargetedHotend(109)){
        break;
      }
      #ifdef AUTOTEMP
        autotemp_enabled=false;
      #endif
      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
      #ifdef AUTOTEMP
        if (code_seen('S')) autotemp_min=code_value();
        if (code_seen('B')) autotemp_max=code_value();
        if (code_seen('F'))
        {
          autotemp_factor=code_value();
          autotemp_enabled=true;
        }
      #endif

      setWatch();
      codenum = millis();

      /* See if we are heating up or cooling down */
      bool target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling

      #ifdef TEMP_RESIDENCY_TIME
        long residencyStart;
        residencyStart = -1;
        /* continue to loop until we have reached the target temp _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
				//第一个while是计时TEMP_RESIDENCY_TIME秒
        while((residencyStart == -1) ||
              (residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))) ) {
      #else
        while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) {
// 			//		while ( isHeatingHotend(tmp_extruder)) {
      #endif //TEMP_RESIDENCY_TIME
          if( (millis() - codenum) > 1000UL )
          { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
            printf("T:");
            printf("%.1f ",degHotend(tmp_extruder));
            printf(" E:");
            printf("%d ",(int)tmp_extruder);
						printf(" M:");
						printf("%d ",(int)degTargetHotend(0));
            #ifdef TEMP_RESIDENCY_TIME
              printf(" W:");
              if(residencyStart > -1)
              {
                 codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
                 printf("%ld\n\r",codenum);
              }
              else
              {
                 printf( "? \n\r" );
              }
            #else
              printf("\n\r");
            #endif
            codenum = millis();
          }
//					IWDG_Feed();
          manage_heater();
          manage_inactivity();
//					MySerialLcd.LCD_Run();
		

        #ifdef TEMP_RESIDENCY_TIME
            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
              or when current temp falls outside the hysteresis after target temp was reached */
          if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
              (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
              (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
          {
            residencyStart = millis();
          }
        #endif //TEMP_RESIDENCY_TIME
         }
		    starttime=millis();
        previous_millis_cmd = millis();
      }
      break;
    case 190: // M190 - Wait for bed heater to reach target.
    #if defined(TEMP_BED_PIN)

        if (code_seen('S')) setTargetBed(code_value());
        codenum = millis();
        while(isHeatingBed())
        {
          if(( millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            float tt=degHotend(active_extruder);
            printf("T: %f",tt);          
            printf(" E: %d",(int)active_extruder);
            printf(" B: %.1f\n\r",degBed());
            codenum = millis();
          }
					IWDG_Feed();
          manage_heater();
          manage_inactivity();
					MySerialLcd.LCD_Run();
        }
    #endif
        break;

    #if defined(FAN_PIN)
      case 106: //M106 Fan On
        if (code_seen('S')){
           fanSpeed=constrain(code_value(),0,255);
        }
        else {
          fanSpeed=255;
        }
        break;
      case 107: //M107 Fan Off
        fanSpeed = 0;
        break;
    #endif //FAN_PIN
    #ifdef BARICUDA
      // PWM for HEATER_1_PIN
      #if defined(HEATER_1_PIN) 
        case 126: //M126 valve open
          if (code_seen('S')){
             ValvePressure=constrain(code_value(),0,255);
          }
          else {
            ValvePressure=255;
          }
          break;
        case 127: //M127 valve closed
          ValvePressure = 0;
          break;
      #endif //HEATER_1_PIN

      // PWM for HEATER_2_PIN
      #if defined(HEATER_2_PIN) 
        case 128: //M128 valve open
          if (code_seen('S')){
             EtoPPressure=constrain(code_value(),0,255);
          }
          else {
            EtoPPressure=255;
          }
          break;
        case 129: //M129 valve closed
          EtoPPressure = 0;
          break;
      #endif //HEATER_2_PIN
    #endif

    #if defined(PS_ON_PIN)
      case 80: // M80 - ATX Power On
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, PS_ON_AWAKE);
        break;
      #endif

      case 81: // M81 - ATX Power Off
	    suicide();
      #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
        st_synchronize();
      #elif defined(PS_ON_PIN) 
        SET_OUTPUT(PS_ON_PIN);
        WRITE(PS_ON_PIN, PS_ON_ASLEEP);
      #endif
        break;

    case 82:
      axis_relative_modes[3] = false;
      break;
    case 83:
      axis_relative_modes[3] = true;
      break;
    case 18: //compatibility
    case 84: // M84
      if(code_seen('S')){
        stepper_inactive_time = code_value() * 1000;
      }
      else
      {
        bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
        if(all_axis)
        {
          st_synchronize();
          disable_e0();
          disable_e1();
          disable_e2();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
          if(code_seen('X')) disable_x();
          if(code_seen('Y')) disable_y();
          if(code_seen('Z')) disable_z();
          ////#if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
            if(code_seen('E')) {
              disable_e0();
              disable_e1();
              disable_e2();
            }
         //// #endif
        }
      }
      break;
    case 85: // M85
      code_seen('S');
      max_inactive_time = code_value() * 1000;
      break;
    case 92: // M92
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if(i == 3) { // E
            float value = code_value();
            if(value < 20.0) {
              float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
              max_e_jerk *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            axis_steps_per_unit[i] = value;
          }
          else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      break;
    case 115: // M115
      printf(MSG_M115_REPORT);
      break;
    case 117: // M117 display message
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
////      lcd_setstatus(strchr_pointer + 5);
      break;
    case 114: // M114
      printf("X:");
      printf("%f",current_position[X_AXIS]);
      printf("Y:");
      printf("%f",current_position[Y_AXIS]);
      printf("Z:");
      printf("%f",current_position[Z_AXIS]);
      printf("E:");
      printf("%f  ",current_position[E_AXIS]);

      printf(MSG_COUNT_X);
      printf("%f",float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
      printf("Y: %f",float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
      printf("Z: %f \n\r",float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);
     
      break;
    case 120: // M120
      enable_endstops(false) ;
      break;
    case 121: // M121
      enable_endstops(true) ;
      break;
    case 119: // M119
    printf(MSG_M119_REPORT);
      #if defined(ENDSTOPPULLUP_XMIN)
        printf(MSG_X_MIN);
        printf("%d",((READ(X_MIN_PIN)^X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(ENDSTOPPULLUP_XMAX)
        printf(MSG_X_MAX);
        printf("%d",((READ(X_MAX_PIN)^X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(ENDSTOPPULLUP_YMIN)
        printf(MSG_Y_MIN);
        printf("%d",((READ(Y_MIN_PIN)^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(ENDSTOPPULLUP_YMAX) 
        printf(MSG_Y_MAX);
        printf("%d",((READ(Y_MAX_PIN)^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(ENDSTOPPULLUP_ZMIN) 
        printf(MSG_Z_MIN);
        printf("%d",((READ(Z_MIN_PIN)^Z_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(ENDSTOPPULLUP_ZMAX) 
        printf(MSG_Z_MAX);
        printf("%d",((READ(Z_MAX_PIN)^Z_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      break;
      //TODO: update for all axis, use for loop
    case 201: // M201
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          max_acceleration_units_per_sq_second[i] = code_value();
        }
      }
      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
      reset_acceleration_rates();
      break;
    #if 0 // Not used for Sprinter/grbl gen6
    case 202: // M202
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
      }
      break;
    #endif
    case 203: // M203 max feedrate mm/sec
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
      }
      break;
    case 204: // M204 acclereration S normal moves T filmanent only moves
      {
        if(code_seen('S')) acceleration = code_value() ;
        if(code_seen('T')) retract_acceleration = code_value() ;
      }
      break;
    case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
    {
      if(code_seen('S')) minimumfeedrate = code_value();
      if(code_seen('T')) mintravelfeedrate = code_value();
      if(code_seen('B')) minsegmenttime = code_value() ;
      if(code_seen('X')) max_xy_jerk = code_value() ;
      if(code_seen('Z')) max_z_jerk = code_value() ;
      if(code_seen('E')) max_e_jerk = code_value() ;
    }
    break;
    case 206: // M206 additional homeing offset
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
      }
      break;
    #ifdef FWRETRACT
    case 207: //M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
    {
      if(code_seen('S'))
      {
        retract_length = code_value() ;
      }
      if(code_seen('F'))
      {
        retract_feedrate = code_value() ;
      }
      if(code_seen('Z'))
      {
        retract_zlift = code_value() ;
      }
    }break;
    case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
    {
      if(code_seen('S'))
      {
        retract_recover_length = code_value() ;
      }
      if(code_seen('F'))
      {
        retract_recover_feedrate = code_value() ;
      }
    }break;
    case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
    {
      if(code_seen('S'))
      {
        int t= code_value() ;
        switch(t)
        {
          case 0: autoretract_enabled=false;retracted=false;break;
          case 1: autoretract_enabled=true;retracted=false;break;
          default:
            printf(MSG_START);
            printf(MSG_UNKNOWN_COMMAND);
            printf("%s",cmdbuffer[bufindr]);
            printf("\"\n\r");
        }
      }

    }break;
    #endif // FWRETRACT
    #if EXTRUDERS > 1
    case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
    {
      if(setTargetedHotend(218)){
        break;
      }
      if(code_seen('X'))
      {
        extruder_offset[X_AXIS][tmp_extruder] = code_value();
      }
      if(code_seen('Y'))
      {
        extruder_offset[Y_AXIS][tmp_extruder] = code_value();
      }
      printf(MSG_START);
      printf(MSG_HOTEND_OFFSET);
      for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
      {
         printf(" ");
         printf("%f",extruder_offset[X_AXIS][tmp_extruder]);
         printf(",");
         printf("%f",extruder_offset[Y_AXIS][tmp_extruder]);
      }
      printf("\n\r");
    }break;
    #endif
    case 220: // M220 S<factor in percent>- set speed factor override percentage
    {
      if(code_seen('S'))
      {
        feedmultiply = code_value() ;
      }
    }
    break;
    case 221: // M221 S<factor in percent>- set extrude factor override percentage
    {
      if(code_seen('S'))
      {
        extrudemultiply = code_value() ;
      }
    }
    break;
    
    #if NUM_SERVOS > 0
    case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
      {
        int servo_index = -1;
        int servo_position = 0;
        if (code_seen('P'))
          servo_index = code_value();
        if (code_seen('S')) {
          servo_position = code_value();
          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
            servos[servo_index].write(servo_position);
          }
          else {
            printf(MSG_START);
            printf("Servo ");
            printf("%d",servo_index);
            printf(" out of range");
          }
        }
        else if (servo_index >= 0) {
          printf(MSG_OK);
          printf(" Servo ");
          printf(servo_index);
          printf(": ");
          printf("%d",servos[servo_index].read());
          printf("");
        }
      }
      break;
    #endif // NUM_SERVOS > 0

    #if LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) )
    case 300: // M300
    {
      int beepS = 400;
      int beepP = 1000;
      if(code_seen('S')) beepS = code_value();
      if(code_seen('P')) beepP = code_value();
      #if BEEPER > 0
        tone(BEEPER, beepS);
        delay(beepP);
        noTone(BEEPER);
			#endif
    }
    break;
    #endif // M300

    #ifdef PIDTEMP
    case 301: // M301
      {
        if(code_seen('P')) Kp = code_value();
        if(code_seen('I')) Ki = scalePID_i(code_value());
        if(code_seen('D')) Kd = scalePID_d(code_value());

        #ifdef PID_ADD_EXTRUSION_RATE
        if(code_seen('C')) Kc = code_value();
        #endif

        updatePID();
        printf(MSG_OK);
        printf(" p:");
        printf("%f",Kp);
        printf(" i:");
        printf("%f",unscalePID_i(Ki));
        printf(" d:");
        printf("%f",unscalePID_d(Kd));
        #ifdef PID_ADD_EXTRUSION_RATE
        printf(" c:");
        //Kc does not have scaling applied above, or in resetting defaults
        printf("%f",Kc);
        #endif
        printf("\n\r");
      }
      break;
    #endif //PIDTEMP
    #ifdef PIDTEMPBED
    case 304: // M304
      {
        if(code_seen('P')) bedKp = code_value();
        if(code_seen('I')) bedKi = scalePID_i(code_value());
        if(code_seen('D')) bedKd = scalePID_d(code_value());

        updatePID();
        printf(MSG_OK);
        printf(" p:");
        printf("%0.3f",bedKp);
        printf(" i:");
        printf("%0.3f",unscalePID_i(bedKi));
				printf(" d:");
        printf("%0.3f",unscalePID_d(bedKd));
        printf("\n");
      }
      break;
    #endif //PIDTEMP
    case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
     {
      #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
        const uint8_t NUM_PULSES=16;
        const float PULSE_LENGTH=0.01524;
        for(int i=0; i < NUM_PULSES; i++) {
          WRITE(PHOTOGRAPH_PIN, HIGH);
          _delay_ms(PULSE_LENGTH);
          WRITE(PHOTOGRAPH_PIN, LOW);
          _delay_ms(PULSE_LENGTH);
        }
        delay(7.33);
        for(int i=0; i < NUM_PULSES; i++) {
          WRITE(PHOTOGRAPH_PIN, HIGH);
          _delay_ms(PULSE_LENGTH);
          WRITE(PHOTOGRAPH_PIN, LOW);
          _delay_ms(PULSE_LENGTH);
        }
      #endif
     }
    break;
    #ifdef PREVENT_DANGEROUS_EXTRUDE
    case 302: // allow cold extrudes, or set the minimum extrude temperature
    {
	  float temp = .0;
	  if (code_seen('S')) temp=code_value();
      set_extrude_min_temp(temp);
    }
    break;
	#endif
    case 303: // M303 PID autotune
    {
      float temp = 150.0;
      int e=0;
      int c=5;
      if (code_seen('E')) e=code_value();
        if (e<0)
          temp=70;
      if (code_seen('S')) temp=code_value();
      if (code_seen('C')) c=code_value();
      PID_autotune(temp, e, c);
    }
    break;
    case 400: // M400 finish all moves
    {
      st_synchronize(); 
    }
    break;
    case 500: // M500 Store settings in EEPROM
    {
//        Config_StoreSettings();
    }
    break;
    case 501: // M501 Read settings from EEPROM
    {
   //     Config_RetrieveSettings();
    }
    break;
    case 502: // M502 Revert to default settings
    {
//        Config_ResetDefault();
    }
    break;
    case 503: // M503 print settings currently in memory
    {
        Config_PrintSettings();
    }
    break;
    #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
    case 540:
    {
        if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
    }
    break;
    #endif
    #ifdef FILAMENTCHANGEENABLE
    case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
    {
        float target[4];
        float lastpos[4];
        target[X_AXIS]=current_position[X_AXIS];
        target[Y_AXIS]=current_position[Y_AXIS];
        target[Z_AXIS]=current_position[Z_AXIS];
        target[E_AXIS]=current_position[E_AXIS];
        lastpos[X_AXIS]=current_position[X_AXIS];
        lastpos[Y_AXIS]=current_position[Y_AXIS];
        lastpos[Z_AXIS]=current_position[Z_AXIS];
        lastpos[E_AXIS]=current_position[E_AXIS];
        //retract by E
        if(code_seen('E'))
        {
          target[E_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FIRSTRETRACT
            target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT ;
          #endif
        }
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

        //lift Z
        if(code_seen('Z'))
        {
          target[Z_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_ZADD
            target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
          #endif
        }
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

        //move xy
        if(code_seen('X'))
        {
          target[X_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_XPOS
            target[X_AXIS]= FILAMENTCHANGE_XPOS ;
          #endif
        }
        if(code_seen('Y'))
        {
          target[Y_AXIS]= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_YPOS
            target[Y_AXIS]= FILAMENTCHANGE_YPOS ;
          #endif
        }

        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

        if(code_seen('L'))
        {
          target[E_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FINALRETRACT
            target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT ;
          #endif
        }

        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

        //finish moves
        st_synchronize();
        //disable extruder steppers so filament can be removed
        disable_e0();
        disable_e1();
        disable_e2();
        delay(100);
    //    LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
//         uint8_t cnt=0;
//         while(!lcd_clicked()){
//           cnt++;
// 					IWDG_Feed();
//           manage_heater();
//           manage_inactivity();
// 					MySerialLcd.LCD_Run();
//           if(cnt==0)
//           {
//           #if BEEPER > 0
//             SET_OUTPUT(BEEPER);

//             WRITE(BEEPER,HIGH);
//             delay(3);
//             WRITE(BEEPER,LOW);
//             delay(3);
//           #endif
//           }
//         }

        //return to normal
        if(code_seen('L'))
        {
          target[E_AXIS]+= -code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FINALRETRACT
            target[E_AXIS]+=(-1)*FILAMENTCHANGE_FINALRETRACT ;
          #endif
        }
        current_position[E_AXIS]=target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
        plan_set_e_position(current_position[E_AXIS]);
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //should do nothing
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move xy back
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move z back
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //final untretract
    }
    break;
    #endif //FILAMENTCHANGEENABLE
    case 907: // M907 Set digital trimpot motor current using axis codes.
    {
      #if defined(DIGIPOTSS_PIN) 
//         for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
//         if(code_seen('B')) digipot_current(4,code_value());
//         if(code_seen('S')) for(int i=0;i<=4;i++) digipot_current(i,code_value());
      #endif
    }
    break;
    case 908: // M908 Control digital trimpot directly.
    {
      #if defined(DIGIPOTSS_PIN) 
//         uint8_t channel,current;
//         if(code_seen('P')) channel=code_value();
//         if(code_seen('S')) current=code_value();
//         digitalPotWrite(channel, current);
      #endif
    }
    break;
    case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
    {
      #if defined(X_MS1_PIN) 
        if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value());
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
        if(code_seen('B')) microstep_mode(4,code_value());
        microstep_readings();
      #endif
    }
    break;
    case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
    {
      #if defined(X_MS1_PIN) 
      if(code_seen('S')) switch((int)code_value())
      {
        case 1:
          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1);
          if(code_seen('B')) microstep_ms(4,code_value(),-1);
          break;
        case 2:
          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value());
          if(code_seen('B')) microstep_ms(4,-1,code_value());
          break;
      }
      microstep_readings();
      #endif
    }
    break;
    case 999: // M999: Restart after being stopped
      Stopped = false;
////      lcd_reset_alert_level();
      gcode_LastN = Stopped_gcode_LastN;
			kill();
      FlushSerialRequestResend();
    break;
    }
  }

  else if(code_seen('T'))
  {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      printf(MSG_START);;
      printf("T");
      printf("%.1f",tmp_extruder);//串口信息异常
      printf(MSG_INVALID_EXTRUDER);
    }
    else {
      bool make_move = false;
      if(code_seen('F')) {
        make_move = true;
        next_feedrate = code_value();
        if(next_feedrate > 0.0) {
          feedrate = next_feedrate;
        }
      }
      #if EXTRUDERS > 1
      if(tmp_extruder != active_extruder) {
        // Save current position to return to after applying extruder offset
        memcpy(destination, current_position, sizeof(destination));
        // Offset extruder (only by XY)
        int i;
        for(i = 0; i < 2; i++) {
           current_position[i] = current_position[i] -
                                 extruder_offset[i][active_extruder] +
                                 extruder_offset[i][tmp_extruder];
        }
        // Set the new active extruder and position
        active_extruder = tmp_extruder;
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        // Move to the old position if 'F' was in the parameters
        if(make_move && Stopped == false) {
           prepare_move();
        }
      }
      #endif
      printf(MSG_START);
      printf(MSG_ACTIVE_EXTRUDER);
      printf("%d\n\r",(int)active_extruder);
    }
  }
  else
  {
    printf(MSG_START);
    printf(MSG_UNKNOWN_COMMAND);
    printf("%s",cmdbuffer[bufindr]);
    printf("\"\n\r");
  }

  ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  printf(MSG_RESEND);
  printf("%ld\n\r",gcode_LastN + 1);
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  #ifdef SDSUPPORT
  if(fromsd[bufindr])
    return;
  #endif //SDSUPPORT
  printf(MSG_OK);
  printf("\r\n");///原本没有这句话，为了配合打印机上位机添加的
}
//获取运动坐标
void get_coordinates()
{
  bool seen[4]={false,false,false,false}; 
  for(int8_t i=0; i < NUM_AXIS; i++) { //XYZE循环4次
    if(code_seen(axis_codes[i])) //寻找XYZE字符
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];  //目标地址
      seen[i]=true;
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  if(code_seen('F')) { //F后面跟的是速度
    next_feedrate = code_value(); 
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
  #ifdef FWRETRACT
  if(autoretract_enabled)	
  if(!(seen[X_AXIS] || seen[Y_AXIS] || seen[Z_AXIS]) && seen[E_AXIS])
  {
    float echange=destination[E_AXIS]-current_position[E_AXIS];
    if(echange<-MIN_RETRACT) //retract
    {
      if(!retracted)
      {
				destination[Z_AXIS]+=retract_zlift; //not sure why chaninging current_position negatively does not work.
				//if slicer retracted by echange=-1mm and you want to retract 3mm, corrrectede=-2mm additionally
				float correctede=-echange-retract_length;
				//to generate the additional steps, not the destination is changed, but inversely the current position
				current_position[E_AXIS]+=-correctede;
				feedrate=retract_feedrate;
				retracted=true;
      }
    }
    else
    if(echange>MIN_RETRACT) //retract_recover
    {
      if(retracted)
      {
		  //current_position[Z_AXIS]+=-retract_zlift;
		  //if slicer retracted_recovered by echange=+1mm and you want to retract_recover 3mm, corrrectede=2mm additionally
		  float correctede=-echange+1*retract_length+retract_recover_length; //total unretract=retract_length+retract_recover_length[surplus]
		  current_position[E_AXIS]+=correctede; //to generate the additional steps, not the destination is changed, but inversely the current position
		  feedrate=retract_recover_feedrate;
		  retracted=false;
      }
    }

  }
  #endif //FWRETRACT
}

void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   get_coordinates();
#ifdef SF_ARC_FIX
   relative_mode=relative_mode_backup;
#endif

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

void clamp_to_software_endstops(float target[3])  //——目前的位置应该最大不超过限位
{
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}
//准备运动函数
void prepare_move()
{
	
  clamp_to_software_endstops(destination);	//位置的限定
  previous_millis_cmd = millis();   //获取当前系统时间
  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) { //XY不运动
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  }
  else { //XY运动的时候支持调速
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  }
  for(int8_t i=0; i < NUM_AXIS; i++) { //保存目标坐标是当前坐标
    current_position[i] = destination[i];
  }
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();
}

#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1

#if defined(FAN_PIN)
  #if CONTROLLERFAN_PIN == FAN_PIN 
    #error "You cannot set CONTROLLERFAN_PIN equal to FAN_PIN"
  #endif
#endif  

unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

//函数说明:驱动器主板散热风扇
//输入参数：无
//输出参数：无
//备注：无
void controllerFan()
{
  if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
  {
    lastMotorCheck = millis();

    if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN)
    #if EXTRUDERS > 2
       || !READ(E2_ENABLE_PIN)
    #endif
    #if EXTRUDER > 1
       || !READ(E1_ENABLE_PIN)
    #endif
       || !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...
    {
      lastMotor = millis(); //... set time to NOW so the fan will turn on
    }
    
    if ((millis() - lastMotor) >= (CONTROLLERFAN_SECS*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...   
    {
			WRITE(CONTROLLERFAN_PIN,0);
    }
    else
    {
			WRITE(CONTROLLERFAN_PIN,0);
        // allows digital or PWM fan output to be used (see M42 handling)
    }
  }
}
#endif

void manage_inactivity()
{
  if( (millis() - previous_millis_cmd) >  max_inactive_time )
    if(max_inactive_time)
      kill();
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false) {
        disable_x();
        disable_y();
        disable_z();
        disable_e0();
        disable_e1();
        disable_e2();
      }
    }
  }
  #if defined(KILL_PIN)
    if( 0 == READ(KILL_PIN) )
      kill();
  #endif
  #if defined(CONTROLLERFAN_PIN)
    controllerFan(); //Check if fan should be turned on to cool stepper drivers down
  #endif
  #ifdef EXTRUDER_RUNOUT_PREVENT
    if( (millis() - previous_millis_cmd) >  EXTRUDER_RUNOUT_SECONDS*1000 )
    if(degHotend(active_extruder)>EXTRUDER_RUNOUT_MINTEMP)
    {
     bool oldstatus=READ(E0_ENABLE_PIN);
     enable_e0();
     float oldepos=current_position[E_AXIS];
     float oldedes=destination[E_AXIS];
     plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
                      current_position[E_AXIS]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS],
                      EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], active_extruder);
     current_position[E_AXIS]=oldepos;
     destination[E_AXIS]=oldedes;
     plan_set_e_position(oldepos);
     previous_millis_cmd=millis();
     st_synchronize();
     WRITE(E0_ENABLE_PIN,oldstatus);
    }
  #endif
  check_axes_activity();
}

//函数说明：异常情况下关闭电机和加热,然后复位
//输入参数：
//输出参数：
//备注：加热或者温度异常才能调用该函数
void kill()
{
  cli(); // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();

#if defined(PS_ON_PIN) 
	SET_INPUT(PS_ON_PIN);
#endif  
  printf(MSG_START);
  printf(MSG_ERR_KILLED);
	printf("\n\r");
  ////LCD_ALERTMESSAGEPGM(MSG_KILLED);
  suicide(); //软复位
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void Stop()
{
  disable_heater();
  if(Stopped == false) {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    printf(MSG_START);
    printf(MSG_ERR_STOPPED);
  }
}

bool IsStopped() { return Stopped; };

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{

	
	
}
#endif //FAST_PWM_FAN

bool setTargetedHotend(int code){
  tmp_extruder = active_extruder;
  if(code_seen('T')) {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      printf(MSG_START);
      switch(code){
        case 104:
          printf(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          printf(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          printf(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          printf(MSG_M218_INVALID_EXTRUDER);
          break;
      }
      printf("%d",tmp_extruder);
      return true;
    }
  }
  return false;
}




