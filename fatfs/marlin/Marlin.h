// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H


//#define  FORCE_INLINE  __INLINE
#define  FORCE_INLINE  inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "Configuration.h"
#include "bsp_usart.h"

#include "bsp_sysclk.h"

#include "bsp_pin.h"

#define PSTR(STR)  STR


#define HIGH  true
#define LOW   false


#define millis()   system_ms




#define cli()  __disable_irq()//__disable_fault_irq

#define sei()  __enable_irq()//__enable_fault_irq  


#define max(a,b) (a>b?a:b)

#define min(a,b) (a<b?a:b)



#define M_PI 3.1515


#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))




#define CRITICAL_SECTION_START cli()

#define CRITICAL_SECTION_END  sei()

#define lround(N) ((floor)(N+0.5f))

#define F_CPU 16000000UL



#define MYSERIAL MSerial






#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))

//void serial_echopair_P(const char *s_P, float v);
//void serial_echopair_P(const char *s_P, double v);
//void serial_echopair_P(const char *s_P, unsigned long v);


//things to write to serial from Programmemory. saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str)
{
  //char ch=pgm_read_byte(str);
	char const*pch = str;
  while(*pch)
  {
    printf("%d",*pch);
		pch++;
   // ch=pgm_read_byte(++str);
  }
}


void get_command();
void process_commands();

void manage_inactivity();

#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

#if defined X_ENABLE_PIN 
  #define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
  #define enable_x()
	#define disable_x() 
#endif

#if defined Y_ENABLE_PIN  
  #define enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
  #define enable_y() 
  #define disable_y() 
#endif

#if defined(Z_ENABLE_PIN) 
  #ifdef Z_DUAL_STEPPER_DRIVERS
    #define  enable_z() { WRITE(Z_ENABLE_PIN, Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); }
    #define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON); }
  #else
    #define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
    #define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
  #endif
#else
  #define enable_z() 
  #define disable_z() 
#endif

#if defined(E0_ENABLE_PIN) 
  #define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) 
  #define enable_e1()  WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
#else
  #define enable_e1() /* nothing */
  #define disable_e1()  /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN)
  #define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e2() WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e2() ;/* nothing */
  #define disable_e2() ;/* nothing */
#endif


enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};




void setup();
void loop();


void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void prepare_move();
void kill();
void Stop();

bool IsStopped();

void enquecommand(const char *cmd); //put an ascii command at the end of the current buffer.
void enquecommand_P(const char *cmd); //put an ascii command at the end of the current buffer, read from flash
void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);


float get_platform_position(float x,float y);

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

extern volatile bool relative_mode; 

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent)
extern float current_position[NUM_AXIS] ;
extern float destination[NUM_AXIS];
extern float add_homeing[3];
extern float min_pos[3];
extern float max_pos[3];
extern int fanSpeed;
#ifdef BARICUDA
extern int ValvePressure;
extern int EtoPPressure;
#endif

#ifdef FWRETRACT
extern bool autoretract_enabled;
extern bool retracted;
extern float retract_length, retract_feedrate, retract_zlift;
extern float retract_recover_length, retract_recover_feedrate;
#endif

extern unsigned long starttime;
extern unsigned long stoptime;

// Handling multiple extruders pins
extern uint8_t active_extruder;


extern volatile uint16_t print_total_time;	     //总的打印时间，在gcode里面如：" ;Print time: 10 minutes "
extern volatile uint16_t print_total_layer;      //总的打印层数，在gcode里面如：" ;Layer count: 55 "
extern volatile uint16_t print_current_layer;    //当前打印层数，在gcode里面如: " ;LAYER:0 "
extern volatile uint16_t print_start_layer;



extern volatile unsigned long previous_millis_cmd;
extern bool Stopped;
#endif

