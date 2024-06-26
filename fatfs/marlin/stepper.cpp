/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"
#include "language.h"
#include "speed_lookuptable.h"
#include "bsp_pin.h"
#include "bsp_usart.h"
#include "bsp_watchdog.h"
//#include "Seriallcd.h"
#include "bsp_watchdog.h"

//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced


//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it inpossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static long counter_x,       // Counter variables for the bresenham line tracer
            counter_y, 
            counter_z,       
            counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block
#ifdef ADVANCE
  static long advance_rate, advance, final_advance = 0;
  static long old_advance = 0;
  static long e_steps[3];
#endif
static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for deccelaration start point
static char step_loops;
static unsigned short OCR1A_nominal;
static unsigned short step_loops_nominal;

volatile long endstops_trigsteps[3]={0,0,0};
volatile long endstops_stepsTotal,endstops_stepsDone;
static volatile bool endstop_x_hit=false;
static volatile bool endstop_y_hit=false;
static volatile bool endstop_z_hit=false;
volatile long platform_trigsteps; //用来保存校平台的触发开关的触发坐标值，用脉冲数来表示
volatile bool platform_hit=false;//用来保存校平台的触发开关的触发状态

#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
bool abort_on_endstop_hit = false;
#endif

static bool old_x_min_endstop=false; 
static bool old_x_max_endstop=false;
static bool old_y_min_endstop=false;
static bool old_y_max_endstop=false;
static bool old_z_min_endstop=false;
static bool old_z_max_endstop=false;
static bool old_platform_endstop = false;

static bool check_endstops = true;
bool check_platform = true;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0}; //在定时器中断里用来标记路径的脉冲数
volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1}; //用来标记脉冲的方向


volatile bool moving_flag = false; //____上位机


//===========================================================================
//=============================functions         ============================
//===========================================================================

#define CHECK_ENDSTOPS  if(check_endstops)


#define MultiU16X8toH16(intRes, charIn1, intIn2) (intRes = (((uint32_t)charIn1 * (uint32_t)intIn2)>>16))
//#define MultiU16X8toH16(intRes, charIn1, intIn2) intRes = (uint16_t)(((uint8_t)((uint8_t)charIn1 * (uint16_t)intIn2))>> 16)
// intRes = intIn1 * intIn2 >> 16

#define MultiU24X24toH16(intRes, longIn1, longIn2) (intRes = (uint32_t)(((uint64_t)longIn1*(uint64_t)longIn2)>>24))
// intRes = longIn1 * longIn2 >> 24



// Some useful constants

#define ENABLE_STEPPER_DRIVER_INTERRUPT() 	TIM_Cmd(TIM2, ENABLE)   
#define DISABLE_STEPPER_DRIVER_INTERRUPT()  TIM_Cmd(TIM2, DISABLE)  


void checkHitEndstops()
{
 if( endstop_x_hit || endstop_y_hit || endstop_z_hit) {
   printf(MSG_START);
   printf(MSG_ENDSTOPS_HIT);
   if(endstop_x_hit) {
     printf(" X: %f ",(float)endstops_trigsteps[X_AXIS]/axis_steps_per_unit[X_AXIS]);
   }
   if(endstop_y_hit) {
     printf(" Y: %f ",(float)endstops_trigsteps[Y_AXIS]/axis_steps_per_unit[Y_AXIS]);
   }
   if(endstop_z_hit) {
     printf(" Z: %f ",(float)endstops_trigsteps[Z_AXIS]/axis_steps_per_unit[Z_AXIS]);
   }
   printf("\r\n");
   endstop_x_hit=false;
   endstop_y_hit=false;
   endstop_z_hit=false;
	 
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
   if (abort_on_endstop_hit)
   {
     card.sdprinting = false;
     card.closefile();
     quickStop();
     setTargetHotend0(0);
     setTargetHotend1(0);
     setTargetHotend2(0);
   }
#endif
 }
}

void endstops_hit_on_purpose()
{
  endstop_x_hit=false;
  endstop_y_hit=false;
  endstop_z_hit=false;
}

void enable_endstops(bool check)
{
  check_endstops = check;
}

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates 
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.

void st_wake_up() {
  ENABLE_STEPPER_DRIVER_INTERRUPT();  
}

void step_wait(){
    for(int8_t i=0; i < 6; i++){
    }
}
  
//脉冲速度计算
FORCE_INLINE unsigned short calc_timer1(unsigned short step_rate) {
  unsigned short timer;
  if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;
  
  if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2)&0x3fff;   //step_rate=(step_rate/4)%1024;
    step_loops = 4;
  }
  else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1)&0x7fff;  //step_rate=(step_rate/2)%32768;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  }   
  
  if(step_rate < (F_CPU/500000)) step_rate = (F_CPU/500000); 
	step_rate -= (F_CPU/500000); // Correct for minimal speed
  if(step_rate >= (8*256)){ // higher step rate 
    unsigned short *table_address = (unsigned short*)(&(speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0]));
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = *(unsigned short*)(table_address+1);//*(&((unsigned char*)(table_address+2)));
    MultiU16X8toH16(timer, tmp_step_rate, gain); //timer=tmp_step_rate*gain>>16;
    timer = (*(unsigned short*)(table_address+0)) - timer;
  }
  else { // lower step rates
    unsigned short *table_address = (unsigned short*)(&speed_lookuptable_slow[0][0]);
    table_address += ((step_rate)>>2) & 0xfffc;
    timer = *(unsigned short*)(table_address+0);
    timer -= (((*(unsigned short*)(table_address+1)) * (unsigned char)(step_rate & 0x0007))>>3);
  }
  if(timer < 100) { timer = 100; printf(MSG_STEPPER_TO_HIGH); printf("%d",step_rate); }//(20kHz this should never happen)
  return timer;
}

//脉冲速度计算
FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
  unsigned short timer;
  if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;
  
  if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2)&0x3fff;   //step_rate=(step_rate/4)%1024;
    step_loops = 4;
  }
  else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1)&0x7fff;  //step_rate=(step_rate/2)%32768;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  }   
	timer = 1000000/step_rate;
  if(timer < 100) { timer = 100; printf(MSG_STEPPER_TO_HIGH); printf("%d",step_rate); }//(20kHz this should never happen) //步进电机速度过快
  return timer;
}
// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
FORCE_INLINE void trapezoid_generator_reset() {
  #ifdef ADVANCE
    advance = current_block->initial_advance;
    final_advance = current_block->final_advance;
    // Do E steps + advance steps
    e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
    old_advance = advance >>8;  
  #endif
  deceleration_time = 0;
  // step_rate to timer interval
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
	
  acceleration_time = calc_timer(acc_step_rate);
	
	TIM_SetAutoreload(TIM2,acceleration_time);
////    OCR1A = acceleration_time;
////    printf(MSG_START);;
////    SERIAL_ECHOPGM("advance :");
////    SERIAL_ECHO(current_block->advance/256.0);
////    SERIAL_ECHOPGM("advance rate :");
////    SERIAL_ECHO(current_block->advance_rate/256.0);
////    SERIAL_ECHOPGM("initial advance :");
////    SERIAL_ECHO(current_block->initial_advance/256.0);
////    SERIAL_ECHOPGM("final advance :");
////    SERIAL_ECHOLN(current_block->final_advance/256.0);
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 

////ISR(TIMER1_COMPA_vect)

#ifdef __cplusplus
extern "C" {
void TIM2_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler()
{ 
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		//清除TIM2的中断待处理位
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
		// If there is no current block, attempt to pop one from the buffer
		if (current_block == NULL) { 
			current_block = plan_get_current_block(); //从插补队列的block_buffer_tail中取出插补队列块
			if(current_block != NULL) {
				current_block->busy = true; 
				trapezoid_generator_reset();
				counter_x = -(current_block->step_event_count >> 1);	//取出改路径的插补数
				counter_y = counter_x;
				counter_z = counter_x;
				counter_e = counter_x;
				step_events_completed = 0; 
      
      #ifdef Z_LATE_ENABLE 
        if(current_block->steps_z > 0) {
          enable_z();
					TIM_SetAutoreload(TIM2,2000);
          //OCR1A = 2000; //1ms wait
          return;
        }
      #endif
	  	  
      #ifdef ADVANCE
      e_steps[current_block->active_extruder] = 0;
      #endif
    } 
    else {
				TIM_SetAutoreload(TIM2,2000);
     ////  OCR1A=2000; // 1kHz. //如果没有插补队列数据要处理
    }    
  } 

  if (current_block != NULL) {
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits; //读出运动方向

    // Set direction en check limit switches
    if ((out_bits & (1<<X_AXIS)) != 0) { //如果方向是朝-X方向
      #if !defined COREXY  //NOT COREXY
        WRITE(X_DIR_PIN, INVERT_X_DIR);
      #endif
      count_direction[X_AXIS]=-1;
      CHECK_ENDSTOPS   // if(check_endstops)
      {
        #if defined(ENDSTOPPULLUP_XMIN)&defined(X_MIN_PIN) //如果X_MIN有配置
          bool x_min_endstop=(READ(X_MIN_PIN) != X_ENDSTOPS_INVERTING); //如果X_MIN被触发，x_min_endstop=1
          if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
            endstops_trigsteps[X_AXIS] = count_position[X_AXIS]; //保存当前X轴的脉冲坐标
            endstop_x_hit=true;
            step_events_completed = current_block->step_event_count; //结束当前插补
          }
          old_x_min_endstop = x_min_endstop;
        #endif
      }
    }
    else { //如果方向是朝X+方向
      #if !defined COREXY  //NOT COREXY
        WRITE(X_DIR_PIN,!INVERT_X_DIR); 
      #endif   
      count_direction[X_AXIS]=1;
	  CHECK_ENDSTOPS 
      {
        #if defined(ENDSTOPPULLUP_XMAX)&defined(X_MAX_PIN) 
          bool x_max_endstop=(READ(X_MAX_PIN) != X_ENDSTOPS_INVERTING);
          if((x_max_endstop && old_x_max_endstop) &&(current_block->steps_x > 0)){
            endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
            endstop_x_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_x_max_endstop = x_max_endstop;
        #endif
      }
    }

    if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
      #if !defined COREXY  //NOT COREXY
        WRITE(Y_DIR_PIN,INVERT_Y_DIR);
      #endif
      count_direction[Y_AXIS]=-1;
      CHECK_ENDSTOPS
      {
        #if defined(ENDSTOPPULLUP_YMIN)&defined(Y_MIN_PIN)
          bool y_min_endstop=(READ(Y_MIN_PIN) != Y_ENDSTOPS_INVERTING);
          if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_y_min_endstop = y_min_endstop;
        #endif
      }
    }
    else { // +direction
      #if !defined COREXY  //NOT COREXY
        WRITE(Y_DIR_PIN,!INVERT_Y_DIR);
      #endif
      count_direction[Y_AXIS]=1;
      CHECK_ENDSTOPS
      {
        #if defined(ENDSTOPPULLUP_YMAX)&defined(Y_MAX_PIN)
          bool y_max_endstop=(READ(Y_MAX_PIN) != Y_ENDSTOPS_INVERTING);
          if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_y_max_endstop = y_max_endstop;
        #endif
      }
    }
    
    
    #ifdef COREXY  //coreXY kinematics defined
      if((current_block->steps_x >= current_block->steps_y)&&((out_bits & (1<<X_AXIS)) == 0)){  //+X is major axis
        WRITE(X_DIR_PIN, !INVERT_X_DIR);
        WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
      }
      if((current_block->steps_x >= current_block->steps_y)&&((out_bits & (1<<X_AXIS)) != 0)){  //-X is major axis
        WRITE(X_DIR_PIN, INVERT_X_DIR);
        WRITE(Y_DIR_PIN, INVERT_Y_DIR);
      }      
      if((current_block->steps_y > current_block->steps_x)&&((out_bits & (1<<Y_AXIS)) == 0)){  //+Y is major axis
        WRITE(X_DIR_PIN, !INVERT_X_DIR);
        WRITE(Y_DIR_PIN, INVERT_Y_DIR);
      }        
      if((current_block->steps_y > current_block->steps_x)&&((out_bits & (1<<Y_AXIS)) != 0)){  //-Y is major axis
        WRITE(X_DIR_PIN, INVERT_X_DIR);
        WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
      }  
    #endif //coreXY
    
    
    if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
      WRITE(Z_DIR_PIN,INVERT_Z_DIR);
      
	  #ifdef Z_DUAL_STEPPER_DRIVERS
        WRITE(Z2_DIR_PIN,INVERT_Z_DIR);
      #endif
      
      count_direction[Z_AXIS]=-1;
      CHECK_ENDSTOPS
	  {
        #if defined(ENDSTOPPULLUP_ZMIN)&defined(Z_MIN_PIN) 
          bool z_min_endstop=(READ(Z_MIN_PIN) != Z_ENDSTOPS_INVERTING);
          if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_min_endstop = z_min_endstop;
        #endif
      }
	  if(check_platform) //如果定义了校平台开关
 	  {

	  }
    }
    else { // +direction
      WRITE(Z_DIR_PIN,!INVERT_Z_DIR);

	  #ifdef Z_DUAL_STEPPER_DRIVERS
        WRITE(Z2_DIR_PIN,!INVERT_Z_DIR);
      #endif

      count_direction[Z_AXIS]=1;
      CHECK_ENDSTOPS
      {
        #if defined(ENDSTOPPULLUP_ZMAX)&defined(Z_MAX_PIN)
          bool z_max_endstop=(READ(Z_MAX_PIN) != Z_ENDSTOPS_INVERTING);
          if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_max_endstop = z_max_endstop;
        #endif
      }
    }


    #ifndef ADVANCE
      if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
        REV_E_DIR();
        count_direction[E_AXIS]=-1;
      }
      else { // +direction
        NORM_E_DIR();
        count_direction[E_AXIS]=1;
      }
    #endif //!ADVANCE
    

    
    for(int8_t i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves) 

      #ifdef ADVANCE
      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        counter_e -= current_block->step_event_count;
        if ((out_bits & (1<<E_AXIS)) != 0) { // - direction
          e_steps[current_block->active_extruder]--;
        }
        else {
          e_steps[current_block->active_extruder]++;
        }
      }    
      #endif //ADVANCE

      #if !defined COREXY      
        counter_x += current_block->steps_x; //X插补计算
        if (counter_x > 0) {
          WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
          counter_x -= current_block->step_event_count;
          count_position[X_AXIS]+=count_direction[X_AXIS];   //X的坐标脉冲累加/累减
          WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
        }
  
        counter_y += current_block->steps_y; //Y轴插补
        if (counter_y > 0) {
          WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
          counter_y -= current_block->step_event_count; 
          count_position[Y_AXIS]+=count_direction[Y_AXIS]; 
          WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
        }
      #endif
  
      #ifdef COREXY
        counter_x += current_block->steps_x;        
        counter_y += current_block->steps_y;
        
        if ((counter_x > 0)&&!(counter_y>0)){  //X step only
          WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
          WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
          counter_x -= current_block->step_event_count; 
          count_position[X_AXIS]+=count_direction[X_AXIS];         
          WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
          WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
        }
        
        if (!(counter_x > 0)&&(counter_y>0)){  //Y step only
          WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
          WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
          counter_y -= current_block->step_event_count; 
          count_position[Y_AXIS]+=count_direction[Y_AXIS];
          WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
          WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
        }        
        
        if ((counter_x > 0)&&(counter_y>0)){  //step in both axes
          if (((out_bits & (1<<X_AXIS)) == 0)^((out_bits & (1<<Y_AXIS)) == 0)){  //X and Y in different directions
            WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
            counter_x -= current_block->step_event_count;             
            WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
            step_wait();
            count_position[X_AXIS]+=count_direction[X_AXIS];
            count_position[Y_AXIS]+=count_direction[Y_AXIS];
            WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
            counter_y -= current_block->step_event_count;
            WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
          }
          else{  //X and Y in same direction
            WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
            counter_x -= current_block->step_event_count;             
            WRITE(X_STEP_PIN, INVERT_X_STEP_PIN) ;
            step_wait();
            count_position[X_AXIS]+=count_direction[X_AXIS];
            count_position[Y_AXIS]+=count_direction[Y_AXIS];
            WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN); 
            counter_y -= current_block->step_event_count;    
            WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);        
          }
        }
      #endif //corexy

      counter_z += current_block->steps_z; //Z轴插补
      if (counter_z > 0) {
        WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN);
        
		#ifdef Z_DUAL_STEPPER_DRIVERS
          WRITE(Z2_STEP_PIN, !INVERT_Z_STEP_PIN);
        #endif
        
        counter_z -= current_block->step_event_count;
        count_position[Z_AXIS]+=count_direction[Z_AXIS];
        WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);
        
		#ifdef Z_DUAL_STEPPER_DRIVERS
          WRITE(Z2_STEP_PIN, INVERT_Z_STEP_PIN);
        #endif
      }

      #ifndef ADVANCE
        counter_e += current_block->steps_e; //E轴插补
        if (counter_e > 0) {
          WRITE_E_STEP(!INVERT_E_STEP_PIN);
          counter_e -= current_block->step_event_count;
          count_position[E_AXIS]+=count_direction[E_AXIS];
          WRITE_E_STEP(INVERT_E_STEP_PIN);
        }
      #endif //!ADVANCE
      step_events_completed += 1;  
      if(step_events_completed >= current_block->step_event_count) break;
    }
    // Calculare new timer value
    unsigned short timer;
    unsigned short step_rate;
    if (step_events_completed <= (unsigned long int)current_block->accelerate_until) { //匀加速阶段
      
      MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;
      
      // upper limit
      if(acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
			TIM_SetAutoreload(TIM2,timer);
 ////     OCR1A = timer;
      acceleration_time += timer;
			
      #ifdef ADVANCE
        for(int8_t i=0; i < step_loops; i++) {
          advance += advance_rate;
        }
        //if(advance > current_block->advance) advance = current_block->advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >>8;  
      #endif
    } 
    else if (step_events_completed > (unsigned long int)current_block->decelerate_after) {   
      MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);
      
      if(step_rate > acc_step_rate) { // Check step_rate stays positive
        step_rate = current_block->final_rate;
      }
      else {
        step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
      }

      // lower limit
      if(step_rate < current_block->final_rate)
        step_rate = current_block->final_rate;

      // step_rate to timer interval
      timer = calc_timer(step_rate);
			TIM_SetAutoreload(TIM2,timer);
////      OCR1A = timer;
      deceleration_time += timer;
      #ifdef ADVANCE
        for(int8_t i=0; i < step_loops; i++) {
          advance -= advance_rate;
        }
        if(advance < final_advance) advance = final_advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >>8;  
      #endif //ADVANCE
    }
    else {
						TIM_SetAutoreload(TIM2,OCR1A_nominal);
////      OCR1A = OCR1A_nominal;
      // ensure we're running at the correct step rate, even if we just came off an acceleration
      step_loops = step_loops_nominal;
    }

    // If current block is finished, reset pointer 
    if (step_events_completed >= current_block->step_event_count) { //插补执行完成
      current_block = NULL;
      plan_discard_current_block();
    }   
  } 
}
}

}
#endif
//ADVANCE是外扩2个挤出轴,这样就有3个挤出轴,可以做混色打印
#ifdef ADVANCE
  unsigned char old_OCR0A;
  // Timer interrupt for E. e_steps is set in the main routine;nono
  // Timer 0 is shared with millies
  ISR(TIMER0_COMPA_vect)
  {
    old_OCR0A += 52; // ~10kHz interrupt (250000 / 26 = 9615kHz)
    OCR0A = old_OCR0A;
    // Set E direction (Depends on E direction + advance)
    for(unsigned char i=0; i<4;i++) {
      if (e_steps[0] != 0) {
        WRITE(E0_STEP_PIN, INVERT_E_STEP_PIN);
        if (e_steps[0] < 0) {
          WRITE(E0_DIR_PIN, INVERT_E0_DIR);
          e_steps[0]++;
          WRITE(E0_STEP_PIN, !INVERT_E_STEP_PIN);
        } 
        else if (e_steps[0] > 0) {
          WRITE(E0_DIR_PIN, !INVERT_E0_DIR);
          e_steps[0]--;
          WRITE(E0_STEP_PIN, !INVERT_E_STEP_PIN);
        }
      }
 #if EXTRUDERS > 1
      if (e_steps[1] != 0) {
        WRITE(E1_STEP_PIN, INVERT_E_STEP_PIN);
        if (e_steps[1] < 0) {
          WRITE(E1_DIR_PIN, INVERT_E1_DIR);
          e_steps[1]++;
          WRITE(E1_STEP_PIN, !INVERT_E_STEP_PIN);
        } 
        else if (e_steps[1] > 0) {
          WRITE(E1_DIR_PIN, !INVERT_E1_DIR);
          e_steps[1]--;
          WRITE(E1_STEP_PIN, !INVERT_E_STEP_PIN);
        }
      }
 #endif
 #if EXTRUDERS > 2
      if (e_steps[2] != 0) {
        WRITE(E2_STEP_PIN, INVERT_E_STEP_PIN);
        if (e_steps[2] < 0) {
          WRITE(E2_DIR_PIN, INVERT_E2_DIR);
          e_steps[2]++;
          WRITE(E2_STEP_PIN, !INVERT_E_STEP_PIN);
        } 
        else if (e_steps[2] > 0) {
          WRITE(E2_DIR_PIN, !INVERT_E2_DIR);
          e_steps[2]--;
          WRITE(E2_STEP_PIN, !INVERT_E_STEP_PIN);
        }
      }
 #endif
    }
  }
#endif // ADVANCE

void Time2_Configure(uint16_t period)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	// Enable clock for TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	// TIM2 initialization for overflow every 500ms
	// Update Event (Hz) = timer_clock / ((TIM_Prescaler + 1) * (TIM_Period + 1))
	// Update Event (Hz) = 72MHz / ((3599 + 1) * (9999 + 1)) = 2Hz (0.5s)
	TIM_TimeBaseInitStruct.TIM_Prescaler = 35;
	TIM_TimeBaseInitStruct.TIM_Period = period;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	// Enable TIM2 interrupt
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	// Start TIM2
	//TIM_Cmd(TIM2, ENABLE);
	
	// Nested vectored interrupt settings
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStruct);
}
	
//函数名称：步进电机初始化函数
//输入参数：无
//输出参数：无
//备注：初始化脉冲发生定时器,初始化IO,初始化路径,速度计算数据
void st_init()
{
  ////digipot_init(); //Initialize Digipot Motor Current  ,接spi调点位进用了设置步进电机的电流
  ////microstep_init(); //Initialize Microstepping Pins ,用来设置步进电机的细分数
  //Initialize Dir Pins
 #if defined(X_DIR_PIN) 
    SET_OUTPUT(X_DIR_PIN);
  #endif
  #if defined(Y_DIR_PIN) 
    SET_OUTPUT(Y_DIR_PIN);
  #endif
  #if defined(Z_DIR_PIN) 
    SET_OUTPUT(Z_DIR_PIN);

    #if defined(Z_DUAL_STEPPER_DRIVERS) && defined(Z2_DIR_PIN) 
      SET_OUTPUT(Z2_DIR_PIN);
    #endif
  #endif
	
  #if defined(E0_DIR_PIN)
    SET_OUTPUT(E0_DIR_PIN);
  #endif
  #if defined(E1_DIR_PIN) 
    SET_OUTPUT(E1_DIR_PIN);
  #endif
  #if defined(E2_DIR_PIN) 
    SET_OUTPUT(E2_DIR_PIN);
  #endif

  //Initialize Enable Pins - steppers default to disabled.

  #if defined(X_ENABLE_PIN) 
    SET_OUTPUT(X_ENABLE_PIN);
    if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
  #endif
  #if defined(Y_ENABLE_PIN) 
    SET_OUTPUT(Y_ENABLE_PIN);
    if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
  #endif
  #if defined(Z_ENABLE_PIN) 
    SET_OUTPUT(Z_ENABLE_PIN);
    if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
    
    #if defined(Z_DUAL_STEPPER_DRIVERS) && defined(Z2_ENABLE_PIN) && (Z2_ENABLE_PIN > -1)
      SET_OUTPUT(Z2_ENABLE_PIN);
      if(!Z_ENABLE_ON) WRITE(Z2_ENABLE_PIN,HIGH);
    #endif
  #endif
  #if defined(E0_ENABLE_PIN) 
    SET_OUTPUT(E0_ENABLE_PIN);
    if(!E_ENABLE_ON) WRITE(E0_ENABLE_PIN,HIGH);
  #endif
  #if defined(E1_ENABLE_PIN) 
    SET_OUTPUT(E1_ENABLE_PIN);
    if(!E_ENABLE_ON) WRITE(E1_ENABLE_PIN,HIGH);
  #endif
  #if defined(E2_ENABLE_PIN) 
    SET_OUTPUT(E2_ENABLE_PIN);
    if(!E_ENABLE_ON) WRITE(E2_ENABLE_PIN,HIGH);
  #endif

  //endstops and pullups
	#ifdef ENDSTOPPULLUP_XMIN
    SET_INPUT(X_MIN_PIN); 
		#if defined(X_MIN_PIN) 
      WRITE(X_MIN_PIN,HIGH);
    #endif
  #endif
      
	#ifdef ENDSTOPPULLUP_YMIN    
    SET_INPUT(Y_MIN_PIN); 
		#if defined(Y_MIN_PIN) 
      WRITE(Y_MIN_PIN,HIGH);
    #endif
  #endif
  
	#ifdef ENDSTOPPULLUP_ZMIN
    SET_INPUT(Z_MIN_PIN); 
		#if defined(Z_MIN_PIN) 
      WRITE(Z_MIN_PIN,HIGH);
    #endif
  #endif
      
	#ifdef ENDSTOPPULLUP_XMAX		
    SET_INPUT(X_MAX_PIN); 
		#if defined(X_MAX_PIN) 
      WRITE(X_MAX_PIN,HIGH);
    #endif
  #endif
      
	#ifdef ENDSTOPPULLUP_YMAX
    SET_INPUT(Y_MAX_PIN); 
		#if defined(Y_MAX_PIN) 
      WRITE(Y_MAX_PIN,HIGH);
    #endif
  #endif
  
	#ifdef ENDSTOPPULLUP_ZMAX
    SET_INPUT(Z_MAX_PIN); 
		#if defined(Z_MAX_PIN) 
      WRITE(Z_MAX_PIN,HIGH);
    #endif
  #endif
 

  //Initialize Step Pins
  #if defined(X_STEP_PIN) 
    SET_OUTPUT(X_STEP_PIN);
    WRITE(X_STEP_PIN,INVERT_X_STEP_PIN);
    disable_x();
  #endif  
  #if defined(Y_STEP_PIN) 
    SET_OUTPUT(Y_STEP_PIN);
    WRITE(Y_STEP_PIN,INVERT_Y_STEP_PIN);
    disable_y();
  #endif  
  #if defined(Z_STEP_PIN) 
    SET_OUTPUT(Z_STEP_PIN);
    WRITE(Z_STEP_PIN,INVERT_Z_STEP_PIN);
    #if defined(Z_DUAL_STEPPER_DRIVERS) && defined(Z2_STEP_PIN) 
      SET_OUTPUT(Z2_STEP_PIN);
      WRITE(Z2_STEP_PIN,INVERT_Z_STEP_PIN);
    #endif
    disable_z();
  #endif  
  #if defined(E0_STEP_PIN) 
    SET_OUTPUT(E0_STEP_PIN);
    WRITE(E0_STEP_PIN,INVERT_E_STEP_PIN);
    disable_e0();
  #endif  
  #if defined(E1_STEP_PIN) 
    SET_OUTPUT(E1_STEP_PIN);
    WRITE(E1_STEP_PIN,INVERT_E_STEP_PIN);
    disable_e1();
  #endif  
  #if defined(E2_STEP_PIN) 
    SET_OUTPUT(E2_STEP_PIN);
    WRITE(E2_STEP_PIN,INVERT_E_STEP_PIN);
    disable_e2();
  #endif  

  // waveform generation = 0100 = CTC
////  TCCR1B &= ~(1<<WGM13);
////  TCCR1B |=  (1<<WGM12);
////  TCCR1A &= ~(1<<WGM11); 
////  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
////  TCCR1A &= ~(3<<COM1A0); 
////  TCCR1A &= ~(3<<COM1B0); 
  Time2_Configure(100); //定时器的配置,用来配置产生步进的脉冲输出
  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
////  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

 //// OCR1A = 0x4000;
 //// TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();  

  #ifdef ADVANCE
 //// #if defined(TCCR0A) && defined(WGM01)
  ////  TCCR0A &= ~(1<<WGM01);
  ////  TCCR0A &= ~(1<<WGM00);
////  #endif  
    e_steps[0] = 0;
    e_steps[1] = 0;
    e_steps[2] = 0;
  ////  TIMSK0 |= (1<<OCIE0A);
  #endif //ADVANCE
  
  enable_endstops(true); // Start with endstops active. After homing they can be disabled
  sei();
}


// Block until all buffered steps are executed
//等待缓存队列数执行完成
void st_synchronize()
{
    while( blocks_queued()|(current_block!=NULL)) {
		IWDG_Feed();
    manage_heater();
    manage_inactivity();
//		MySerialLcd.LCD_Run();
  }
}

//重置路径脉冲数
void st_set_position(const long &x, const long &y, const long &z, const long &e)
{
  CRITICAL_SECTION_START;
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

//重置Z轴高度
void st_set_z_position(const long &z)
{
	CRITICAL_SECTION_START;
	count_position[Z_AXIS] = z;
	CRITICAL_SECTION_END;
}


//重置E的脉冲数
void st_set_e_position(const long &e)
{
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}
//读出坐标轴的脉冲数
long st_get_position(uint8_t axis)
{
  long count_pos;
  CRITICAL_SECTION_START;
  count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

//读出坐标轴的脉冲数
long st_syn_position(uint8_t axis)
{
	long count_pos;
	CRITICAL_SECTION_START;
	position[axis] = count_position[axis];
	CRITICAL_SECTION_END;
	return count_pos;
}

//等待缓存数据执行完后解锁所有的电机
void finishAndDisableSteppers()
{
  st_synchronize(); 
  disable_x(); 
  disable_y(); 
  disable_z(); 
  disable_e0(); 
//  disable_e1(); 
//  disable_e2(); 
}

//关闭步进电机脉冲输出后，清空缓存区的数据,该操作会丢失脉冲坐标
void quickStop()
{
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while(blocks_queued())
    plan_discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

void digitalPotWrite(int address, int value) // From Arduino DigitalPotControl example
{
  #if defined(DIGIPOTSS_PIN)
//     WRITE(DIGIPOTSS_PIN,LOW); // take the SS pin low to select the chip
//     SPI.transfer(address); //  send in the address and value via SPI:
//     SPI.transfer(value);
//     WRITE(DIGIPOTSS_PIN,HIGH); // take the SS pin high to de-select the chip:
    //delay(10);
  #endif
}

void digipot_init() //Initialize Digipot Motor Current
{
  #if defined(DIGIPOTSS_PIN)
    const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;
    
//     SPI.begin(); 
// 		SET_OUTPUT(DIGIPOTSS_PIN);    
//     for(int i=0;i<=4;i++) 
//       digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
//       digipot_current(i,digipot_motor_current[i]);
  #endif
}

void digipot_current(uint8_t driver, int current)
{
  #if defined(DIGIPOTSS_PIN)
 ////   const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
 ////   digitalPotWrite(digipot_ch[driver], current);
  #endif
}

void microstep_init()
{
  #if defined(X_MS1_PIN) 
  const uint8_t microstep_modes[] = MICROSTEP_MODES;
  SET_OUTPUT(X_MS2_PIN);
  SET_OUTPUT(Y_MS2_PIN);
  SET_OUTPUT(Z_MS2_PIN);
  SET_OUTPUT(E0_MS2_PIN);
  SET_OUTPUT(E1_MS2_PIN);
  for(int i=0;i<=4;i++) microstep_mode(i,microstep_modes[i]);
  #endif
}

void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2)
{
  if(ms1 > -1) switch(driver)
  {
 /* case 0: WRITE( X_MS1_PIN,ms1); break;
    case 1: WRITE( Y_MS1_PIN,ms1); break;
    case 2: WRITE( Z_MS1_PIN,ms1); break;
    case 3: WRITE(E0_MS1_PIN,ms1); break;
    case 4: WRITE(E1_MS1_PIN,ms1); break;*/
  }
  if(ms2 > -1) switch(driver)
  {
/*  case 0: WRITE( X_MS2_PIN,ms2); break;
    case 1: WRITE( Y_MS2_PIN,ms2); break;
    case 2: WRITE( Z_MS2_PIN,ms2); break;
    case 3: WRITE(E0_MS2_PIN,ms2); break;
    case 4: WRITE(E1_MS2_PIN,ms2); break;*/
  }
}

void microstep_mode(uint8_t driver, uint8_t stepping_mode)
{
  switch(stepping_mode)
  {
    case 1: microstep_ms(driver,MICROSTEP1); break;
    case 2: microstep_ms(driver,MICROSTEP2); break;
    case 4: microstep_ms(driver,MICROSTEP4); break;
    case 8: microstep_ms(driver,MICROSTEP8); break;
    case 16: microstep_ms(driver,MICROSTEP16); break;
  }
}

void microstep_readings()
{
//     printf("MS1,MS2 Pins\n");
//       printf("X: ");
//       printf(   READ(X_MS1_PIN));
//       printf( READ(X_MS2_PIN));
//       printf("Y: ");
//       printf(   READ(Y_MS1_PIN));
//       printf( READ(Y_MS2_PIN));
//       printf("Z: ");
//       printf(   READ(Z_MS1_PIN));
//       printf( READ(Z_MS2_PIN));
//       printf("E0: ");
//       printf(   READ(E0_MS1_PIN));
//       printf( READ(E0_MS2_PIN));
//       printf("E1: ");
//       printf(   READ(E1_MS1_PIN));
//       printf( READ(E1_MS2_PIN));
}


