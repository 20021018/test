/*
  temperature.c - temperature control
  Part of Marlin
  
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
#include "Configuration.h"
#include "temperature.h"
#include "bsp_watchdog.h"
#include "bsp_pin.h"
#include "bsp_sysclk.h"
#include "bsp_adc.h"
#include "bsp_usart.h"
#include "language.h"

//解决多次重复定义，在头文件声明，源文件定义。 注释掉的是类型1的传感器，根据B值最新选择为9

//const short temptable_1[][2]  = {
//{       23*OVERSAMPLENR ,       300     },
//{       25*OVERSAMPLENR ,       295     },
//{       27*OVERSAMPLENR ,       290     },
//{       28*OVERSAMPLENR ,       285     },
//{       31*OVERSAMPLENR ,       280     },
//{       33*OVERSAMPLENR ,       275     },
//{       35*OVERSAMPLENR ,       270     },
//{       38*OVERSAMPLENR ,       265     },
//{       41*OVERSAMPLENR ,       260     },
//{       44*OVERSAMPLENR ,       255     },
//{       48*OVERSAMPLENR ,       250     },
//{       52*OVERSAMPLENR ,       245     },
//{       56*OVERSAMPLENR ,       240     },
//{       61*OVERSAMPLENR ,       235     },
//{       66*OVERSAMPLENR ,       230     },
//{       71*OVERSAMPLENR ,       225     },
//{       78*OVERSAMPLENR ,       220     },
//{       84*OVERSAMPLENR ,       215     },
//{       92*OVERSAMPLENR ,       210     },
//{       100*OVERSAMPLENR        ,       205     },
//{       109*OVERSAMPLENR        ,       200     },
//{       120*OVERSAMPLENR        ,       195     },
//{       131*OVERSAMPLENR        ,       190     },
//{       143*OVERSAMPLENR        ,       185     },
//{       156*OVERSAMPLENR        ,       180     },
//{       171*OVERSAMPLENR        ,       175     },
//{       187*OVERSAMPLENR        ,       170     },
//{       205*OVERSAMPLENR        ,       165     },
//{       224*OVERSAMPLENR        ,       160     },
//{       245*OVERSAMPLENR        ,       155     },
//{       268*OVERSAMPLENR        ,       150     },
//{       293*OVERSAMPLENR        ,       145     },
//{       320*OVERSAMPLENR        ,       140     },
//{       348*OVERSAMPLENR        ,       135     },
//{       379*OVERSAMPLENR        ,       130     },
//{       411*OVERSAMPLENR        ,       125     },
//{       445*OVERSAMPLENR        ,       120     },
//{       480*OVERSAMPLENR        ,       115     },
//{       516*OVERSAMPLENR        ,       110     },
//{       553*OVERSAMPLENR        ,       105     },
//{       591*OVERSAMPLENR        ,       100     },
//{       628*OVERSAMPLENR        ,       95      },
//{       665*OVERSAMPLENR        ,       90      },
//{       702*OVERSAMPLENR        ,       85      },
//{       737*OVERSAMPLENR        ,       80      },
//{       770*OVERSAMPLENR        ,       75      },
//{       801*OVERSAMPLENR        ,       70      },
//{       830*OVERSAMPLENR        ,       65      },
//{       857*OVERSAMPLENR        ,       60      },
//{       881*OVERSAMPLENR        ,       55      },
//{       903*OVERSAMPLENR        ,       50      },
//{       922*OVERSAMPLENR        ,       45      },
//{       939*OVERSAMPLENR        ,       40      },
//{       954*OVERSAMPLENR        ,       35      },
//{       966*OVERSAMPLENR        ,       30      },
//{       977*OVERSAMPLENR        ,       25      },
//{       985*OVERSAMPLENR        ,       20      },
//{       993*OVERSAMPLENR        ,       15      },
//{       999*OVERSAMPLENR        ,       10      },
//{       1004*OVERSAMPLENR       ,       5       },
//{       1008*OVERSAMPLENR       ,       0       } //safety
//};
const short temptable_9[][2]  = {
    {1*OVERSAMPLENR, 936},
    {36*OVERSAMPLENR, 300},
    {71*OVERSAMPLENR, 246},
    {106*OVERSAMPLENR, 218},
    {141*OVERSAMPLENR, 199},
    {176*OVERSAMPLENR, 185},
    {211*OVERSAMPLENR, 173},
    {246*OVERSAMPLENR, 163},
    {281*OVERSAMPLENR, 155},
    {316*OVERSAMPLENR, 147},
    {351*OVERSAMPLENR, 140},
    {386*OVERSAMPLENR, 134},
    {421*OVERSAMPLENR, 128},
    {456*OVERSAMPLENR, 122},
    {491*OVERSAMPLENR, 117},
    {526*OVERSAMPLENR, 112},
    {561*OVERSAMPLENR, 107},
    {596*OVERSAMPLENR, 102},
    {631*OVERSAMPLENR, 97},
    {666*OVERSAMPLENR, 92},
    {701*OVERSAMPLENR, 87},
    {736*OVERSAMPLENR, 81},
    {771*OVERSAMPLENR, 76},
    {806*OVERSAMPLENR, 70},
    {841*OVERSAMPLENR, 63},
    {876*OVERSAMPLENR, 56},
    {911*OVERSAMPLENR, 48},
    {946*OVERSAMPLENR, 38},
    {981*OVERSAMPLENR, 23},
    {1005*OVERSAMPLENR, 5},
    {1016*OVERSAMPLENR, 0}
};
//===========================================================================
//=============================public variables============================
//===========================================================================
int target_temperature[EXTRUDERS] = { 0};
int target_temperature_bed = 0;
int current_temperature_raw[EXTRUDERS] = { 0 };
float current_temperature[EXTRUDERS] = { 0 };
int current_temperature_bed_raw = 0;
float current_temperature_bed = 0;

#ifdef PIDTEMP
  float Kp=DEFAULT_Kp;
  float Ki=(DEFAULT_Ki*PID_dT);
  float Kd=(DEFAULT_Kd/PID_dT);
  #ifdef PID_ADD_EXTRUSION_RATE
    float Kc=DEFAULT_Kc;
  #endif
#endif //PIDTEMP

#ifdef PIDTEMPBED
  float bedKp=DEFAULT_bedKp;
  float bedKi=(DEFAULT_bedKi*PID_dT);
  float bedKd=(DEFAULT_bedKd/PID_dT);
#endif //PIDTEMPBED
  
  
//===========================================================================
//=============================private variables============================
//===========================================================================
static volatile bool temp_meas_ready = false;

#ifdef PIDTEMP
  //static cannot be external:
  static float temp_iState[EXTRUDERS] = { 0 };
  static float temp_dState[EXTRUDERS] = { 0 };
  static float pTerm[EXTRUDERS];
  static float iTerm[EXTRUDERS];
  static float dTerm[EXTRUDERS];
  //int output;
  static float pid_error[EXTRUDERS];
  static float temp_iState_min[EXTRUDERS];
  static float temp_iState_max[EXTRUDERS];
  // static float pid_input[EXTRUDERS];
  // static float pid_output[EXTRUDERS];
  static bool pid_reset[EXTRUDERS];
#endif //PIDTEMP
#ifdef PIDTEMPBED
  //static cannot be external:
  static float temp_iState_bed = { 0 };
  static float temp_dState_bed = { 0 };
  static float pTerm_bed;
  static float iTerm_bed;
  static float dTerm_bed;
  //int output;
  static float pid_error_bed;
  static float temp_iState_min_bed;
  static float temp_iState_max_bed;
#else //PIDTEMPBED
	static unsigned long  previous_millis_bed_heater;
#endif //PIDTEMPBED
  static unsigned char soft_pwm[EXTRUDERS];
  static unsigned char soft_pwm_bed;
#ifdef FAN_SOFT_PWM
  static unsigned char soft_pwm_fan;
#endif

  
#if EXTRUDERS > 3
# error Unsupported number of extruders
#elif EXTRUDERS > 2
# define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2, v3 }
#elif EXTRUDERS > 1
# define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2 }
#else
# define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1 }
#endif

// Init min and max temp with extreme values to prevent false errors during startup
static int minttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP );
static int maxttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP );
static int minttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 0, 0, 0 );
static int maxttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 16383, 16383, 16383 );
static int bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP; /* No bed mintemp error implemented?!? */
#ifdef BED_MAXTEMP
static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif
static void *heater_ttbl_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( (void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE, (void *)HEATER_2_TEMPTABLE );
static uint8_t heater_ttbllen_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN );

float analog2temp(int raw, uint8_t e);
float analog2tempBed(int raw);
static void updateTemperaturesFromRawValues();

#ifdef WATCH_TEMP_PERIOD
int watch_start_temp[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0,0,0);
unsigned long watchmillis[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0,0,0);
#endif //WATCH_TEMP_PERIOD

//===========================================================================
//=============================   functions      ============================
//===========================================================================

void PID_autotune(float temp, int extruder, int ncycles)
{
  float input = 0.0;
  int cycles=0;
  bool heating = true;

  unsigned long temp_millis = millis();
  unsigned long t1=temp_millis;
  unsigned long t2=temp_millis;
  long t_high = 0;
  long t_low = 0;

  long bias, d;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float max = 0, min = 10000;

	if ((extruder > EXTRUDERS)
  ////#if (TEMP_BED_PIN <= -1)
	////	||(extruder < 0)
	////#endif
	){
  	printf("PID Autotune failed. Bad extruder number.");
  	return;
	}
  printf("PID Autotune start");

  disable_heater(); // switch off all heaters.
	if (extruder<0)
	{
	 	soft_pwm_bed = (MAX_BED_POWER)/2;
		bias = d = (MAX_BED_POWER)/2;
  }
	else
	{
	  soft_pwm[extruder] = (PID_MAX)/2;
		bias = d = (PID_MAX)/2;
  }

 for(;;) {

    if(temp_meas_ready == true) { // temp sample ready
      updateTemperaturesFromRawValues();

      input = (extruder<0)?current_temperature_bed:current_temperature[extruder];

      max=max(max,input);
      min=min(min,input);
      if(heating == true && input > temp) {
        if(millis() - t2 > 5000) { 
          heating=false;
					if (extruder<0)
						soft_pwm_bed = (bias - d) >> 1;
					else
						soft_pwm[extruder] = (bias - d) >> 1;
          t1=millis();
          t_high=t1 - t2;
          max=temp;
        }
      }
      if(heating == false && input < temp) {
        if(millis() - t1 > 5000) {
          heating=true;
          t2=millis();
          t_low=t2 - t1;
          if(cycles > 0) {
            bias += (d*(t_high - t_low))/(t_low + t_high);
            bias = constrain(bias, 20 ,(extruder<0?(MAX_BED_POWER):(PID_MAX))-20);
            if(bias > (extruder<0?(MAX_BED_POWER):(PID_MAX))/2) d = (extruder<0?(MAX_BED_POWER):(PID_MAX)) - 1 - bias;
            else d = bias;

            printf(" bias: %ld \n\r",bias); 
            printf(" d: %ld \n\r", d);
            printf(" min: %f \n\r",min); 
            printf(" max: %f \n\r",max); 
            if(cycles > 2) {
              Ku = (4.0*d)/(3.14159*(max-min)/2.0);
              Tu = ((float)(t_low + t_high)/1000.0);
              printf(" Ku: %f \n\r",Ku); 
              printf(" Tu: %f \n\r",Tu); 
              Kp = 0.6*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/8;
              printf(" Clasic PID \n\r");
              printf(" Kp: %f \n\r",Kp);
              printf(" Ki: %f \n\r",Ki);
              printf(" Kd: %f \n\r",Kd);
              /*
              Kp = 0.33*Ku;
              Ki = Kp/Tu;
              Kd = Kp*Tu/3;
              printf(" Some overshoot \n\r")
              printf(" Kp: %f",Kp); 
              printf(" Ki: %f",Ki); 
              printf(" Kd: %f",Kd); 
              Kp = 0.2*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/3;
              printf(" No overshoot ")
							printf(" Kp: %f",Kp); 
              printf(" Ki: %f",Ki); 
              printf(" Kd: %f",Kd); 
              */
            }
          }
					if (extruder<0)
						soft_pwm_bed = (bias + d) >> 1;
					else
						soft_pwm[extruder] = (bias + d) >> 1;
          cycles++;
          min=temp;
        }
      } 
    }
    if(input > (temp + 20)) {
      printf("PID Autotune failed! Temperature to high\n\r");
      return;
    }
    if(millis() - temp_millis > 2000) {
			int p;
			if (extruder<0){
	      p=soft_pwm_bed;       
	      printf("ok B:");
			}else{
	      p=soft_pwm[extruder];       
	      printf("ok T:");
			}
			
      printf("%f",input);   
      printf(" @:");
      printf("%d",p);       

      temp_millis = millis();
    }
    if(((millis() - t1) + (millis() - t2)) > (10L*60L*1000L*2L)) {
      printf("PID Autotune failed! timeout\n\r");
      return;
    }
    if(cycles > ncycles) {
      printf("PID Autotune finished ! Place the Kp, Ki and Kd constants in the configuration.h\n\r");
      return;
    }
  }
}

void updatePID()
{
#ifdef PIDTEMP
  for(int e = 0; e < EXTRUDERS; e++) { 
     temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;  
  }
#endif
#ifdef PIDTEMPBED
  temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;  
#endif
}
  
int getHeaterPower(int heater) {
	if (heater<0)
		return soft_pwm_bed;
  return soft_pwm[heater];
}

  // update extruder auto fan states


void manage_heater()
{
  float pid_input;
  float pid_output;
	//ad采样完成后，才能更新温度
  if(temp_meas_ready != true)   //better readability
    return; 

  updateTemperaturesFromRawValues();  //温度计算更新

	//挤出机温度的处理
  for(int e = 0; e < EXTRUDERS; e++) 
  {
/******************PID的处理过程**********************/
  #ifdef PIDTEMP
    pid_input = current_temperature[e]; //获取当前温度

    #ifndef PID_OPENLOOP
        pid_error[e] = target_temperature[e] - pid_input; //误差值
        if(pid_error[e] > PID_FUNCTIONAL_RANGE) { //正误差超过PID_FUNCTIONAL_RANGE
          pid_output = BANG_MAX;
          pid_reset[e] = true;
        }
        else if(pid_error[e] < -PID_FUNCTIONAL_RANGE || target_temperature[e] == 0) { //如果负误差超过PID_FUNCTIONAL_RANGE,或者没有读到温度
          pid_output = 0;
          pid_reset[e] = true;//为什么程序进入这里
        }
        else {
          if(pid_reset[e] == true) {
            temp_iState[e] = 0.0;
            pid_reset[e] = false;
          }
          pTerm[e] = Kp * pid_error[e]; //P*误差值
          temp_iState[e] += pid_error[e]; //累计误差
          temp_iState[e] = constrain(temp_iState[e], temp_iState_min[e], temp_iState_max[e]);
          iTerm[e] = Ki * temp_iState[e]; //I*累计误差值

          //K1 defined in Configuration.h in the PID settings
          #define K2 (1.0-K1)
          dTerm[e] = (Kd * (pid_input - temp_dState[e]))*K2 + (K1 * dTerm[e]); 
          temp_dState[e] = pid_input;

          pid_output = constrain(pTerm[e] + iTerm[e] - dTerm[e], 0, PID_MAX);  //根据PID计算输出量
        }
				
    #else 
          pid_output = constrain(target_temperature[e], 0, PID_MAX);
    #endif //PID_OPENLOOP
    #ifdef PID_DEBUG
    printf(" PIDDEBUG ");
    printf("%f",e);
    printf(": Input ");
    printf("%d",pid_input);
    printf(" Output ");
    printf("%d",pid_output);
    printf(" pTerm ");
    printf("%f",pTerm[e]);
    printf(" iTerm ");
    printf("%f",iTerm[e]);
    printf(" dTerm ");
    printf("%f",dTerm[e]);  
		printf("\n\r");
    #endif //PID_DEBUG
  #else /* PID off */
    pid_output = 0;
    if(current_temperature[e] < target_temperature[e]) { //当前温度小于设定温度,全力加热
      pid_output = PID_MAX;
    }
  #endif

/********************根据PID输出值确定软PWM的占空比来调节温度***********************/
    // Check if temperature is within the correct range
    if((current_temperature[e] > minttemp[e]) && (current_temperature[e] < maxttemp[e])) 
    {
      soft_pwm[e] = (int)pid_output >> 1;  //根据输出量设置软PWM的占空比,来调节加热时间
		//	printf("soft_pwm=%d",soft_pwm[0]);
    }
    else {
      soft_pwm[e] = 0;
			printf("current_temperature error!");
    }
		/************加热升温检查***********/
    #ifdef WATCH_TEMP_PERIOD
    if(watchmillis[e] && millis() - watchmillis[e] > WATCH_TEMP_PERIOD)
    {
        if(degHotend(e) < watch_start_temp[e] + WATCH_TEMP_INCREASE)
        {
            setTargetHotend(0, e);
        
            printf(MSG_START);
            printf("Heating failed!\n\r");
						kill();
        }else{
            watchmillis[e] = 0;
        }
    }
    #endif
} // End extruder for loop

     
  
  #ifndef PIDTEMPBED
  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
  previous_millis_bed_heater = millis();
  #endif

	//热床温度的处理
  #if TEMP_SENSOR_BED != 0
  #ifdef PIDTEMPBED
    pid_input = current_temperature_bed;

    #ifndef PID_OPENLOOP
		  pid_error_bed = target_temperature_bed - pid_input;
		  pTerm_bed = bedKp * pid_error_bed;
		  temp_iState_bed += pid_error_bed;
		  temp_iState_bed = constrain(temp_iState_bed, temp_iState_min_bed, temp_iState_max_bed);
		  iTerm_bed = bedKi * temp_iState_bed;

		  //K1 defined in Configuration.h in the PID settings
		  #define K2 (1.0-K1)
		  dTerm_bed= (bedKd * (pid_input - temp_dState_bed))*K2 + (K1 * dTerm_bed);
		  temp_dState_bed = pid_input;

		  pid_output = constrain(pTerm_bed + iTerm_bed - dTerm_bed, 0, MAX_BED_POWER);

    #else 
      pid_output = constrain(target_temperature_bed, 0, MAX_BED_POWER);
    #endif //PID_OPENLOOP

	  if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP)) 
	  {
	    soft_pwm_bed = (int)pid_output >> 1;
	  }
	  else {
	    soft_pwm_bed = 0;
	  }

    #elif !defined(BED_LIMIT_SWITCHING)
      // Check if temperature is within the correct range
      if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP))
      {
        if(current_temperature_bed >= target_temperature_bed)
        {
          soft_pwm_bed = 0;
        }
        else 
        {
          soft_pwm_bed = MAX_BED_POWER>>1;
        }
      }
      else
      {
        soft_pwm_bed = 0;
        WRITE(HEATER_BED_PIN,HIGH);
      }
    #else //#ifdef BED_LIMIT_SWITCHING
      // Check if temperature is within the correct band
      if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP))
      {
        if(current_temperature_bed > target_temperature_bed + BED_HYSTERESIS)
        {
          soft_pwm_bed = 0;
        }
        else if(current_temperature_bed <= target_temperature_bed - BED_HYSTERESIS)
        {
          soft_pwm_bed = MAX_BED_POWER>>1;
        }
      }
      else
      {
        soft_pwm_bed = 0;
        WRITE(HEATER_BED_PIN,HIGH);
      }
    #endif
  #endif
}

////#define PGM_RD_W(x)   (short)pgm_read_word(&x)
#define PGM_RD_W(x)   x
// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
float analog2temp(int raw, uint8_t e) {
  if(e >= EXTRUDERS)
  {
      printf(MSG_ERR);
      printf("%d\n\r",(int)e);
      printf(" - Invalid extruder number !\n\r");
      kill();
  } 
  #ifdef HEATER_0_USES_MAX6675
    if (e == 0) //6675的数值除于4就是温度值
    {
      return 0.25 * raw;
    }
  #endif

  if(heater_ttbl_map[e] != NULL)
  {
    float celsius = 0;
    uint8_t i;
    short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);

    for (i=1; i<heater_ttbllen_map[e]; i++)
    {
      if (PGM_RD_W((*tt)[i][0]) > raw)
      {
        celsius = PGM_RD_W((*tt)[i-1][1]) + 
          (raw - PGM_RD_W((*tt)[i-1][0])) * 
          (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i-1][1])) /
          (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) celsius = PGM_RD_W((*tt)[i-1][1]);

    return celsius;
  }
  return ((raw * ((5.0 * 100.0) / 4096.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
float analog2tempBed(int raw) {
  #ifdef BED_USES_THERMISTOR
    float celsius = 0;
    char i;

    for (i=1; i<BEDTEMPTABLE_LEN; i++)
    {
      if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw)
      {
        celsius  = PGM_RD_W(BEDTEMPTABLE[i-1][1]) + 
          (raw - PGM_RD_W(BEDTEMPTABLE[i-1][0])) * 
          (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i-1][1])) /
          (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) celsius = PGM_RD_W(BEDTEMPTABLE[i-1][1]);

    return celsius;
  #elif defined BED_USES_AD595
    return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
  #else
    return 0;
  #endif
}

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues()
{
    for(uint8_t e=0;e<EXTRUDERS;e++)
    {
        current_temperature[e] = analog2temp(current_temperature_raw[e], e); //最多3个挤出机的温度
    }
    current_temperature_bed = analog2tempBed(current_temperature_bed_raw); //热床的温度
		
    CRITICAL_SECTION_START;
    temp_meas_ready = false;
    CRITICAL_SECTION_END;
}



void tp_init()
{
	// Finish init of mult extruder arrays 
  for(int e = 0; e < EXTRUDERS; e++) {
    // populate with the first value 
    maxttemp[e] = maxttemp[0];
#ifdef PIDTEMP
    temp_iState_min[e] = 0.0;
    temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;
#endif //PIDTEMP
#ifdef PIDTEMPBED
    temp_iState_min_bed = 0.0;
    temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;
#endif //PIDTEMPBED
  }

  #if defined(HEATER_0_PIN)  
    SET_OUTPUT(HEATER_0_PIN);
  #endif  
  #if defined(HEATER_1_PIN)
    SET_OUTPUT(HEATER_1_PIN);
  #endif  
  #if defined(HEATER_2_PIN) 
    SET_OUTPUT(HEATER_2_PIN);
  #endif  
  #if defined(HEATER_BED_PIN) 
    SET_OUTPUT(HEATER_BED_PIN);
  #endif  
  #if defined(FAN_PIN) 
    SET_OUTPUT(FAN_PIN);
    #ifdef FAST_PWM_FAN
    setPwmFrequency(FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
    #ifdef FAN_SOFT_PWM
		soft_pwm_fan=(unsigned char)fanSpeed;
	#endif
  #endif  

  #ifdef HEATER_0_USES_MAX6675
    #ifndef SDSUPPORT
      SET_OUTPUT(MAX_SCK_PIN);
      WRITE(MAX_SCK_PIN,0);
    
      SET_OUTPUT(MAX_MOSI_PIN);
      WRITE(MAX_MOSI_PIN,1);
    
      SET_INPUT(MAX_MISO_PIN);
      WRITE(MAX_MISO_PIN,1);
    #endif
    
    SET_OUTPUT(MAX6675_SS);
    WRITE(MAX6675_SS,1);
  #endif

	// Set analog inputs
  #if defined(TEMP_0_PIN) 
			MODE(TEMP_0_PIN,GPIO_Mode_AIN);
  #endif
  #if defined(TEMP_1_PIN) 
		MODE(TEMP_1_PIN,GPIO_Mode_AIN);
  #endif
  #if defined(TEMP_2_PIN) 
		MODE(TEMP_2_PIN,GPIO_Mode_AIN);
  #endif
  #if defined(TEMP_BED_PIN) 
		MODE(TEMP_BED_PIN,GPIO_Mode_AIN);
  #endif
  
  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
		timer3_init(); //定时器3产生1ms的中断，用来测量控制温度
	
#ifdef HEATER_0_MINTEMP
  minttemp[0] = HEATER_0_MINTEMP;
  while(analog2temp(minttemp_raw[0], 0) < HEATER_0_MINTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    minttemp_raw[0] += OVERSAMPLENR;
#else
    minttemp_raw[0] -= OVERSAMPLENR;
#endif
  }
#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP
  maxttemp[0] = HEATER_0_MAXTEMP;
  while(analog2temp(maxttemp_raw[0], 0) > HEATER_0_MAXTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    maxttemp_raw[0] -= OVERSAMPLENR;
#else
    maxttemp_raw[0] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP

#if (EXTRUDERS > 1) && defined(HEATER_1_MINTEMP)
  minttemp[1] = HEATER_1_MINTEMP;
  while(analog2temp(minttemp_raw[1], 1) < HEATER_1_MINTEMP) {
#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
    minttemp_raw[1] += OVERSAMPLENR;
#else
    minttemp_raw[1] -= OVERSAMPLENR;
#endif
  }
#endif // MINTEMP 1
#if (EXTRUDERS > 1) && defined(HEATER_1_MAXTEMP)
  maxttemp[1] = HEATER_1_MAXTEMP;
  while(analog2temp(maxttemp_raw[1], 1) > HEATER_1_MAXTEMP) {
#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
    maxttemp_raw[1] -= OVERSAMPLENR;
#else
    maxttemp_raw[1] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP 1

#if (EXTRUDERS > 2) && defined(HEATER_2_MINTEMP)
  minttemp[2] = HEATER_2_MINTEMP;
  while(analog2temp(minttemp_raw[2], 2) < HEATER_2_MINTEMP) {
#if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
    minttemp_raw[2] += OVERSAMPLENR;
#else
    minttemp_raw[2] -= OVERSAMPLENR;
#endif
  }
#endif //MINTEMP 2
#if (EXTRUDERS > 2) && defined(HEATER_2_MAXTEMP)
  maxttemp[2] = HEATER_2_MAXTEMP;
  while(analog2temp(maxttemp_raw[2], 2) > HEATER_2_MAXTEMP) {
#if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
    maxttemp_raw[2] -= OVERSAMPLENR;
#else
    maxttemp_raw[2] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP 2

#ifdef BED_MINTEMP
  /* No bed MINTEMP error implemented?!? */ 
  while(analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_minttemp_raw += OVERSAMPLENR;
#else
    bed_minttemp_raw -= OVERSAMPLENR;
#endif
  }
  
#endif //BED_MINTEMP
#ifdef BED_MAXTEMP
  while(analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_maxttemp_raw -= OVERSAMPLENR;
#else
    bed_maxttemp_raw += OVERSAMPLENR;
#endif
  }
#endif //BED_MAXTEMP
}




void setWatch() 
{  
#ifdef WATCH_TEMP_PERIOD
  for (int e = 0; e < EXTRUDERS; e++)
  {
    if(degHotend(e) < degTargetHotend(e) - (WATCH_TEMP_INCREASE * 2))
    {
      watch_start_temp[e] = degHotend(e);
      watchmillis[e] = millis();
    } 
  }
#endif 
}


void disable_heater()
{
  for(int i=0;i<EXTRUDERS;i++)
    setTargetHotend(0,i);
  setTargetBed(0);
  #if defined(TEMP_0_PIN) 
  target_temperature[0]=0;
  soft_pwm[0]=0;
   #if defined(HEATER_0_PIN) 
     WRITE(HEATER_0_PIN,!TEMP_HEAT_VALID);
   #endif
  #endif
     
  #if defined(TEMP_1_PIN) 
    target_temperature[1]=0;
    soft_pwm[1]=0;
    #if defined(HEATER_1_PIN)
      WRITE(HEATER_1_PIN,!TEMP_HEAT_VALID);
    #endif
  #endif
      
  #if defined(TEMP_2_PIN) 
    target_temperature[2]=0;
    soft_pwm[2]=0;
    #if defined(HEATER_2_PIN) 
      WRITE(HEATER_2_PIN,!TEMP_HEAT_VALID);
    #endif
  #endif 

  #if defined(TEMP_BED_PIN) 
    target_temperature_bed=0;
    soft_pwm_bed=0;
    #if defined(HEATER_BED_PIN)
      WRITE(HEATER_BED_PIN,!TEMP_HEAT_VALID);
    #endif
  #endif 
}

void max_temp_error(uint8_t e) {
  disable_heater();
  if(IsStopped() == false) {
    printf(MSG_ERR);
    printf("%d\n\r",(int)e);
    printf(": Extruder switched off. MAXTEMP triggered !\n\r");
////    LCD_ALERTMESSAGEPGM("Err: MAXTEMP");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop();
  #endif
}

void min_temp_error(uint8_t e) {
  disable_heater();
  if(IsStopped() == false) {
    printf(MSG_ERR);
    printf("%d\n\r",(int)e);
    printf(": Extruder switched off. MINTEMP triggered !\n\r");
////    LCD_ALERTMESSAGEPGM("Err: MINTEMP");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop();
  #endif
}

void bed_max_temp_error(void) {
#ifdef HEATER_BED_PIN 
  WRITE(HEATER_BED_PIN, Bit_SET);
#endif
  if(IsStopped() == false) {
    printf(MSG_ERR);
    printf("Temperature heated bed switched off. MAXTEMP triggered !!!\n\r");
////    LCD_ALERTMESSAGEPGM("Err: MAXTEMP BED");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop();
  #endif
}

#ifdef HEATER_0_USES_MAX6675
#define MAX6675_HEAT_INTERVAL 250
long max6675_previous_millis = -HEAT_INTERVAL;
int max6675_temp = 2000;


int read_max6675()
{
  if (millis() - max6675_previous_millis < MAX6675_HEAT_INTERVAL) 
    return max6675_temp;
  
  max6675_previous_millis = millis();
  max6675_temp = 0;  
  // enable TT_MAX6675
  WRITE(MAX6675_SS, 0);

  return max6675_temp;
}
#endif



// Timer 0 is shared with millies


//定时器1ms中断,1:测量温度值,2:通过PID输出PWM控制温度, 3蜂鸣器的驱动
#ifdef __cplusplus
extern "C" {
#endif		  
void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM3_IRQHandler(void)
{

//  printf("for test\r\n");
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
			 //清除TIM2的中断待处理位
		TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);
		//these variables are only accesible from the ISR, but static, so they don't loose their value
		
		static unsigned char temp_count = 0;
		static unsigned long raw_temp_0_value = 0;
		static unsigned long raw_temp_1_value = 0;
		static unsigned long raw_temp_2_value = 0;
		static unsigned long raw_temp_bed_value = 0;
		static unsigned char temp_state = 0;
		static unsigned char pwm_count = 1; //用来软PWM计数，0-255
		static unsigned char soft_pwm_0;  //挤出1的PWM占空比0-255之间

		
		#if EXTRUDERS > 1
		static unsigned char soft_pwm_1; //挤出2的PWM占空比0-255之间
		#endif
		#if EXTRUDERS > 2
		static unsigned char soft_pwm_2; //挤出3的PWM占空比0-255之间
		#endif
		#ifdef HEATER_BED_PIN 
		static unsigned char soft_pwm_b; //热床PWM占空比0-255之间
		#endif
		
		/***************模拟PWM,用来挤出机加热和热床*******************/
		//起始值 PWM为0的时候的，让输出有效，等到sotf_pwm小于pwm_count的时候，输出关掉，这样就能产生0--soft_pwm的占空比的模拟PWM
		//温度的控制，就是在main中manage_heater()函数中通过PID或者误差计算出soft_pwm[]的数据，来输出软pwm来调整温度的
		if(pwm_count == 0){
			soft_pwm_0 = soft_pwm[0]; //1个挤出机
			if(soft_pwm_0 > 0)
			{
//				WRITE(HEATER_0_PIN,!LOW);//原来为low
//				WRITE(HEATER_0_PIN,!HIGH);
//				WRITE(HEATER_0_PIN,!LOW);

                WRITE(HEATER_0_PIN,TEMP_HEAT_VALID);
                WRITE(HEATER_0_PIN,!TEMP_HEAT_VALID);
                WRITE(HEATER_0_PIN,TEMP_HEAT_VALID);

				WRITE(LED_PIN,LOW);
				WRITE(LED_PIN,HIGH);
				WRITE(LED_PIN,LOW);
			}
			#if EXTRUDERS > 1  //2个挤出机
			soft_pwm_1 = soft_pwm[1];
			if(soft_pwm_1 > 0) WRITE(HEATER_1_PIN,TEMP_HEAT_VALID);
			#endif
			#if EXTRUDERS > 2 //3个挤出机
			soft_pwm_2 = soft_pwm[2];
			if(soft_pwm_2 > 0) WRITE(HEATER_2_PIN,TEMP_HEAT_VALID);
			#endif
			#if defined(HEATER_BED_PIN) //热床
			soft_pwm_b = soft_pwm_bed;
			if(soft_pwm_b > 0) WRITE(HEATER_BED_PIN,TEMP_HEAT_VALID);
			#endif
			#ifdef FAN_SOFT_PWM //风扇
			soft_pwm_fan =(unsigned char) fanSpeed;
			if(soft_pwm_fan > 0) WRITE(FAN_PIN,TEMP_HEAT_VALID);
			#endif
		}
		//soft_pwm是设定的占空比,在0---soft_pwm_0的区间内,输出低电平,soft_pwm_0---0xff区间输出高电平
		if(soft_pwm_0 <= pwm_count)	WRITE(HEATER_0_PIN,!TEMP_HEAT_VALID);
		#if EXTRUDERS > 1
		if(soft_pwm_1 <= pwm_count) WRITE(HEATER_1_PIN,!TEMP_HEAT_VALID);
		#endif
		#if EXTRUDERS > 2
		if(soft_pwm_2 <= pwm_count) WRITE(HEATER_2_PIN,!TEMP_HEAT_VALID);
		#endif
		#if defined(HEATER_BED_PIN)
		if(soft_pwm_b <= pwm_count) WRITE(HEATER_BED_PIN,!TEMP_HEAT_VALID);
		#endif
		#ifdef FAN_SOFT_PWM
		if(soft_pwm_fan <= pwm_count) WRITE(FAN_PIN,!TEMP_HEAT_VALID);
		#endif
		pwm_count++;
		pwm_count &= 0xFf; //软件模拟PWM,最大值是255
		
		
	/*************温度的读出过程**************/
		//温度是temp_state循环，先通过swtich()选择温度通道，然后通过AD读出该AD通道的数据保存起来，第一路挤出，热床，第二路挤出，第三路挤出，一个4组数据,每组累加16次，
		//数据保存进current_temperature_raw，最后通过main里面调用函数  updateTemperaturesFromRawValues();把AD值转成温度的

		switch(temp_state) {
			case 0: // 挤出机0测温
				#if defined(TEMP_0_PIN) 
					Adc1_Init(TEMP_0_CHANNEL);// Start conversion
				#endif
				temp_state = 1;
				break;
			case 1: //读挤出机ADC的数据,数据是12位,只要前10位为了和以前的AVR匹配
				#if defined(TEMP_0_PIN)
					raw_temp_0_value += (ADC_GetConversionValue(ADC1)>>2);  //ADC的数值是累加的,因为stm32的AD是12位的，以前AD是10位的，所有去掉了最后两位数据
				     printf( "%04d\r\n", ADC_GetConversionValue(ADC1));
				#endif
				#ifdef HEATER_0_USES_MAX6675 // TODO remove the blocking
					raw_temp_0_value = read_max6675(); //如果是max6675就直接读程序温度
				#endif
				temp_state = 2;
				break;
			case 2: //热床的温度
				#if defined(TEMP_BED_PIN)
					Adc1_Init(TEMP_BED_CHANNEL);
				#endif
				temp_state = 3;
				break;
			case 3: // Measure TEMP_BED
				#if defined(TEMP_BED_PIN)
					raw_temp_bed_value += (ADC_GetConversionValue(ADC1)>>2);//累加热床的温度
				#endif
				temp_state = 4;
				break;
			case 4: //挤出机1的温度
				#if defined(TEMP_1_PIN)
					Adc1_Init(ADC_Channel_9);
				#endif
				temp_state = 5;
				break;
			case 5: // Measure TEMP_1
				#if defined(TEMP_1_PIN)
					raw_temp_1_value +=  (ADC_GetConversionValue(ADC1)>>2);
				#endif
				temp_state = 6;
				break;
			case 6: //挤出机2的温度
				#if defined(TEMP_2_PIN)
				Adc1_Init(ADC_Channel_8);
				#endif
		 //   lcd_buttons_update();
				temp_state = 7;
				break;
			case 7: // Measure TEMP_2
				#if defined(TEMP_2_PIN)
				 raw_temp_2_value += ADC_GetConversionValue(ADC1>>2);////ADC;
				#endif
				temp_state = 0; //0-7不断的循环
				temp_count++; //一轮读4个温度(3个挤出机和1个热床)耗时8ms,一共累加每组16次的AD值,
				break;
			default:
				printf(MSG_ERR);
				printf("Temp measurement error!\n\r");
				break;
		}
	//每一个温度读16次,一共耗时8*16=128ms
		if(temp_count >= 16) // 8 ms * 16 = 128ms.更新温度
	  {
				if (!temp_meas_ready) //Only update the raw values if they have been read. Else we could be updating them during reading.
				{
						current_temperature_raw[0] = raw_temp_0_value; //保存4组的AD数据，累加16次的
			#if EXTRUDERS > 1
						current_temperature_raw[1] = raw_temp_1_value;
			#endif
			#if EXTRUDERS > 2
						current_temperature_raw[2] = raw_temp_2_value;
			#endif
						current_temperature_bed_raw = raw_temp_bed_value;
				}
				temp_meas_ready = true; //数据清空,进行下一轮温度的读数
				temp_count = 0;
				raw_temp_0_value = 0;
				raw_temp_1_value = 0;
				raw_temp_2_value = 0;
				raw_temp_bed_value = 0;

	#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
			if(current_temperature_raw[0] <= maxttemp_raw[0]) {
	#else
			if(current_temperature_raw[0] >= maxtte	mp_raw[0]) {
	#endif
			//		max_temp_error(0);
			}
	#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
			if(current_temperature_raw[0] >= minttemp_raw[0]) {
	#else
			if(current_temperature_raw[0] <= minttemp_raw[0]) {
	#endif
				//	min_temp_error(0);
			}
	#if EXTRUDERS > 1
	#if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
			if(current_temperature_raw[1] <= maxttemp_raw[1]) {
	#else
			if(current_temperature_raw[1] >= maxttemp_raw[1]) {
	#endif
				//	max_temp_error(1);
			}
	#if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
			if(current_temperature_raw[1] >= minttemp_raw[1]) {
	#else
			if(current_temperature_raw[1] <= minttemp_raw[1]) {
	#endif
					min_temp_error(1);
			}
	#endif
	#if EXTRUDERS > 2
	#if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
			if(current_temperature_raw[2] <= maxttemp_raw[2]) {
	#else
			if(current_temperature_raw[2] >= maxttemp_raw[2]) {
	#endif
					max_temp_error(2);
			}
	#if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
			if(current_temperature_raw[2] >= minttemp_raw[2]) {
	#else
			if(current_temperature_raw[2] <= minttemp_raw[2]) {
	#endif
					min_temp_error(2);
			}
	#endif

		/* No bed MINTEMP error? */
	#if defined(BED_MAXTEMP) && (TEMP_SENSOR_BED != 0)
	# if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
			if(current_temperature_bed_raw <= bed_maxttemp_raw) {
	#else
			if(current_temperature_bed_raw >= bed_maxttemp_raw) {
	#endif
				 target_temperature_bed = 0;
				 bed_max_temp_error();
			}
	#endif
		}
	}
}
#ifdef __cplusplus
	}
#endif

#ifdef PIDTEMP
// Apply the scale factors to the PID values

float scalePID_i(float i)
{
	return i*PID_dT;
}

float unscalePID_i(float i)
{
	return i/PID_dT;
}

float scalePID_d(float d)
{
    return d/PID_dT;
}

float unscalePID_d(float d)
{
	return d*PID_dT;
}

#endif //PIDTEMP



