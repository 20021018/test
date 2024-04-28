#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <stdbool.h>
// This configurtion file contains the basic settings.


#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ // build date and time
#define STRING_CONFIG_H_AUTHOR "(SardineBoard, default config)" //Who made the changes.

/***************�Ƿ���COREXY�ṹ*************/
//#define COREXY

/**************1mm��������*************/
#define DEFAULT_AXIS_STEPS_PER_UNIT   {106.6667,106.6667,400,192}
/**************����ٶ�***************/
#define DEFAULT_MAX_FEEDRATE          {80, 80, 30, 30}    // (mm/sec)
/***************�����ٶ�*********************/
#define DEFAULT_MAX_ACCELERATION      {1000,1000,100,1000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
/****************Ĭ�ϼ��ٶ�******************/
#define DEFAULT_ACCELERATION         1000    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
/*****************�س�ʱ�ļ��ٶ�********************/
#define DEFAULT_RETRACT_ACCELERATION  1000   // X, Y, Z and E max acceleration in mm/s^2 for r retracts

/*****************����г�********************/
// Travel limits after homing
#define X_MAX_POS 50
#define X_MIN_POS 0
#define Y_MAX_POS 50
#define Y_MIN_POS 0
#define Z_MAX_POS 50
#define Z_MIN_POS 0

/******************(����������ʱ)����ķ���********************/
#define INVERT_X_DIR  true     // for Mendel set to false, for Orca set to true
#define INVERT_Y_DIR  true    // for Mendel set to true, for Orca set to false
#define INVERT_Z_DIR  false     // for Mendel set to false, for Orca set to true
#define INVERT_E0_DIR true   // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E1_DIR false    // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E2_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false

/****************��λ���ص�ѡ��**************/
// corse Endstop Settings
// fine Enstop settings: Individual Pullups. will be ignord if ENDSTOPPULLUPS is defined
//#define ENDSTOPPULLUP_XMAX
//#define ENDSTOPPULLUP_YMAX
//#define ENDSTOPPULLUP_ZMAX
#define ENDSTOPPULLUP_XMIN
#define ENDSTOPPULLUP_YMIN
#define ENDSTOPPULLUP_ZMIN


/******************��λ���صĽӷ�,false�ǳ���,true����,������C���ź�,����(NO)���߳���(NC)��GND***********************/
// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
//const bool X_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
//const bool Y_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
//const bool Z_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
//const bool LEVEL_INVERTING  = false;
extern  const bool X_ENDSTOPS_INVERTING; // set to true to invert the logic of the endstops.
extern  const bool Y_ENDSTOPS_INVERTING; // set to true to invert the logic of the endstops.
extern  const bool Z_ENDSTOPS_INVERTING; // set to true to invert the logic of the endstops.
extern  const bool LEVEL_INVERTING;

/****************��λ���ع����ʱ��ȥ��Сλ�û������λ��,������λ����ѡ��********************/
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#ifdef ENDSTOPPULLUP_XMIN
#define X_HOME_DIR -1
#else
#define X_HOME_DIR 1
#endif
#ifdef ENDSTOPPULLUP_YMIN
#define Y_HOME_DIR  -1
#else 
#define Y_HOME_DIR  1
#endif
#ifdef ENDSTOPPULLUP_ZMIN
#define Z_HOME_DIR -1
#else
#define Z_HOME_DIR 1
#endif


/*******************�˶���ĸ���******************/
//// MOVEMENT SETTINGS
#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
/*********************������ٶ�**********************/
#define HOMING_FEEDRATE {50*10, 50*10, 20*50, 0}  // set the homing speeds (mm/min)
/*********************ͨѶ���ڲ�����************************/
// This determines the communication speed of the printer
#define BAUDRATE 115200
/*****************�������ĸ���***********************/
// This defines the number of extruders
#define EXTRUDERS 1

/***********************��Դ����ѡ��********************/
//// The following define selects which power supply you have. Please choose the one that matches your setup
// 1 = ATX
// 2 = X-Box 360 203Watts (the blue wire connected to PS_ON and the red wire to VCC)
#define POWER_SUPPLY 1

/**********************�¶ȴ�������ѡ��(SardineBoardֻ����1)******************/
//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================
//--NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
//
//// Temperature sensor settings:
// -2 is thermocouple with MAX6675 (only for sensor 0)
// -1 is thermocouple with AD595
// 0 is not used
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)    4100k
// 2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
// 3 is mendel-parts thermistor (4.7k pullup)
// 4 is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
// 5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan) (4.7k pullup)
// 6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
// 7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup) 3975
// 8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) no
// 9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)   3952k!!
// 10 is 100k RS thermistor 198-961 (4.7k pullup)
//    1k ohm pullup tables - This is not normal, you would have to have changed out your 4.7k for 1k
//                          (but gives greater accuracy and more stable PID)
// 51 is 100k thermistor - EPCOS (1k pullup)
// 52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
// 55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan) (1k pullup)
#define TEMP_SENSOR_0 9 //�޸������ֵ��Ҫ��temperature.c ��THERMISTORTABLES_H_�����޸Ķ�Ӧ�Ĳ�����
#define TEMP_SENSOR_1 0//һ���¶ȴ�����
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_BED 0//û���ȴ��¶ȴ�����

//������ʾIO����ߵ�ƽ���Ȼ��ǵ͵�ƽ����
#define TEMP_HEAT_VALID  HIGH  //pnp �͵�ƽ��ͨ ->  aon6504 �ߵ�ƽ��ͨ

/********************M109����ʱ�̶�ʱ���ڵ��¶�ƫ��ֵ***********************/
// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 10       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     10       // (degC) Window around target to start the recidency timer x degC early.

/*************************��С�Ĺ����¶�**************************/
// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP -10
#define HEATER_1_MINTEMP -10
#define HEATER_2_MINTEMP -10
#define BED_MINTEMP 5

/******************************���Ĺ����¶�*****************************/
// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.

#define HEATER_0_MAXTEMP 250
#define HEATER_1_MAXTEMP 190
#define HEATER_2_MAXTEMP 190
#define BED_MAXTEMP 50

// If your bed has low resistance e.g. .6 ohm and throws the fuse you can duty cycle it to reduce the
// average current. The value should be an integer and the heat bed will be turned on for 1 interval of
// HEATER_BED_DUTY_CYCLE_DIVIDER intervals.
//#define HEATER_BED_DUTY_CYCLE_DIVIDER 4

/**************************PID����������*************************************/
// PID settings:
// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 256=full current
#define PID_MAX 255 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 256=full current
#ifdef PIDTEMP
//#define PID_DEBUG // Sends debug data to the serial port.
//#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
#define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
// is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
#define PID_INTEGRAL_DRIVE_MAX 255  //limit for the integral term
#define K1 0.95 //smoothing factor withing the PID
#define PID_dT ((16.0 * 8.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the temperature routine

// If you are using a preconfigured hotend then you can use one of the value sets by uncommenting it
// Ultimaker
#define  DEFAULT_Kp 22.2
#define  DEFAULT_Ki 1.08
#define  DEFAULT_Kd 114

// Makergear
//    #define  DEFAULT_Kp 7.0
//    #define  DEFAULT_Ki 0.1
//    #define  DEFAULT_Kd 12

// Mendel Parts V9 on 12V
//    #define  DEFAULT_Kp 63.0
//    #define  DEFAULT_Ki 2.25
//    #define  DEFAULT_Kd 440
#endif // PIDTEMP

// Bed Temperature Control
// Select PID or bang-bang with PIDTEMPBED.  If bang-bang, BED_LIMIT_SWITCHING will enable hysteresis

//#define PIDTEMPBED
//
//#define BED_LIMIT_SWITCHING

// This sets the max power delived to the bed, and replaces the HEATER_BED_DUTY_CYCLE_DIVIDER option.
// all forms of bed control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 256 enables a form of PWM to the bed just like HEATER_BED_DUTY_CYCLE_DIVIDER did,
// so you shouldn't use it unless you are OK with PWM on your bed.  (see the comment on enabling PIDTEMPBED)
#define MAX_BED_POWER 256 // limits duty cycle to bed; 256=full current

#ifdef PIDTEMPBED
//120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
//from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, argressive factor of .15 (vs .1, 1, 10)
#define  DEFAULT_bedKp 6.00
#define  DEFAULT_bedKi 0.023
#define  DEFAULT_bedKd 305.4

//120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
//from pidautotune
//    #define  DEFAULT_bedKp 97.1
//    #define  DEFAULT_bedKi 1.41
//    #define  DEFAULT_bedKd 1675.16

// FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
#endif // PIDTEMPBED


/*********************���¼�������*************************/
//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
/*********************��С��������*************************/
//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE
/*********************��ͼ������¶�*************************/
#define EXTRUDE_MINTEMP 100 
/**********************һ�����ļ�������************************/
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH)*2 

/*********************���ʹ����Ч��ƽ***************************/
// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON  LOW
#define Y_ENABLE_ON  LOW
#define Z_ENABLE_ON  LOW
#define E_ENABLE_ON  LOW // For all extruders��Ӧ������ʹ��EN�͵�ƽ
/***************�����ĸ�����****************/
// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false // For all extruders

/********************֧����������С��λ********************/
#define min_software_endstops true //If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  //If true, axis won't move to coordinates greater than the defined lengths below.

/*******************XYZ�ĳߴ�******************************/
#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

// The position of the homing switches
//#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
//#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)

/*********************�����ƶ�����������,������ƽ̨���м�*****************/
//Manual homing switch locations:
#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS 0


/******************************�����������첹��******************************/
// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
 #define EXTRUDER_OFFSET_X {0.0, 20.00} // (in mm) for each extruder, offset of the hotend on the X axis
 #define EXTRUDER_OFFSET_Y {0.0, 5.00}  // (in mm) for each extruder, offset of the hotend on the Y axis

 /**************************������ٶ�������(С������ٶ��ٶȱ仯û�м��ٶ�)*********************/
// The speed change that does not require acceleration (i.e. the software might assume it can be done instanteneously)
#define DEFAULT_XYJERK                5.0    // (mm/sec)
#define DEFAULT_ZJERK                 0.4     // (mm/sec)
#define DEFAULT_EJERK                 5.0    // (mm/sec)

//===========================================================================
//=============================Additional Features===========================
//===========================================================================
// EEPROM
// the microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable eeprom support
//#define EEPROM_SETTINGS
//to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
//#define EEPROM_CHITCHAT

/*******************�Ƽ��Ĵ�ӡ�¶�����************************/
#define PLA_PREHEAT_HOTEND_TEMP 190
#define PLA_PREHEAT_HPB_TEMP 70
#define PLA_PREHEAT_FAN_SPEED 210   // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 200
#define ABS_PREHEAT_HPB_TEMP 100
#define ABS_PREHEAT_FAN_SPEED 235   // Insert Value between 0 and 255

/**********************�Ƿ�֧��SD��**************************/
#define SDSUPPORT // Enable SD Card Support in Hardware Console
//#define SDSLOW // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)

//#define ULTIMAKERCONTROLLER //as available from the ultimaker online store.
//#define ULTIPANEL  //the ultipanel as on thingiverse


// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
// #define PHOTOGRAPH_PIN     23

// SF send wrong arc g-codes when using Arc Point as fillet procedure
//#define SF_ARC_FIX

// Support for the BariCUDA Paste Extruder.
//#define BARICUDA

/*********************************************************************\

  R/C SERVO support

  Sponsored by TrinityLabs, Reworked by codexmas

**********************************************************************/

// Number of servos
//
// If you select a configuration below, this will receive a default value and does not need to be set manually
// set it manually if you have more servos than extruders and wish to manually control some
// leaving it undefined or defining as 0 will disable the servo subsystem
// If unsure, leave commented / disabled
//
// #define NUM_SERVOS 3

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //__CONFIGURATION_H

