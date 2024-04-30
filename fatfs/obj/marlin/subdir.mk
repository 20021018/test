################################################################################
# MRS Version: 1.9.1
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../marlin/ConfigurationStore.cpp \
../marlin/Marlin_main.cpp \
../marlin/SerialLcd.cpp \
../marlin/Servo.cpp \
../marlin/cardreader.cpp \
../marlin/motion_control.cpp \
../marlin/planner.cpp \
../marlin/stepper.cpp \
../marlin/temperature.cpp \
../marlin/ultralcd.cpp 

OBJS += \
./marlin/ConfigurationStore.o \
./marlin/Marlin_main.o \
./marlin/SerialLcd.o \
./marlin/Servo.o \
./marlin/cardreader.o \
./marlin/motion_control.o \
./marlin/planner.o \
./marlin/stepper.o \
./marlin/temperature.o \
./marlin/ultralcd.o 

CPP_DEPS += \
./marlin/ConfigurationStore.d \
./marlin/Marlin_main.d \
./marlin/SerialLcd.d \
./marlin/Servo.d \
./marlin/cardreader.d \
./marlin/motion_control.d \
./marlin/planner.d \
./marlin/stepper.d \
./marlin/temperature.d \
./marlin/ultralcd.d 


# Each subdirectory must supply rules for building sources it contributes
marlin/%.o: ../marlin/%.cpp
	@	@	riscv-none-embed-g++ -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"D:\桌面\test-master\fatfs\BSP" -I"D:\桌面\test-master\fatfs\Peripheral\inc" -I"D:\桌面\test-master\fatfs\User" -I"D:\桌面\test-master\fatfs\marlin" -I"D:\桌面\test-master\fatfs\Core" -I"D:\桌面\test-master\fatfs\Debug" -std=gnu++11 -fabi-version=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

