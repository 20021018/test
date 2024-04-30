################################################################################
# MRS Version: 1.9.1
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../BSP/bsp_usart.cpp 

C_SRCS += \
../BSP/MMC_SD.c \
../BSP/bsp_adc.c \
../BSP/bsp_flash.c \
../BSP/bsp_pin.c \
../BSP/bsp_sysclk.c \
../BSP/bsp_watchdog.c 

C_DEPS += \
./BSP/MMC_SD.d \
./BSP/bsp_adc.d \
./BSP/bsp_flash.d \
./BSP/bsp_pin.d \
./BSP/bsp_sysclk.d \
./BSP/bsp_watchdog.d 

OBJS += \
./BSP/MMC_SD.o \
./BSP/bsp_adc.o \
./BSP/bsp_flash.o \
./BSP/bsp_pin.o \
./BSP/bsp_sysclk.o \
./BSP/bsp_usart.o \
./BSP/bsp_watchdog.o 

CPP_DEPS += \
./BSP/bsp_usart.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/%.o: ../BSP/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"D:\桌面\test-master\fatfs\Debug" -I"D:\桌面\test-master\fatfs\FATFS\src\option" -I"D:\桌面\test-master\fatfs\FATFS" -I"D:\桌面\test-master\fatfs\FATFS\src" -I"D:\桌面\test-master\fatfs\BSP" -I"D:\桌面\test-master\fatfs\marlin" -I"D:\桌面\test-master\fatfs\Core" -I"D:\桌面\test-master\fatfs\User" -I"D:\桌面\test-master\fatfs\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
BSP/%.o: ../BSP/%.cpp
	@	@	riscv-none-embed-g++ -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"D:\桌面\test-master\fatfs\Debug" -I"D:\桌面\test-master\fatfs\FATFS\src\option" -I"D:\桌面\test-master\fatfs\FATFS" -I"D:\桌面\test-master\fatfs\FATFS\src" -I"D:\桌面\test-master\fatfs\BSP" -I"D:\桌面\test-master\fatfs\marlin" -I"D:\桌面\test-master\fatfs\Core" -I"D:\桌面\test-master\fatfs\User" -I"D:\桌面\test-master\fatfs\Peripheral\inc" -std=gnu++11 -fabi-version=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

