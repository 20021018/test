################################################################################
# MRS Version: 1.9.1
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Debug/debug.c 

C_DEPS += \
./Debug/debug.d 

OBJS += \
./Debug/debug.o 


# Each subdirectory must supply rules for building sources it contributes
Debug/%.o: ../Debug/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"D:\桌面\毕业设计\程序源码\fatfs\fatfs\Debug" -I"D:\桌面\毕业设计\程序源码\fatfs\fatfs\FATFS\src\option" -I"D:\桌面\毕业设计\程序源码\fatfs\fatfs\FATFS" -I"D:\桌面\毕业设计\程序源码\fatfs\fatfs\FATFS\src" -I"D:\桌面\毕业设计\程序源码\fatfs\fatfs\BSP" -I"D:\桌面\毕业设计\程序源码\fatfs\fatfs\marlin" -I"D:\桌面\毕业设计\程序源码\fatfs\fatfs\Core" -I"D:\桌面\毕业设计\程序源码\fatfs\fatfs\User" -I"D:\桌面\毕业设计\程序源码\fatfs\fatfs\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

