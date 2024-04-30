################################################################################
# MRS Version: 1.9.1
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FATFS/src/option/ccsbcs.c \
../FATFS/src/option/syscall.c 

C_DEPS += \
./FATFS/src/option/ccsbcs.d \
./FATFS/src/option/syscall.d 

OBJS += \
./FATFS/src/option/ccsbcs.o \
./FATFS/src/option/syscall.o 


# Each subdirectory must supply rules for building sources it contributes
FATFS/src/option/%.o: ../FATFS/src/option/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"D:\桌面\test-master\fatfs\Debug" -I"D:\桌面\test-master\fatfs\FATFS\src\option" -I"D:\桌面\test-master\fatfs\FATFS" -I"D:\桌面\test-master\fatfs\FATFS\src" -I"D:\桌面\test-master\fatfs\BSP" -I"D:\桌面\test-master\fatfs\marlin" -I"D:\桌面\test-master\fatfs\Core" -I"D:\桌面\test-master\fatfs\User" -I"D:\桌面\test-master\fatfs\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

