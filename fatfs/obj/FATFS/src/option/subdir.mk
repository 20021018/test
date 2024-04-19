################################################################################
# MRS Version: 1.9.1
# �Զ����ɵ��ļ�����Ҫ�༭��
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
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"D:\����\��ҵ���\����Դ��\fatfs\fatfs\Debug" -I"D:\����\��ҵ���\����Դ��\fatfs\fatfs\FATFS\src\option" -I"D:\����\��ҵ���\����Դ��\fatfs\fatfs\FATFS" -I"D:\����\��ҵ���\����Դ��\fatfs\fatfs\FATFS\src" -I"D:\����\��ҵ���\����Դ��\fatfs\fatfs\BSP" -I"D:\����\��ҵ���\����Դ��\fatfs\fatfs\marlin" -I"D:\����\��ҵ���\����Դ��\fatfs\fatfs\Core" -I"D:\����\��ҵ���\����Դ��\fatfs\fatfs\User" -I"D:\����\��ҵ���\����Դ��\fatfs\fatfs\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

