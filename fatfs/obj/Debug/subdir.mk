################################################################################
# MRS Version: 1.9.1
# �Զ����ɵ��ļ�����Ҫ�༭��
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
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"D:\����\��ҵ���\����Դ��\mini-3D-printer based on ch32v307 - git\fatfs\Debug" -I"D:\����\��ҵ���\����Դ��\mini-3D-printer based on ch32v307 - git\fatfs\FATFS\src\option" -I"D:\����\��ҵ���\����Դ��\mini-3D-printer based on ch32v307 - git\fatfs\FATFS" -I"D:\����\��ҵ���\����Դ��\mini-3D-printer based on ch32v307 - git\fatfs\FATFS\src" -I"D:\����\��ҵ���\����Դ��\mini-3D-printer based on ch32v307 - git\fatfs\BSP" -I"D:\����\��ҵ���\����Դ��\mini-3D-printer based on ch32v307 - git\fatfs\marlin" -I"D:\����\��ҵ���\����Դ��\mini-3D-printer based on ch32v307 - git\fatfs\Core" -I"D:\����\��ҵ���\����Դ��\mini-3D-printer based on ch32v307 - git\fatfs\User" -I"D:\����\��ҵ���\����Դ��\mini-3D-printer based on ch32v307 - git\fatfs\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

