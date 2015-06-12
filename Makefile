
# Put your stlink folder here so make burn will work.
STLINK=~/Programs/stlink

# Put your source files here (or *.c, etc)
SRCS=main.c stm32f10x_it.c

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=apitox

# Put your STM32F100 library code directory here
STM_COMMON=/home/yborisov/projects_work/mc_study/stm32vlddiscovery/an3268/stm32vldiscovery_package

# Normally you shouldn't need to change anything below this line!
#######################################################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

//CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld 
CFLAGS  = -g -O0 -Wall -Tstm32_flash.ld 
#CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m3 -mthumb-interwork
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m3
#CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I.
CFLAGS += -specs=nosys.specs

# Include files from STM libraries
CFLAGS += -I$(STM_COMMON)/Utilities
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/CM3/CoreSupport
CFLAGS += -I$(STM_COMMON)/Libraries/STM32F10x_StdPeriph_Driver/inc

# add sysem clock init
SRCS += $(STM_COMMON)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
# Led utilities
SRCS += $(STM_COMMON)/Utilities/STM32vldiscovery.c
# Reset and clock control (RCC) drivers
SRCS += $(STM_COMMON)/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
# GPIO drivers
SRCS += $(STM_COMMON)/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
# EXTI drivers
SRCS += $(STM_COMMON)/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c
# Timers drivers
SRCS += $(STM_COMMON)/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
# Misc drivers
SRCS += $(STM_COMMON)/Libraries/STM32F10x_StdPeriph_Driver/src/misc.c

# add startup file to build
SRCS += $(STM_COMMON)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md_vl.s
OBJS = $(SRCS:.c=.o)


.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin

# Flash the STM32F1
burn: proj
	$(STLINK)/st-flash write $(PROJ_NAME).bin 0x8000000
