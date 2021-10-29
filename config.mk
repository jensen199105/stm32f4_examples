export SELF_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

CROSS_COMPILER := arm-none-eabi-

export CC = $(CROSS_COMPILER)gcc
export AS = $(CROSS_COMPILER)gcc -x assembler-with-cpp
export AR = $(CROSS_COMPILER)ar
export LD = $(CROSS_COMPILER)ld
export OD = $(CROSS_COMPILER)objdump
export BIN = $(CROSS_COMPILER)objcopy -O ihex
export SIZE = $(CROSS_COMPILER)size
export GDB = $(CROSS_COMPILER)gdb


export CFLAGS += -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -ffunction-sections -fdata-sections \
		-O0 -DUSE_HAL_DRIVER -DSTM32F411xE -DUSE_STM32F411E_DISCO -gdwarf-2 -Wall -Wno-attributes -fverbose-asm \

export ASFLAGS += -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g -gdwarf-2
export LDFLAGS += -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wl,--cref,--gc-sections,--no-warn-mismatch --specs=rdimon.specs -lc -lgcc -lm -lrdimon

export INCLUDES := -I./inc -I/home/jensen/Documents/STM32CubeF4/Drivers/BSP/STM32F411E-Discovery \
			-I/home/jensen/Documents/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Inc \
			-I/home/jensen/Documents/STM32CubeF4/Drivers/CMSIS/Core/Include \
			-I/home/jensen/Documents/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Include

