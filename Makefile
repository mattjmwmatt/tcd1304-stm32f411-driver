# Makefile for TCD1304 driver on Nucleo-F411RE
# Requires: arm-none-eabi-gcc, arm-none-eabi-objcopy, st-flash (or openocd)

TARGET  = tcd1304_f411
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

# Adjust to your local STM32CubeF4 path
CMSIS_INC ?= $(HOME)/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Include
CORE_INC  ?= $(HOME)/STM32CubeF4/Drivers/CMSIS/Include

SRCS  = src/main.c
ASRCS = src/startup_stm32f411xe.s
LD    = STM32F411RETx_FLASH.ld

CFLAGS  = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS += -DSTM32F411xE
CFLAGS += -I$(CMSIS_INC) -I$(CORE_INC)
CFLAGS += -O2 -Wall -Wextra -ffunction-sections -fdata-sections -g
LDFLAGS = -T$(LD) -Wl,--gc-sections -Wl,-Map=$(TARGET).map --specs=nosys.specs

all: $(TARGET).bin

$(TARGET).elf: $(SRCS) $(ASRCS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x08000000

clean:
	rm -f $(TARGET).elf $(TARGET).bin $(TARGET).map

.PHONY: all flash clean
