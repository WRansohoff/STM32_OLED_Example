TARGET = main

# Default target chip.
MCU ?= STM32F103C8
#MCU ?= STM32F303K8

ifeq ($(MCU), STM32F103C8)
	MCU_FILES  = STM32F103C8T6
	ST_MCU_DEF = STM32F103xB
	MCU_CLASS  = F1
else ifeq ($(MCU), STM32F303K8)
	MCU_FILES  = STM32F303K8T6
	ST_MCU_DEF = STM32F303x8
	MCU_CLASS  = F3
endif

# Define the chip architecture.
LD_SCRIPT = $(MCU_FILES).ld
ifeq ($(MCU_CLASS), F1)
	MCU_SPEC = cortex-m3
	FREERTOS_PORT_I = ./freertos/Source/portable/GCC/ARM_CM3
else ifeq ($(MCU_CLASS), F3)
	MCU_SPEC = cortex-m4
	FREERTOS_PORT_I = ./freertos/Source/portable/GCC/ARM_CM4F
endif
FREERTOS_PORT_C = $(FREERTOS_PORT_I)/port.c

# Toolchain definitions (ARM bare metal defaults)
TOOLCHAIN = /usr/bin
CC  = $(TOOLCHAIN)/arm-none-eabi-gcc
CPP = $(TOOLCHAIN)/arm-none-eabi-g++
AS  = $(TOOLCHAIN)/arm-none-eabi-as
LD  = $(TOOLCHAIN)/arm-none-eabi-ld
OC  = $(TOOLCHAIN)/arm-none-eabi-objcopy
OD  = $(TOOLCHAIN)/arm-none-eabi-objdump
OS  = $(TOOLCHAIN)/arm-none-eabi-size

# Assembly directives.
ASFLAGS += -mcpu=$(MCU_SPEC)
ASFLAGS += -mthumb

# C compilation directives
CFLAGS += -mcpu=$(MCU_SPEC)
CFLAGS += -mthumb
ifeq ($(MCU_CLASS), F3)
	CFLAGS += -mhard-float
	CFLAGS += -mfloat-abi=hard
	CFLAGS += -mfpu=fpv4-sp-d16
else
	CFLAGS += -msoft-float
	CFLAGS += -mfloat-abi=soft
endif
CFLAGS += -Wall
CFLAGS += -g
CFLAGS += -Os
CFLAGS += -std=c99
CFLAGS += -fmessage-length=0 -fno-common
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -D$(ST_MCU_DEF)
CFLAGS += -DVVC_$(MCU_CLASS)
CFLAGS += -DVVC_$(MCU)

# C++ compilation directives
CPPFLAGS += -mcpu=$(MCU_SPEC)
CPPFLAGS += -mthumb
ifeq ($(MCU_CLASS), F3)
	CPPFLAGS += -mhard-float
	CPPFLAGS += -mfloat-abi=hard
	CPPFLAGS += -mfpu=fpv4-sp-d16
else
	CPPFLAGS += -msoft-float
	CPPFLAGS += -mfloat-abi=soft
endif
CPPFLAGS += -Wall
CPPFLAGS += -g
CPPFLAGS += -Os
CPPFLAGS += -fmessage-length=0 -fno-common
CPPFLAGS += -ffunction-sections -fdata-sections
CPPFLAGS += -fno-exceptions
CPPFLAGS += -D$(ST_MCU_DEF)
CPPFLAGS += -DVVC_$(MCU_CLASS)
CPPFLAGS += -DVVC_$(MCU)

# Linker directives.
LSCRIPT = ./ld/$(LD_SCRIPT)
LFLAGS += -mcpu=$(MCU_SPEC)
LFLAGS += -mthumb
ifeq ($(MCU_CLASS), F3)
	LFLAGS += -mhard-float
	LFLAGS += -mfloat-abi=hard
	LFLAGS += -mfpu=fpv4-sp-d16
else
	LFLAGS += -msoft-float
	LFLAGS += -mfloat-abi=soft
endif
LFLAGS += -Wall
LFLAGS += --static
LFLAGS += -Wl,-Map=$(TARGET).map
LFLAGS += -Wl,--gc-sections
LFLAGS += -lgcc
LFLAGS += -lc
LFLAGS += -T$(LSCRIPT)

# Source files.
AS_SRC    = ./boot_s/$(MCU_FILES)_boot.S
AS_SRC   += ./vector_tables/$(MCU_FILES)_vt.S
C_SRC     = ./freertos/Source/portable/MemMang/heap_4.c
C_SRC    += $(FREERTOS_PORT_C)
C_SRC    += ./freertos/Source/list.c
C_SRC    += ./freertos/Source/tasks.c
C_SRC    += ./freertos/Source/queue.c
CPP_SRC   = ./src/main.cpp
CPP_SRC  += ./src/peripherals.cpp
CPP_SRC  += ./src/util.cpp
CPP_SRC  += ./src/global.cpp

INCLUDE  += -I./
INCLUDE  += -I./src
INCLUDE  += -I./device_headers
INCLUDE  += -I./freertos/Source/include
INCLUDE  += -I$(FREERTOS_PORT_I)

OBJS  = $(C_SRC:.c=.o)
OBJS += $(CPP_SRC:.cpp=.o)
OBJS += $(AS_SRC:.S=.o)

.PHONY: all
all: $(TARGET).bin

%.o: %.S
	$(AS) $(ASFLAGS) -c $< -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $(INCLUDE) $< -o $@

%.o: %.cpp
	$(CPP) -c $(CPPFLAGS) $(INCLUDE) $< -o $@

$(TARGET).elf: $(OBJS)
	$(CPP) $^ $(LFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OC) -S -O binary $< $@
	$(OS) $<

.PHONY: clean
clean:
	rm -f $(OBJS)
	rm -f $(TARGET).elf
	rm -f $(TARGET).bin
	rm -f $(TARGET).map
