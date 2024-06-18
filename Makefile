TARGET= h723_ssr
DEBUG = 1
OPT = -Og
BUILD_DIR = build

LIB_ROOT = ../..
DEV_ENV = ../../stm32armenv/h723zgt6_ssr
DRIVER_NAME = STM32H7xx_HAL_Driver


######################################
# source
######################################
# C sources
C_SOURCES =  \
$(wildcard $(DEV_ENV)/Core/Src/*.c) \
$(wildcard $(DEV_ENV)/Drivers/$(DRIVER_NAME)/Src/*.c) \
$(wildcard $(DEV_ENV)/Middlewares/Third_Party/FreeRTOS/Source/*.c) \
$(wildcard $(DEV_ENV)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c) \
$(wildcard $(DEV_ENV)/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c) \
$(wildcard $(DEV_ENV)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c) \
$(wildcard $(LIB_ROOT)/stm32armlibs/segger_rtt/*.c) \
$(wildcard Src/*.c) \


# ASM sources
ASM_SOURCES =  \
$(DEV_ENV)/startup_stm32h723xx.s

PREFIX = arm-none-eabi-

ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

CPU = -mcpu=cortex-m4
# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

AS_DEFS =

C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32H723xx


# AS includes
AS_INCLUDES =  \
-I$(DEV_ENV)/Core/Inc

# C includes
C_INCLUDES =  \
-I$(DEV_ENV)/Core/Inc \
-I$(DEV_ENV)/Drivers/$(DRIVER_NAME)/Inc \
-I$(DEV_ENV)/Drivers/$(DRIVER_NAME)/Inc/Legacy \
-I$(DEV_ENV)/Middlewares/Third_Party/FreeRTOS/Source/include \
-I$(DEV_ENV)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I$(DEV_ENV)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
-I$(DEV_ENV)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-I$(DEV_ENV)/Drivers/CMSIS/Include \
-I$(DEV_ENV)/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
-I$(LIB_ROOT)/stm32armlibs/segger_rtt \
-IInc \


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = $(DEV_ENV)/STM32H723ZGTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

clean:
	-rm -fR $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***