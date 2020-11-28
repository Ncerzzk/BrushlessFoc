##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.5.2] date: [Sat Sep 26 15:17:27 CST 2020] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = BJMDriver
TARGET2 = MYFOCDriver

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build
BUILD_DIR2 = build_my

######################################
# source
######################################


# C sources
C_SOURCES =  \
Src/main.c \
mylib/uart_ext.c \
mylib/command.c \
mylib/as5047.c \
mylib/pid.c \
mylib/soft_i2c.c \
mylib/icm20600.c \
mylib/easy_angle.c \
mylib/debug_utils.c \
Src/utils.c \
Src/gpio.c \
Src/SVPWM.c \
Src/parameter.c \
Src/arm_sin_cos_f32.c \
Src/arm_sin_f32.c \
Src/arm_cos_f32.c \
Src/arm_common_tables.c \
Src/foc.c \
Src/cmd_fun.c \
Src/dac.c \
Src/adc.c \
Src/can.c \
Src/sys.c \
Src/tim.c \
Src/usart.c \
Src/usb_otg.c \
Src/stm32f4xx_it.c \
Src/stm32f4xx_hal_msp.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c \
Src/system_stm32f4xx.c \
Src/dma.c \
Src/spi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c

# ASM sources
ASM_SOURCES =  \
startup_stm32f405xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
CPP = $(GCC_PATH)/$(PREFIX)g++
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
CPP = $(PREFIX)g++
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F405xx \
-DARM_MATH_CM4 \
-D__FPU_PRESENT=1 


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/CMSIS/DSP/Include \
-Imylib


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F405RGTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
LDFLAGS += -lrdimon -u _printf_float

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
	$(CC) -c $(CFLAGS) -DBENJAMIN -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

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


#######################################
# MY FOC Driver Build
#######################################
my: $(BUILD_DIR2)/$(TARGET2).elf $(BUILD_DIR2)/$(TARGET2).hex $(BUILD_DIR2)/$(TARGET2).bin
LDFLAGS2 = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR2)/$(TARGET2).map,--cref -Wl,--gc-sections
LDFLAGS2 += -lrdimon -u _printf_float
#######################################
# build the application
#######################################
# list of objects
OBJECTS2 = $(addprefix $(BUILD_DIR2)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS2 += $(addprefix $(BUILD_DIR2)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR2)/%.o: %.c Makefile | $(BUILD_DIR2) 
	$(CC) -c $(CFLAGS) -DMYOWN_FOC -Wa,-a,-ad,-alms=$(BUILD_DIR2)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR2)/%.o: %.cpp Makefile | $(BUILD_DIR2) 
	$(CC) -c $(CFLAGS) -DMYOWN_FOC -Wa,-a,-ad,-alms=$(BUILD_DIR2)/$(notdir $(<:.c=.lst)) $< -o $

$(BUILD_DIR2)/%.o: %.s Makefile | $(BUILD_DIR2)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR2)/$(TARGET2).elf: $(OBJECTS2) Makefile
	$(CC) $(OBJECTS2) $(LDFLAGS2) -o $@
	$(SZ) $@

$(BUILD_DIR2)/%.hex: $(BUILD_DIR2)/%.elf | $(BUILD_DIR2)
	$(HEX) $< $@
	
$(BUILD_DIR2)/%.bin: $(BUILD_DIR2)/%.elf | $(BUILD_DIR2)
	$(BIN) $< $@	
	
$(BUILD_DIR2):
	mkdir $@		
#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
	-rm -fR $(BUILD_DIR2)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
