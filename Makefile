# Portable Cortex-M0+ build; do not reuse the checked-in Windows Debug objects.
TOOLCHAIN_PATH ?= /workspace/toolchains/arm-debian
ifneq ($(wildcard $(TOOLCHAIN_PATH)/usr/bin/arm-none-eabi-gcc),)
CROSS ?= $(TOOLCHAIN_PATH)/usr/bin/arm-none-eabi-
NEWLIB := $(TOOLCHAIN_PATH)/usr/lib/arm-none-eabi/newlib
SYSFLAGS := -isystem $(TOOLCHAIN_PATH)/usr/include/newlib -B$(NEWLIB)/thumb/v6-m/nofp/
else
CROSS ?= arm-none-eabi-
endif

CC := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE := $(CROSS)size
BUILD_DIR ?= build
TARGET := $(BUILD_DIR)/FlightHapticController
ARCH := -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft
CPPFLAGS := -DDEBUG -DUSE_HAL_DRIVER -DSTM32C031xx \
 -ICore/Inc -IDrivers/STM32C0xx_HAL_Driver/Inc \
 -IDrivers/STM32C0xx_HAL_Driver/Inc/Legacy \
 -IDrivers/CMSIS/Device/ST/STM32C0xx/Include -IDrivers/CMSIS/Include
CFLAGS := $(ARCH) $(SYSFLAGS) -std=gnu11 -O0 -g3 -Wall -Wextra \
 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP --specs=nano.specs
LDFLAGS := $(ARCH) $(SYSFLAGS) -TSTM32C031C6TX_FLASH.ld \
 --specs=nano.specs --specs=nosys.specs -static \
 -Wl,--gc-sections,-Map=$(TARGET).map,--print-memory-usage
C_SOURCES := $(wildcard Core/Src/*.c) $(wildcard Drivers/STM32C0xx_HAL_Driver/Src/*.c)
ASM_SOURCES := $(wildcard Core/Startup/*.s)
OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(C_SOURCES)) \
 $(patsubst %.s,$(BUILD_DIR)/%.o,$(ASM_SOURCES))

.PHONY: all clean test
all: $(TARGET).elf $(TARGET).hex $(TARGET).bin

$(TARGET).elf: $(OBJECTS) STM32C031C6TX_FLASH.ld Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -Wl,--start-group -lc -lm -Wl,--end-group -o $@
	$(SIZE) $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile
	@mkdir -p $(dir $@)
	$(CC) $(ARCH) $(SYSFLAGS) -g3 -x assembler-with-cpp -MMD -MP -c $< -o $@

test:
	./tests/run.sh

clean:
	rm -rf $(BUILD_DIR)

-include $(OBJECTS:.o=.d)
