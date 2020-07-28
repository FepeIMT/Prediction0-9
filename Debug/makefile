################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Prediction_0_9.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Prediction_0_9.list \

OBJCOPY_BIN += \
Prediction_0_9.bin \


# All Target
all: Prediction_0_9.elf secondary-outputs

# Tool invocations
Prediction_0_9.elf: $(OBJS) $(USER_OBJS) C:\Users\pepe_\STM32CubeIDE\workspace_1.3.0\Prediction_0_9\STM32L4P5ZGTX_FLASH.ld
	arm-none-eabi-gcc -o "Prediction_0_9.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\pepe_\STM32CubeIDE\workspace_1.3.0\Prediction_0_9\STM32L4P5ZGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Prediction_0_9.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Prediction_0_9.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Prediction_0_9.list"
	@echo 'Finished building: $@'
	@echo ' '

Prediction_0_9.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Prediction_0_9.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
