################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include PetitFS/subdir.mk
-include MKUART/subdir.mk
-include I2C_TWI/subdir.mk
-include DS3231/subdir.mk
-include ADXL_345/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
Zeg_M1.lss \

FLASH_IMAGE += \
Zeg_M1.hex \

EEPROM_IMAGE += \
Zeg_M1.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: Zeg_M1.elf secondary-outputs

# Tool invocations
Zeg_M1.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,Zeg_M1.map -mmcu=atmega328p -o "Zeg_M1.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Zeg_M1.lss: Zeg_M1.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Zeg_M1.elf  >"Zeg_M1.lss"
	@echo 'Finished building: $@'
	@echo ' '

Zeg_M1.hex: Zeg_M1.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex Zeg_M1.elf  "Zeg_M1.hex"
	@echo 'Finished building: $@'
	@echo ' '

Zeg_M1.eep: Zeg_M1.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex Zeg_M1.elf  "Zeg_M1.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Zeg_M1.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p Zeg_M1.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) Zeg_M1.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
