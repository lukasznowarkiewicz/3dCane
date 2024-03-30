################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include MKUART/subdir.mk
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
KijD_T2_2_CZUJ.lss \

FLASH_IMAGE += \
KijD_T2_2_CZUJ.hex \

EEPROM_IMAGE += \
KijD_T2_2_CZUJ.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: KijD_T2_2_CZUJ.elf secondary-outputs

# Tool invocations
KijD_T2_2_CZUJ.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,KijD_T2_2_CZUJ.map -mmcu=atmega8 -o "KijD_T2_2_CZUJ.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

KijD_T2_2_CZUJ.lss: KijD_T2_2_CZUJ.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S KijD_T2_2_CZUJ.elf  >"KijD_T2_2_CZUJ.lss"
	@echo 'Finished building: $@'
	@echo ' '

KijD_T2_2_CZUJ.hex: KijD_T2_2_CZUJ.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex KijD_T2_2_CZUJ.elf  "KijD_T2_2_CZUJ.hex"
	@echo 'Finished building: $@'
	@echo ' '

KijD_T2_2_CZUJ.eep: KijD_T2_2_CZUJ.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex KijD_T2_2_CZUJ.elf  "KijD_T2_2_CZUJ.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: KijD_T2_2_CZUJ.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega8 KijD_T2_2_CZUJ.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) KijD_T2_2_CZUJ.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
