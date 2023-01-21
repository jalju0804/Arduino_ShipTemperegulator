################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../__vm/


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../Adafruit_MAX31865.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
Adafruit_MAX31865.o

OBJS_AS_ARGS +=  \
Adafruit_MAX31865.o

C_DEPS +=  \
Adafruit_MAX31865.d

C_DEPS_AS_ARGS +=  \
Adafruit_MAX31865.d

OUTPUT_FILE_PATH +=SteamGen1.elf

OUTPUT_FILE_PATH_AS_ARGS +=SteamGen1.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler
./Adafruit_MAX31865.o: .././Adafruit_MAX31865.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 5.4.0
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -D__AVR_ATmega168__ -DARDUINO=108012 -DARDUINO_MAIN -DF_CPU=8000000L -D__AVR__ -DARDUINO_AVR_PRO -DARDUINO_ARCH_AVR  -I"../../../ShipTemp_6thD60701300ptc" -I"c:/Program Files (x86)/arduino/hardware/arduino/avr/libraries/EEPROM/src" -I"c:/Program Files (x86)/arduino/hardware/arduino/avr/cores/arduino" -I"c:/Program Files (x86)/arduino/hardware/arduino/avr/variants/eightanaloginputs" -I"c:/Program Files (x86)/arduino/hardware/tools/avr//lib/gcc/avr/7.3.0/include" -I"c:/Program Files (x86)/arduino/hardware/tools/avr/avr/include" -I"c:/Program Files (x86)/arduino/hardware/tools/avr/avr/include-fixed" -I"c:/Program Files (x86)/arduino/hardware/tools/avr/avr/include/avr" -I"c:/Program Files (x86)/arduino/hardware/tools/avr/lib/gcc/avr/4.9.2/include" -I"c:/Program Files (x86)/arduino/hardware/tools/avr/lib/gcc/avr/4.9.3/include" -I"{vm.intellisense.add-paths}"  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	




# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR8/GNU Linker : 5.4.0
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="SteamGen1.map" -Wl,--start-group  -Wl,--end-group -Wl,--gc-sections  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "SteamGen1.elf" "SteamGen1.hex"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "SteamGen1.elf" "SteamGen1.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "SteamGen1.elf" > "SteamGen1.lss"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "SteamGen1.elf" "SteamGen1.srec"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe" "SteamGen1.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "SteamGen1.elf" "SteamGen1.a" "SteamGen1.hex" "SteamGen1.lss" "SteamGen1.eep" "SteamGen1.map" "SteamGen1.srec" "SteamGen1.usersignatures"
	