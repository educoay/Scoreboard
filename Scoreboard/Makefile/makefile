CXX=avr-g++

ARDUINO_DIR=/opt/arduino
PROJECT_DIR=..
DISPLAYGROUP_DIR=../../DisplayGroup
ANALOG_BUT_DIR=../../AnalogButtons

# avr tools path
AVR_OBJCOPY=avr-objcopy
AVR_SIZE=avr-size
AVRDUDE=$(ARDUINO_DIR)/hardware/tools/avrdude


# CPU type and speed
MCU=atmega328p
CPU_SPEED=16000000UL

# Arduino USB port for Linux
PORT=/dev/ttyACM0


# Include (dependencies: displaygroup, timedaction, analogbuttons, avr-stl, arduino)
INCLUDE=-I$(DISPLAYGROUP_DIR)\
-I$(ANALOG_BUT_DIR)\
-I$(ARDUINO_DIR)/hardware/tools/avr/avr-stl/include\
-I$(ARDUINO_DIR)/hardware/arduino/variants/standard\
-I$(ARDUINO_DIR)/hardware/arduino/cores/arduino

# Libraries (dependencies: displaygroup, timedaction, analogbuttons, core)
LIBS=-L$(DISPLAYGROUP_DIR)/Makefile\
-L$(ANALOG_BUT_DIR)/Release\
-L$(PROJECT_DIR)/Lib\
-ldisplaygroup -lanalogbuttons -lcore

# Source file and application name
OBJ=main
TARGET=ScoreBoard

CFLAGS=-Wall -Wno-unused-local-typedefs -Os -fpack-struct -fshort-enums -funsigned-char -funsigned-bitfields\
-fno-exceptions -ffunction-sections -fdata-sections -fno-use-cxa-atexit -mmcu=$(MCU) -DF_CPU=$(CPU_SPEED) -MMD -MP -MF"$(OBJ).d" -MT"$(OBJ).d"

CLINKFLAGS=-Wl,-Map,$(TARGET).map,--cref -Wl,-gc-sections -mmcu=$(MCU)



default: build sizedummy upload 

build: $(TARGET).hex
	
$(TARGET).hex: $(TARGET).elf
	@echo 'Create Flash image (ihex format)'
	$(AVR_OBJCOPY) -R .eeprom -O ihex $< $@
	@echo 'Finished building target: $@'
	@echo ' '

$(TARGET).elf: $(OBJ).o
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	$(CXX) $(CLINKFLAGS) $(INCLUDE) $^ -o $@ $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	
%.o: ../%.cpp
	$(CXX) $< $(CFLAGS) $(INCLUDE) -c -o $@
	
upload:
	@echo 'Invoking: AVRDude'
	$(AVRDUDE) -pm328p -carduino -P$(PORT) -b115200 -Uflash:w:$(TARGET).hex:a
	@echo 'Finished building: $@'
	@echo ' '
	
sizedummy: $(TARGET).elf
	@echo 'Invoking: Print Size'
	$(AVR_SIZE) --format=avr --mcu=$(MCU) $(TARGET).elf
	@echo 'Finished building: $@'
	@echo ' '	
	
clean:
	@echo -n Cleaning ...
	$(shell rm $(TARGET).elf 2> /dev/null)
	$(shell rm $(TARGET).hex 2> /dev/null)
	$(shell rm $(TARGET).map 2> /dev/null)
	$(shell rm *.o 2> /dev/null)
	$(shell rm *.d 2> /dev/null)
	@echo " done"
	