CXX=avr-g++

ARDUINO_DIR=/opt/arduino-1.0.5
LIB_DIR=..

# avr tools path
AVR_AR=avr-ar
AVR_OBJDUMP=avr-objdump
AVR_SIZE=avr-size


# CPU type and speed
MCU=atmega328p
CPU_SPEED=16000000UL


# Include (dependencies: avr-stl, arduino)
INCLUDE=-I$(LIB_DIR)\
-I$(ARDUINO_DIR)/hardware/tools/avr/avr-stl/include\
-I$(ARDUINO_DIR)/hardware/arduino/variants/standard\
-I$(ARDUINO_DIR)/hardware/arduino/cores/arduino


# Source objects and library name
LIBNAME=displaygroup
LIBFILE = lib$(LIBNAME).a

LIBOBJS=Display.o DisplayGroup.o DisplayManager.o

CFLAGS=-Wall -Os -fpack-struct -fshort-enums -funsigned-char -funsigned-bitfields\
-fno-exceptions -ffunction-sections -fdata-sections -mmcu=$(MCU) -DF_CPU=$(CPU_SPEED) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"


default: build lss sizedummy

build: $(LIBFILE)
	
$(LIBFILE): $(LIBOBJS)
	@echo "Creating library $@"
	$(AVR_AR) -r $@ $(LIBOBJS)
	@echo 'Finished building target: $@'
	@echo ' '
	
%.o: ../%.cpp
	$(CXX) $< $(CFLAGS) $(INCLUDE) -c -o $@
	
lss: $(LIBFILE)
	@echo 'Invoking: AVR Create Extended Listing'
	$(AVR_OBJDUMP) -h -S $(LIBFILE) > "lib$(LIBNAME).lss"
	@echo 'Finished building: $@'
	@echo ' '
	
sizedummy: $(LIBFILE)
	@echo 'Invoking: Print Size'
	$(AVR_SIZE) --format=avr --mcu=$(MCU) $(LIBFILE)
	@echo 'Finished building target: $@'
	@echo ' '
	
clean:
	@echo -n Cleaning ...
	$(shell rm *.a 2> /dev/null)
	$(shell rm *.d 2> /dev/null)
	$(shell rm *.o 2> /dev/null)
	$(shell rm *.lss 2> /dev/null)
	@echo " done"
	