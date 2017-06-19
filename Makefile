PORT ?= /dev/ttyACM0
TARGET=main

LIB=lib


DEVICE = attiny2313a
MCU = attiny2313a
AVRDUDE_DEVICE = attiny2313
PROGRAMMER = linuxspi
#PROGRAMMER = linuxspi2
PORT = /dev/spidev0.0

OBJ=$(TARGET).o

OPT=-Os -flto
CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) $(DEVICE_SPECIFIC_CFLAGS) $(OPT) -std=c99
CC=avr-gcc
LDFLAGS=-Wl,-gc-sections -Wl,-relax -flto

#-lpololu_$(DEVICE) 

AVRDUDE=avrdude

#.PRECIOUS: %.elf

all:$(TARGET).hex memory-used
	
clean:
	rm -f *.o *.hex *.obj *.hex *.elf *.i *.s *.eep
	
##
# Add an explicit target for firmware binaries that will be versioned
# 
# regular builds with 'make' will not be tracked, to build a firmware file
# that is tracked by the VCS, run 'make dist'
# to flash this firmware, run 'make program_dist'
##

.PHONY: dist program_dist
dist: dist/$(TARGET).hex dist/$(TARGET).eep
	
program_dist:# dist/$(TARGET).hex dist/$(TARGET).eep
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c $(PROGRAMMER) -P $(PORT) -U flash:w:dist/$(TARGET).hex -U eeprom:w:dist/$(TARGET).eep -B 40
	
dist/%.eep: %.elf
	 avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex $< $@

dist/%.hex: %.elf
	avr-objcopy -O ihex -R .eeprom $< $@ 

%.eep: %.elf
	 avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex $< $@

%.hex: %.elf
	avr-objcopy -O ihex -R .eeprom $< $@ 

%.elf: $(OBJ)
	avr-gcc $(CFLAGS) $(OBJ) --output $@ $(LDFLAGS)

%.obj: %.o
	$(CC) $(CFLAGS) $< $(LDFLAGS) -o $@

#%.o: %.c
	#$(CC) $(CFLAGS) $< -o $@

program: $(TARGET).hex $(TARGET).eep
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c $(PROGRAMMER) -P $(PORT) -U flash:w:$(TARGET).hex -U eeprom:w:$(TARGET).eep -B 40
	
memory-used:$(TARGET).elf
	avr-size --mcu=$(MCU) -C $(TARGET).elf
#	avr-nm -S	$(OBJ)
