PORT ?= /dev/spidev0.0
TARGET=main

LIB=lib


DEVICE = attiny2313a
MCU = attiny2313a
AVRDUDE_DEVICE = attiny2313
AVRDUDE_INTERFACE = linuxspi

OBJ=$(TARGET).o

OPT=-Os
CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) $(DEVICE_SPECIFIC_CFLAGS) $(OPT) -std=c99 -flto
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
dist: dist/$(TARGET).hex
	
program_dist: dist/$(TARGET).hex
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c $(AVRDUDE_INTERFACE) -P $(PORT) -U flash:w:$< -B 40
	

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

program: $(TARGET).hex
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c $(AVRDUDE_INTERFACE) -P $(PORT) -U flash:w:$(TARGET).hex -B 40
	
memory-used:$(TARGET).elf
	avr-size --mcu=$(MCU) -C $(TARGET).elf
#	avr-nm -S	$(OBJ)
