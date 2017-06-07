PORT ?= /dev/ttyACM0
TARGET=main

LIB=lib


DEVICE = attiny2313a
MCU = attiny2313a
AVRDUDE_DEVICE = attiny2313

OBJ=$(TARGET).o

OPT=-Os
CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) $(DEVICE_SPECIFIC_CFLAGS) $(OPT) -std=c99
CC=avr-gcc
LDFLAGS=-Wl,-gc-sections -Wl,-relax

#-lpololu_$(DEVICE) 

AVRDUDE=avrdude

#.PRECIOUS: %.elf

all:$(TARGET).hex memory-used
	
clean:
	rm -f *.o *.hex *.obj *.hex *.elf *.i *.s *.eep

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
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c stk500v2 -P $(PORT) -U flash:w:$(TARGET).hex -B 40
	
memory-used:$(TARGET).elf
	avr-size --mcu=$(MCU) -C $(TARGET).elf
#	avr-nm -S	$(OBJ)
