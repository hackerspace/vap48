PORT=usb #/dev/ttyUSB0
MCU=atmega8
CC=avr-gcc
OBJCOPY=avr-objcopy
PROJECT=main
ISINT=INTERRUPT
F_CPU=14745600
#1000000

# optimize for size:
CFLAGS=-mmcu=$(MCU) -Wall -Wstrict-prototypes -Os -mcall-prologues #-Wl,-u,vfprintf -lprintf_flt -lm
#-------------------
all: $(PROJECT).hex
#-------------------
$(PROJECT).hex : $(PROJECT).out
	avr-strip $(PROJECT).out
	$(OBJCOPY) -R .eeprom -O ihex $(PROJECT).out $(PROJECT).hex
$(PROJECT).out : $(PROJECT).o
	$(CC) $(CFLAGS) -o $(PROJECT).out -Wl,-Map,$(PROJECT).map $(PROJECT).o
$(PROJECT).o : $(PROJECT).c
	$(CC) $(CFLAGS) -Os -c $(PROJECT).c -DF_CPU=$(F_CPU)
$(PROJECT).s : $(PROJECT).c
	$(CC) $(CFLAGS) -Os -S $(PROJECT).c
# you need to erase first before loading the program.
# load (program) the software into the eeprom:
load: $(PROJECT).hex
	avrdude -p $(MCU) -c usbasp -P $(PORT) -b 19200 -U flash:w:$(PROJECT).hex:a
#-------------------
clean:
	rm -f *.o *.map *.out *.s
#-------------------
