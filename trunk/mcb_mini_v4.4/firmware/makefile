PRG            = mcb_mini
OBJ            = $(PRG).o

MCU_TARGET     = atmega168
OPTIMIZE       = -Os

DEFS           =
LIBS           =

# You should not have to change anything below here.

CC             = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall -Wstrict-prototypes $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -std=c99
override LDFLAGS       = -Wl,-Map,$(PRG).map,--cref

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst hex

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

# dependency:
$(PRG).o: 	$(PRG).c $(PRG).h circBuffer.h controller.h
controller.h:		motor.h 	pid.h	 

clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak 
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# for programming

AVRDUDE_FLAGS = -p $(MCU_TARGET) -P usb -c avrispmkII 

program: $(PRG).hex
	avrdude $(AVRDUDE_FLAGS) -U flash:w:$(PRG).hex

LFUSE = 0xCE
#HFUSE = 0xDC
HFUSE = 0xCC // Watch Dog Timer on
EFUSE = 0xF9
AVRDUDE_LFUSE = -U lfuse:w:$(LFUSE):m
AVRDUDE_HFUSE = -U hfuse:w:$(HFUSE):m
AVRDUDE_EFUSE = -U efuse:w:$(EFUSE):m

fuse: 
	avrdude $(AVRDUDE_FLAGS) $(AVRDUDE_LFUSE) 
	sleep 1
	avrdude $(AVRDUDE_FLAGS) $(AVRDUDE_HFUSE) 

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }

# Every thing below here is used by avr-libc's build system and can be ignored
# by the casual user.

FIG2DEV                 = fig2dev
EXTRA_CLEAN_FILES       = *.hex *.bin *.srec

dox: eps png pdf

eps: $(PRG).eps
png: $(PRG).png
pdf: $(PRG).pdf

%.eps: %.fig
	$(FIG2DEV) -L eps $< $@

%.pdf: %.fig
	$(FIG2DEV) -L pdf $< $@

%.png: %.fig
	$(FIG2DEV) -L png $< $@

