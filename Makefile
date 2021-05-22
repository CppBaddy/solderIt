CC=avr-gcc

CFLAGS=-g -std=gnu99 -Os -Wall -mcall-prologues -mmcu=attiny85 -DF_CPU=8000000
## Use short (8-bit) data types
##CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums

LDFLAGS = -Wl,-Map,$(TARGET).map
## Optional, but often ends up with smaller code
LDFLAGS += -Wl,--gc-sections

OBJ2HEX=avr-objcopy

UISP=avrdude

TARGET=solderIt

C_FILES = $(wildcard *.c)

OBJS = $(C_FILES:.c=.o)


program : flash eeprom

flash : $(TARGET).hex
	sudo $(UISP) -p t85 -c USBasp -v -U flash:w:$(TARGET).hex:i

eeprom : $(TARGET).eep
	sudo $(UISP) -p t85 -c USBasp -v -U eeprom:w:$(TARGET).eep:i

build : $(TARGET).hex $(TARGET).eep
	ls -l $(TARGET).*

%.hex : %.elf
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

%.eep : %.elf
	$(OBJ2HEX) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@


%.elf : $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@

%.o : %.c Makefile
	$(CC) $(CFLAGS) -c $< -o $@

clean :
	rm -f *.hex *.eep *.elf *.o
