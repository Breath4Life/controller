PRG            = main

SOURCES_DIR    = src
SRC            = $(shell find src/ -name '*.c') #$(wildcard $(SOURCES_DIR)/*.c)
OBJ = $(subst src/,obj/src/,$(subst .c,.o,$(SRC)))
OBJ_CPP =

FREERTOS_DIR = lib/miniAVRfreeRTOS
FREERTOS_SRC       = $(wildcard $(FREERTOS_DIR)/*.c)
OBJ += $(subst $(FREERTOS_DIR)/,obj/$(FREERTOS_DIR)/,$(subst .c,.o,$(FREERTOS_SRC)))

MCU_TARGET     = atmega2560

OPTIMIZE       = -Os
LIBS           = 
DEFS           = -D ARDUINO=100 -D F_CPU=16000000UL
CC             = avr-gcc
CXX			   = avr-g++

COMMON_FLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -Isrc/ -I$(FREERTOS_DIR) -DRT_B4L -flto
override CFLAGS        = $(COMMON_FLAGS) -std=gnu11
override CPPFLAGS      = $(COMMON_FLAGS)
override LDFLAGS       = -Wl,--gc-sections,-Map,$(PRG).map -fuse-linker-plugin -fno-exceptions -ffunction-sections -fdata-sections -MMD 

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst text eeprom

print-%  : ; @echo $* = $($*)

$(PRG).elf: $(OBJ) $(OBJ_CPP)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

$(OBJ): obj/%.o : %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) $(LDFLAGS) -c $< -o $@ $(LIBS)

$(OBJ_CPP): obj/%.o : %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(LDFLAGS) -c $< -o $@ $(LIBS)

clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)
	rm -rf obj

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

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

flash:
	@tools/flash.sh

serial:
	screen /dev/ttyACM0 19200

.PHONY: flash serial
