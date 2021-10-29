include config.mk

PROJ_NAME := app

SRC := $(wildcard src/*.c)
ASRC := startup_stm32f411xe.s

OBJ := $(patsubst %.c, %.o, $(SRC)) $(patsubst %.s, %.o, $(ASRC))

PROJ := $(PROJ_NAME).hex

ELF := $(PROJ_NAME).elf

LDSCRIPT := STM32F411VETx_FLASH.ld

LDFLAGS += -T$(LDSCRIPT)

.PHONY: all clean

all: $(PROJ)
	echo $(PROJ)

%.hex: %.elf
	$(BIN) $< $@

$(ELF): $(OBJ) $(LDSCRIPT)
	$(CC) $(OBJ) -Wl,-Map=app.map $(LDFLAGS) -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@

%.o: %.s
	$(AS) -c $(ASFLAGS) $< -o $@

clean:
	rm $(PROJ)
	rm $(ELF)
	rm $(PROJ_NAME).map
	rm $(OBJ)