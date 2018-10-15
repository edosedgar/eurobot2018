#######################################################################
# Makefile for STM32F4 Discovery board projects

OUTPATH = build
PROJECT = $(OUTPATH)/robot
OPENOCD_SCRIPT_DIR ?= /usr/local/Cellar/open-ocd/0.10.0/share/openocd/scripts
HEAP_SIZE = 0x500

################
# Sources

SOURCES_S = core/startup_stm32f407xx.s

SOURCES_RTOS = $(wildcard freertos/*.c freertos/portable/GCC/ARM_CM4F/*.c)
SOURCES_RTOS += $(wildcard freertos/portable/MemMang/*.c)

SOURCES_C = $(wildcard *.c core/*.c plib/*.c lib/*.c old/*.c)
SOURCES_C += $(SOURCES_RTOS)

SOURCES = $(SOURCES_S) $(SOURCES_C)
OBJS = $(SOURCES_S:.s=.o) $(SOURCES_C:.c=.o)

# Includes and Defines

INCLUDES_RTOS = -Ifreertos -Ifreertos/include -Ifreertos/portable/GCC/ARM_CM4F
INCLUDES = -Icore -Iplib -Ilib -Iold
INCLUDES += $(INCLUDES_RTOS)

DEFINES = -DSTM32 -DSTM32F4 -DSTM32F407xx -DHEAP_SIZE=$(HEAP_SIZE)

# Compiler/Assembler/Linker/etc

PREFIX = arm-none-eabi

CC = $(PREFIX)-gcc
AS = $(PREFIX)-as
AR = $(PREFIX)-ar
LD = $(PREFIX)-gcc
NM = $(PREFIX)-nm
OBJCOPY = $(PREFIX)-objcopy
OBJDUMP = $(PREFIX)-objdump
READELF = $(PREFIX)-readelf
SIZE = $(PREFIX)-size
GDB = $(PREFIX)-gdb
RM = rm -f
OPENOCD=openocd

# Compiler options

MCUFLAGS = -mcpu=cortex-m4 -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16\
	   -mthumb -fsingle-precision-constant -mno-unaligned-access

DEBUG_OPTIMIZE_FLAGS = -O0 -ggdb -gdwarf-2

CFLAGS = -Wall -Wextra --pedantic
CFLAGS_EXTRA = -nostartfiles -nodefaultlibs -nostdlib\
	       -fdata-sections -ffunction-sections

CFLAGS += $(DEFINES) $(MCUFLAGS) $(DEBUG_OPTIMIZE_FLAGS) $(CFLAGS_EXTRA) $(INCLUDES)

LDFLAGS = -static $(MCUFLAGS) -Wl,--start-group -lgcc -lc -lg -Wl,--end-group \
	  -Wl,--gc-sections -T STM32F407VGTx_FLASH.ld -specs=nano.specs \
	  -u _printf_float
	  #-u _scanf_float

.PHONY: dirs all clean flash erase

all: dirs $(PROJECT).bin $(PROJECT).asm

dirs: ${OUTPATH}

${OUTPATH}:
	mkdir -p ${OUTPATH}

clean:
	$(RM) $(OBJS) $(PROJECT).elf $(PROJECT).bin $(PROJECT).asm

# Hardware specific

flash: $(PROJECT).bin
	st-flash write $(PROJECT).bin 0x08000000

erase:
	st-flash erase

gdb-server-ocd:
	$(OPENOCD) -f $(OPENOCD_SCRIPT_DIR)/interface/stlink-v2.cfg \
		   -f $(OPENOCD_SCRIPT_DIR)/board/stm32f4discovery.cfg

gdb-server-st:
	st-util

OPENOCD_P=3333
gdb-openocd: $(PROJECT).elf
	$(GDB) --eval-command="target extended-remote localhost:$(OPENOCD_P)"\
	       --eval-command="monitor halt" $(PROJECT).elf

GDB_P=4242
gdb-st-util: $(PROJECT).elf
	$(GDB) --eval-command="target extended-remote localhost:$(GDB_P)"\
	       --eval-command="monitor halt" $(PROJECT).elf

$(PROJECT).elf: $(OBJS)

%.elf:
	$(LD) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) -A $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.asm: %.elf
	$(OBJDUMP) -dwh $< > $@
