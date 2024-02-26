LIBOPENCM3_DIR = ../libopencm3

PREFIX=arm-none-eabi
CC=$(PREFIX)-gcc
OBJCOPY=$(PREFIX)-objcopy
AR=$(PREFIX)-ar

CFLAGS=-Os -fno-tree-loop-distribute-patterns

COMMON_FLAGS=-mcpu=cortex-m3 -mthumb -mfix-cortex-m3-ldrd -msoft-float #-Wall -ggdb3
CFLAGS=$(COMMON_FLAGS) -Os -std=gnu99 -fno-tree-loop-distribute-patterns -I.. \
	   -I$(LIBOPENCM3_DIR)/include -DSTM32F1 -fno-common \
       -ffunction-sections -fdata-sections \
       #-Wimplicit-function-declaration -Wmissing-prototypes -Wstrict-prototypes -Wundef -Wextra -Wshadow -Wredundant-decls
LDFLAGS=$(COMMON_FLAGS) -lc -nostartfiles -nostdlib -Wl,--gc-sections -Wl,--print-memory-usage #-Wl,--print-gc-sections

PHONY:all

all: sram.bin attack.bin

%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o : %.S
	$(CC) $(CFLAGS) -c $< -o $@

LIBOPENCM3_OBJS = $(LIBOPENCM3_DIR)/lib/stm32/f1/gpio.o \
		$(LIBOPENCM3_DIR)/lib/stm32/f1/rcc.o \
		$(LIBOPENCM3_DIR)/lib/stm32/f1/timer.o \
		$(LIBOPENCM3_DIR)/lib/stm32/f1/flash.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/gpio_common_all.o \
 		$(LIBOPENCM3_DIR)/lib/stm32/common/usart_common_all.o \
 		$(LIBOPENCM3_DIR)/lib/stm32/common/usart_common_f124.o \
 		$(LIBOPENCM3_DIR)/lib/stm32/common/rcc_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/pwr_common_v1.o \
 		$(LIBOPENCM3_DIR)/lib/cm3/sync.o \
 		$(LIBOPENCM3_DIR)/lib/cm3/assert.o \
 		$(LIBOPENCM3_DIR)/lib/cm3/scb.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/timer_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/flash_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/flash_common_f01.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/flash_common_f.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/desig_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/desig_common_v1.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/iwdg_common_all.o

sram.bin: dump.o sram.o $(LIBOPENCM3_OBJS)
	$(CC) $^ -L$(LIBOPENCM3_DIR)/lib -Tsram.ld $(LDFLAGS) -Wl,-Map=sram.map -o sram.elf
	$(OBJCOPY) -O binary sram.elf $@

attack.bin: attack.o ../timer.o $(LIBOPENCM3_OBJS) $(LIBOPENCM3_DIR)/lib/cm3/vector.o
	$(CC) $^ -L$(LIBOPENCM3_DIR)/lib -Tstm32f103.ld $(LDFLAGS) -Wl,-Map=attack.map -o attack.elf
	$(OBJCOPY) -O binary attack.elf $@

load: sram.elf
	openocd -f ../stm32f1x.cfg -c "cmsis_dap_backend hid" -c "init; reset halt; targets" -c "load_image sram.elf;exit"

flash: attack.bin
	openocd -f ../stm32f1x.cfg -c "cmsis_dap_backend hid" -c "init; reset halt; program attack.bin 0x8000000 verify; reset; exit"

lock:
	openocd -f ../stm32f1x.cfg -c "cmsis_dap_backend hid" -c "init; reset halt; stm32f1x lock 0; reset halt;exit"

unlock:
	openocd -f ../stm32f1x.cfg -c "cmsis_dap_backend hid" -c "init; reset halt; stm32f1x unlock 0; reset halt; exit"

read:
	openocd -f ../stm32f1x.cfg -c "cmsis_dap_backend hid" -c init -c "reset halt" -c "flash read_bank 0 read.bin 0 0x4000" -c "reset" -c shutdown

clean:
	rm -rf *.bin *.bin.* *.map $(LIBOPENCM3_OBJS) *.o *.d *.elf

