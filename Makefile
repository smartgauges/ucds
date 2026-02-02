LIBOPENCM3_DIR = libopencm3
BUILD_DIR = build

PREFIX=arm-none-eabi
CC=$(PREFIX)-gcc
OBJCOPY=$(PREFIX)-objcopy
AR=$(PREFIX)-ar

COMMON_FLAGS=-mcpu=cortex-m3 -mthumb -mfix-cortex-m3-ldrd -msoft-float -Wall -ggdb3
CFLAGS=$(COMMON_FLAGS) -Os -std=gnu99 -I. \
       -I$(LIBOPENCM3_DIR)/include -I$(LIBOPENCM3_DIR)/lib/ -I$(LIBOPENCM3_DIR)/lib/usb -DSTM32F1 -fno-common \
       -ffunction-sections -fdata-sections \
       -Wimplicit-function-declaration -Wmissing-prototypes -Wstrict-prototypes -Wundef -Wextra -Wshadow -Wredundant-decls
LDFLAGS=$(COMMON_FLAGS) -lc -nostartfiles -Wl,--gc-sections -Wl,--print-memory-usage #-Wl,--print-gc-sections

# --- Object File Definitions ---

# Local objects shared by most targets
LOCAL_OBJS := sbrk.o led.o clock.o tick.o ring.o timer.o adc.o can.o usb.o
LOCAL_OBJS := $(addprefix $(BUILD_DIR)/, $(LOCAL_OBJS))

# libopencm3 objects
LIBOPENCM3_OBJS := $(LIBOPENCM3_DIR)/lib/stm32/f1/gpio.o \
		$(LIBOPENCM3_DIR)/lib/stm32/f1/rcc.o \
		$(LIBOPENCM3_DIR)/lib/stm32/f1/timer.o \
		$(LIBOPENCM3_DIR)/lib/stm32/f1/flash.o \
		$(LIBOPENCM3_DIR)/lib/stm32/f1/adc.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/adc_common_v1.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/gpio_common_all.o \
 		$(LIBOPENCM3_DIR)/lib/stm32/common/usart_common_all.o \
 		$(LIBOPENCM3_DIR)/lib/stm32/common/usart_common_f124.o \
 		$(LIBOPENCM3_DIR)/lib/stm32/common/rcc_common_all.o \
 		$(LIBOPENCM3_DIR)/lib/stm32/common/i2c_common_v1.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/pwr_common_v1.o \
		$(LIBOPENCM3_DIR)/lib/cm3/vector.o \
		$(LIBOPENCM3_DIR)/lib/cm3/systick.o \
 		$(LIBOPENCM3_DIR)/lib/cm3/nvic.o \
 		$(LIBOPENCM3_DIR)/lib/cm3/sync.o \
 		$(LIBOPENCM3_DIR)/lib/cm3/assert.o \
 		$(LIBOPENCM3_DIR)/lib/cm3/scb.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/timer_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/flash_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/flash_common_f01.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/flash_common_f.o \
		$(LIBOPENCM3_DIR)/lib/stm32/can.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/desig_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/desig_common_v1.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/iwdg_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/exti_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/spi_common_all.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/spi_common_v1.o \
		$(LIBOPENCM3_DIR)/lib/stm32/common/st_usbfs_core.o \
		$(LIBOPENCM3_DIR)/lib/stm32/st_usbfs_v1.o \
		$(LIBOPENCM3_DIR)/lib/usb/usb.o \
		$(LIBOPENCM3_DIR)/lib/usb/usb_f107.o \
		$(LIBOPENCM3_DIR)/lib/usb/usb_dwc_common.o \
		$(LIBOPENCM3_DIR)/lib/usb/usb_control.o \
		$(LIBOPENCM3_DIR)/lib/usb/usb_standard.o

LIBOPENCM3_OBJS := $(addprefix $(BUILD_DIR)/, $(LIBOPENCM3_OBJS))

.PHONY: all clean dfu flash-bl

all: gs.bin slcan.bin slcan-win10.bin ch.bin ucds.bin jlrm.bin bl.bin dfu

# --- Compilation Rules ---

# Generic rule for .o files
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# Special case for win10 usb-serial hack
$(BUILD_DIR)/%-win10.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ -DOSWIN10

# --- Binary Linking Rules ---

# Rule for standard app binaries
%.bin: $(BUILD_DIR)/main_%.o $(BUILD_DIR)/usb_%.o $(LOCAL_OBJS) $(LIBOPENCM3_OBJS)
	$(CC) $^ -L$(LIBOPENCM3_DIR)/lib -Tstm32f105-app.ld $(LDFLAGS) -Wl,-Map=$(BUILD_DIR)/$*.map -o $(BUILD_DIR)/$*.elf
	$(OBJCOPY) -O binary $(BUILD_DIR)/$*.elf $@

# Special rule for the bootloader
bl.bin: $(BUILD_DIR)/bl.o $(BUILD_DIR)/usb_dfu.o $(LOCAL_OBJS) $(LIBOPENCM3_OBJS)
	$(CC) $^ -L$(LIBOPENCM3_DIR)/lib -Tstm32f105-bl.ld $(LDFLAGS) -Wl,-Map=$(BUILD_DIR)/bl.map -o $(BUILD_DIR)/bl.elf
	$(OBJCOPY) -O binary $(BUILD_DIR)/bl.elf $@

# --- Utility Targets ---

flash-bl: bl.bin
	#openocd -f stm32f1x.cfg -c "init; reset halt; targets; flash banks; stm32f1x unlock 0; reset; exit"
	openocd -f stm32f1x.cfg -c "cmsis_dap_backend hid" -c "init; reset halt; program bl.bin 0x8000000 verify; reset; exit"

flash-%: %.bin
	dfu-util -s 0x08004000:leave -D $<

dfu: gs.dfu slcan.dfu slcan-win10.dfu ch.dfu ucds.dfu jlrm.dfu

%.dfu: %.bin
	./dfu-convert -b 0x8004000:$< $@

clean:
	rm -rf $(BUILD_DIR) *.bin *.dfu
