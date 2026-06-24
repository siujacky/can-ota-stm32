# Makefile.bl — Build rules for STM32F103 MCP2515 CAN bootloader
# No STM32F1 HAL or CMSIS device headers required.
# Produces: bootloader.elf, bootloader.bin
# Size target: < 8192 bytes

TARGET  = bootloader
PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

MCU     = -mcpu=cortex-m3 -mthumb

CFLAGS  = $(MCU) \
          -Os \
          -Wall \
          -Wextra \
          -std=gnu11 \
          -ffunction-sections \
          -fdata-sections \
          -nostdlib \
          -ffreestanding \
          -fno-builtin

LDFLAGS = $(MCU) \
          -T bootloader.ld \
          -nostdlib \
          -Wl,--gc-sections \
          -Wl,-Map=$(TARGET).map

SRCS    = startup_bl.s \
          bxcan_bl.c \
          usart_bl.c \
          flash_bl.c \
          main_bl.c

OBJS    = $(SRCS:.s=.o)
OBJS   := $(OBJS:.c=.o)

.PHONY: all clean size

all: $(TARGET).bin size

%.o: %.s
	$(CC) $(MCU) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

size: $(TARGET).bin $(TARGET).elf
	@echo "--- ELF section sizes ---"
	$(SIZE) $(TARGET).elf
	@echo ""
	@BINSIZE=$$(wc -c < $(TARGET).bin); \
	echo "bootloader.bin size: $${BINSIZE} bytes"; \
	if [ $${BINSIZE} -gt 8192 ]; then \
	    echo "ERROR: Binary exceeds 8192 bytes by $$(( $${BINSIZE} - 8192 )) bytes!"; \
	    exit 1; \
	else \
	    echo "OK: fits in 8KB flash (margin: $$(( 8192 - $${BINSIZE} )) bytes)"; \
	fi

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin $(TARGET).map
