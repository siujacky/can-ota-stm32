# Makefile — Blue Pill CAN Tool
# Target: STM32F103C6T6 (32KB flash, 10KB SRAM)
# Toolchain: arm-none-eabi-gcc
# No HAL, no CMSIS device headers, no interrupts, 8MHz HSI.

PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc  # use gcc to assemble (handles preprocessing)
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

MCU     = -mcpu=cortex-m3 -mthumb

CFLAGS  = $(MCU) -O2 -Wall -Wextra -std=gnu11 \
          -ffunction-sections -fdata-sections \
          -nostdlib -ffreestanding -fno-builtin \
          -Wno-unused-parameter

ASFLAGS = $(MCU) -x assembler-with-cpp

LDFLAGS = $(MCU) -T linker.ld -nostdlib \
          -Wl,--gc-sections \
          -Wl,-Map=blue_pill_can_tool.map

TARGET  = blue_pill_can_tool

# Source files (startup.s must come first to ensure vector table is at offset 0)
SRCS_C  = main.c bxcan.c mcp2515.c usart.c slcan.c
SRCS_S  = startup.s

OBJS_C  = $(SRCS_C:.c=.o)
OBJS_S  = $(SRCS_S:.s=.o)
OBJS    = $(OBJS_S) $(OBJS_C)

# ---- Default target ----
all: $(TARGET).bin size_check

$(TARGET).elf: $(OBJS) linker.ld
	$(CC) $(LDFLAGS) $(OBJS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# Compile C
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble startup
startup.o: startup.s
	$(CC) $(ASFLAGS) -c $< -o $@

# ---- Size check ----
size_check: $(TARGET).elf $(TARGET).bin
	@echo ""
	@$(SIZE) $(TARGET).elf
	@echo ""
	@BIN_SIZE=$$(wc -c < $(TARGET).bin); \
	 BSS_SIZE=$$($(SIZE) $(TARGET).elf | awk 'NR==2{print $$3}'); \
	 echo "Flash (.bin): $$BIN_SIZE bytes (limit 32768)"; \
	 echo "BSS:          $$BSS_SIZE bytes (limit 10240)"; \
	 if [ $$BIN_SIZE -gt 32768 ]; then \
	     echo "ERROR: binary exceeds 32KB flash!"; exit 1; \
	 fi; \
	 if [ $$BSS_SIZE -gt 10240 ]; then \
	     echo "ERROR: BSS exceeds 10KB SRAM!"; exit 1; \
	 fi; \
	 echo "Size check PASSED."

# ---- Flash target (requires st-flash) ----
flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x08000000

# ---- Clean ----
clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin $(TARGET).map

.PHONY: all size_check flash clean
