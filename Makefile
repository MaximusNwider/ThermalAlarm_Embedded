PROJECT=thermal_alarm_portspec
TC?=arm-none-eabi
CC=$(TC)-gcc
LD=$(TC)-gcc
OBJCOPY=$(TC)-objcopy
CFLAGS=-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O2 -ffunction-sections -fdata-sections -Wall -Wextra -std=c11
LDFLAGS=-T tm4c123gh6pm.ld -Wl,--gc-sections -nostartfiles -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
INCS=-I. -Ibsp -Idrivers -Iapp
SRCS=startup_gcc.c bsp/clock.c bsp/systick.c bsp/leds.c drivers/uart0.c drivers/adc_pe4.c drivers/dac_r2r.c app/alarm.c app/logger.c main.c
OBJDIR=build
OBJS=$(SRCS:%=$(OBJDIR)/%.o)

all: $(OBJDIR)/firmware.elf $(OBJDIR)/firmware.bin
$(OBJDIR):
	mkdir -p $(OBJDIR)/bsp $(OBJDIR)/drivers $(OBJDIR)/app
$(OBJDIR)/%.o: %.c | $(OBJDIR)
	$(CC) $(CFLAGS) $(INCS) -c $< -o $@
$(OBJDIR)/firmware.elf: $(OBJS)
	$(LD) $(LDFLAGS) $^ -o $@
$(OBJDIR)/firmware.bin: $(OBJDIR)/firmware.elf
	$(OBJCOPY) -O binary $< $@
clean:
	rm -rf $(OBJDIR)

flash: $(OBJDIR)/firmware.bin
	@echo "Flash build/firmware.bin with lm4flash/OpenOCD/Uniflash"

.PHONY: all clean flash
