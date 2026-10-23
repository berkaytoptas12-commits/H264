# Linux build of the bridge firmware: TAP device instead of the MSS MAC,
# host/fpga_emu.py instead of the fabric.  Run as root (TAP):
#   make && sudo ./kit_sim            kit at 192.168.77.2, PC at 192.168.77.1
SRC  := ../src
CC   ?= cc
CFLAGS ?= -O2 -g -Wall -Wextra -Werror
CFLAGS += -I$(SRC) -I. -DAPP_IP='IP4(192,168,77,2)' \
          -DEMU_PATH='"$(abspath ../../host/fpga_emu.py)"'

OBJS := $(SRC)/net.c $(SRC)/bridge.c $(SRC)/service.c $(SRC)/log.c $(SRC)/kit_main.c \
        port_tap.c fabric_sim.c main_sim.c

kit_sim: $(OBJS) $(wildcard $(SRC)/*.h) sim.h
	$(CC) $(CFLAGS) -o $@ $(OBJS)

# whole chain co-simulation: the firmware's fabric driver against the RTL in
# GHDL (tb/tb_cosim.vhd loads libh264cosim.so); see cosim.sh
COSIM_OBJS := $(SRC)/net.c $(SRC)/bridge.c $(SRC)/service.c $(SRC)/log.c $(SRC)/kit_main.c \
              $(SRC)/fabric_mmio.c port_tap.c fabric_cosim.c main_sim.c

kit_cosim: $(COSIM_OBJS) $(wildcard $(SRC)/*.h) sim.h
	$(CC) $(CFLAGS) -DFABRIC_SIM_REGS -o $@ $(COSIM_OBJS)

libh264cosim.so: cosim_vhpi.c
	$(CC) $(CFLAGS) -shared -fPIC -o $@ $<

cosim: kit_cosim libh264cosim.so

clean:
	rm -f kit_sim kit_cosim libh264cosim.so

.PHONY: clean cosim
