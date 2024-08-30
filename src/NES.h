#pragma once

#include "6502.h"

struct NES {
	cpu6502 cpu;
	Memory mem;

	void init() {
		cpu.pinout.resN = false;
		for (int i = 0; i < 20; i++)
			cpu.tick();
		mem.debug_setmem(0xfffc, 0x00);
		mem.debug_setmem(0xfffd, 0x01);
		mem.debug_setmem(0x0100, 0xA9);
		mem.debug_setmem(0x0101, 42);
	}

	void tick() {
		cpu.pinout.resN = true;
		cpu.tick();

		mem.pinout.a = cpu.pinout.a;
		mem.tick();

		mem.pinout.rw = cpu.pinout.rw;
		if (cpu.pinout.rw == RW_READ)
			cpu.pinout.d = mem.pinout.d;
		else
			mem.pinout.d = cpu.pinout.d;
	}
};

