#pragma once

#include <core/Log.h>
#include <types/Str.h>

#include "6502.h"

static inline Log::Channel nesChan = { "NES" };

struct NES {
	cpu6502 cpu;
	Memory mem;
	bool test_rom;

	inline void init() {
		cpu.init();

		cpu.pinout.resN = false;
		LOG(Log::INFO, nesChan, "Init");
		for (int i = 0; i < 20; i++)
			cpu.tick();

		mem.debug_set_all_mem(0xea);
		mem.debug_setmem(0xfffc, 0x00);
		mem.debug_setmem(0xfffd, 0x01);
		mem.debug_setmem(0x0100, 0xA9);
		mem.debug_setmem(0x0101, 42);

		if (test_rom) {
			load_rom("roms/6502_functional_test.bin");

			for (int i = 0; i < 1; ++i)
				cpu.tick();
			cpu.pc = 0x0400; // test rom start addr. Resets are trapped
			cpu.clear_uop_queue();
		}
	}

	inline void tick() {
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

	inline void load_rom(str path) {
#pragma warning (disable : 4996)
		std::FILE* f = fopen(path.s, "r");
		ASSERT(f, "Could not open file %s for reading", path.s);

		size_t ret = fread(mem.memory, 1, 64*1024, f);
		ASSERT(ret > 0, "Could not read any bytes from file");
	}
};

