#pragma once

#include <types/str.h>
#include <cstdio>
#include <stdio.h>

#include "6502.h"
#include "Memory.h"

// TODO: make this standalone compatible
struct test_6502_t {
	cpu6502 cpu;
	RAM<0x10000> mem;

	inline void load_raw_rom(str path) {
#pragma warning (disable : 4996)
		std::FILE* f = fopen(path.s, "rb");
		ASSERT(f, "Could not open file %s for reading", path.s);

		size_t ret = fread(mem.memory, 1, 64*1024, f);
		ASSERT(ret > 0, "Could not read any bytes from file");
	}

	void init() {
		load_raw_rom("roms/6502_functional_test.bin");

		for (int i = 0; i < 1; ++i)
			cpu.tick();
		cpu.pc = 0x0400; // test rom start addr. Resets are trapped
		cpu.clear_uop_queue();
	}
};
