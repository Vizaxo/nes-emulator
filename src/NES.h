#pragma once

#include <core/Log.h>
#include <types/Str.h>

#include "Memory.h"
#include "6502.h"

static inline Log::Channel nesChan = { "NES" };

struct NES {
	cpu6502 cpu;
	CPUMemory mem;
	bool test_rom = false;

	inline void reset() {
		cpu.init();

		cpu.pinout.resN = false;
		LOG(Log::INFO, nesChan, "Init");
		for (int i = 0; i < 20; i++)
			cpu.tick();

		mem.debug_set_all_mem(0xea);

		load_nes_rom("../../../../games/roms/nes/Super Mario Bros. (Japan, USA).nes");
	}

	inline void tick() {
		cpu.pinout.resN = true;
		cpu.fetching = false;
		cpu.tick();

		mem.pinout.a = cpu.pinout.a;
		mem.pinout.rw = cpu.pinout.rw;
		mem.tick();

		if (cpu.pinout.rw == RW_READ)
			cpu.pinout.d = mem.pinout.d;
		else
			mem.pinout.d = cpu.pinout.d;

		mem.tick();
	}

	enum mapper_t {
		mapper0 = 0,
	};

	// Read INES rom format
	inline void load_nes_rom(str path) {
#pragma warning (disable : 4996)
		std::FILE* f = fopen(path.s, "rb");
		ASSERT(f, "Could not open file %s", path.s);
		u8 header[16];
		ASSERT(fread(header, 1, 16, f) == 16, "Could not read 16 bytes of header");
		u8 prg_rom_size = header[4]; // *16K
		u8 chr_rom_size = header[5]; // *8k
		mapper_t mapper = (mapper_t)((header[6]&0xf0)>>4 | header[7]&0xf0);

		switch (mapper) {
		case mapper0:
			ASSERT(prg_rom_size > 0 && prg_rom_size <= 2, "Mapper0 rom must be 16K or 32K");
			for (int i = 0; i < prg_rom_size; ++i) {
				size_t bytes_read = fread(mem.cartridge_rom.memory + (0x8000 - Memory::CARTRIDGE_ROM_START) + i*0x4000, 1, 0x4000, f);
				ASSERT(bytes_read = 0x4000, "Could not read 16K from ROM");
			}
			// TODO: load CHR ROM
			break;
		default:
			ASSERT(false, "Unimplemented mapper %d", mapper);
		}
	}
};

