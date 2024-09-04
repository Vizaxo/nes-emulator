#pragma once

#include <core/Log.h>
#include <types/Str.h>

#include "Memory.h"
#include "6502.h"
#include "PPU.h"
#include "Palette.h"

static inline Log::Channel nesChan = { "NES" };

namespace flags_6 {
	enum flags_6 : u8 {
		nametable = 1 << 0,
		battery_backed_prg_ram = 1 << 1,
		trainer = 1 << 2,
		alt_nametable = 1 << 3,
		mapper_low = 0xf0,
	};
}

namespace flags_7 {
	enum flags_7 : u8 {
		vs_unisystem = 1 << 0,
		playchoice_10 = 1 << 1,
		nes2_0 = 0x0c,
		mapper_high = 0xf0,
	};
}

struct NES {
	cpu6502 cpu;
	PPU ppu;
	CPUMemory mem;
	PPUMemory ppu_mem;
	palette_t palette;
	bool test_rom = false;

	inline void reset() {
		//mem.debug_set_all_mem(0xea, ppu_mem);
		load_nes_rom("../../../../games/roms/nes/Super Mario Bros. (Japan, USA).nes");

		cpu.init();

		// TODO: this is clearly broken somewhere
#if _DEBUG
		ppu.fine_x_shr = 5;
#else
		ppu.fine_x_shr = 7;
#endif

		cpu.pinout.resN = false;
		LOG(Log::INFO, nesChan, "Init");
		for (int i = 0; i < 20; i++)
			cpu.tick();

		palette = palette_t::load_palette("resources/ntscpalette.pal");
	}

	inline void tick() {
		cpu.pinout.resN = true;
		cpu.fetching = false;
		cpu.tick();
		ppu.tick(palette, cpu, mem, ppu_mem);
		ppu.tick(palette, cpu, mem, ppu_mem);
		ppu.tick(palette, cpu, mem, ppu_mem);


		mem.pinout.a = cpu.pinout.a;
		mem.pinout.rw = cpu.pinout.rw;

		if (cpu.pinout.rw == RW_READ)
			cpu.pinout.d = mem.pinout.d;
		else if (cpu.pinout.rw == RW_WRITE)
			mem.pinout.d = cpu.pinout.d;
		else
			mem.pinout.a = 0x0000;

		mem.tick(ppu_mem);

		if (cpu.pinout.rw == RW_READ)
			cpu.pinout.d = mem.pinout.d;
		else if (cpu.pinout.rw == RW_WRITE)
			mem.pinout.d = cpu.pinout.d;
		else
			mem.pinout.a = 0x0000;
	}

	enum mapper_t {
		mapper0 = 0,
	};

	// Read INES rom format
	inline void load_nes_rom(str path) {
#pragma warning (disable : 4996)
		std::FILE* f = fopen(path.s, "rb");
		RELEASE_ASSERT(f, "Could not open file %s", path.s);
		u8 header[16];
		RELEASE_ASSERT(fread(header, 1, 16, f) == 16, "Could not read 16 bytes of header");
		u8 prg_rom_size = header[4]; // *16K
		u8 chr_rom_size = header[5]; // *8k
		mapper_t mapper = (mapper_t)((header[6]&flags_6::mapper_low)>>4 | header[7]&flags_7::mapper_high);
		ppu_mem.mirror_mode = header[6] & flags_6::nametable ? mirror_mode_t::vertical : mirror_mode_t::horizontal;

		switch (mapper) {
		case mapper0:
		{
			RELEASE_ASSERT(prg_rom_size > 0 && prg_rom_size <= 2, "Mapper0 rom must be 16K or 32K");
			for (int i = 2 - prg_rom_size; i < 2; ++i) {
				size_t bytes_read = fread(mem.cartridge_rom.memory + (0x8000 - Memory::CARTRIDGE_ROM_START) + i*0x4000, 1, 0x4000, f);
				RELEASE_ASSERT(bytes_read == 0x4000, "Could not read 16K from ROM");
			}
			RELEASE_ASSERT(chr_rom_size == 1, "Currently only supports 8K of CHR-ROM");
			size_t bytes_read = fread(ppu_mem.chr_rom.memory, 1, 0x2000, f);
			RELEASE_ASSERT(bytes_read == 0x2000, "Could not read 8K CHR-ROM from ROM");
			break;

		}
		default:
			RELEASE_ASSERT(false, "Unimplemented mapper %d", mapper);
			break;
		}
	}
};

