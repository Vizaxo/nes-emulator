#pragma once

#include <types/Types.h>

#define RW_READ 1
#define RW_WRITE 0

template <int n>
struct RAM {
	u8 memory[n];

	u8& operator[](u16 addr) {
		return memory[addr];
	}
};

template <int n>
struct ROM {
	u8 memory[n];

	u8& operator[](u16 addr) {
		static u8 dummy;
		dummy = memory[addr];
		return dummy;
	}
};

namespace Memory {
	static constexpr u32 MEM_MAX = 0xffff;
	static constexpr u32 CARTRIDGE_ROM_START = 0x4020;
};

template <typename M>
struct Mem {

	struct Pinout {
		u16 a;
		u8 d;
		u8 rw : 1; // read/write. If high CPU is reading
	} pinout;

	void tick() {
		if (pinout.rw == RW_READ)
			pinout.d = (*(M*)this)[pinout.a];
		else
			(*(M*)this)[pinout.a] = pinout.d;
	}
};

struct CPUMemory : Mem<CPUMemory> {
	RAM<0x0800> internal_ram;
	RAM<0x08> ppu_reg;
	RAM<0x18> apu_io_reg;
	RAM<0x08> apu_io_disabled;
	ROM<0x10000-Memory::CARTRIDGE_ROM_START> cartridge_rom;

	u8& operator[](u16 addr) {
		if (addr < 0x2000)
			return internal_ram[addr % 0x800];
		if (addr < 0x4000)
			return ppu_reg[addr % 0x08];
		if (addr < 0x4018)
			return apu_io_reg[addr % 0x18];
		if (addr < 0x4020)
			return apu_io_disabled[addr % 0x08];
		else
			return cartridge_rom[addr - 0x4020];
	}

	void debug_set_all_mem(u8 d) {
		for (int i = 0; i <= Memory::MEM_MAX; ++i)
			(*this)[i] = d;
	}

	void debug_setmem(u16 a, u8 d) {
		(*this)[a] = d;
	}
};

struct PPUMemory : Mem<PPUMemory> {
	ROM<0x2000> chr_rom;
	RAM<0x0800> vram;
	RAM<0x20> palette_ram;

	u8& operator[](u16 addr) {
		if (addr < 0x2000)
			return chr_rom[addr];
		if (addr < 0x3000)
			return vram[addr % 0x800]; // TODO: mirroring modes
		if (addr < 0x3f00)
			return vram[addr % 0x800];
		if (addr < 0x4000)
			return palette_ram[addr % 0x20];
		ASSERT(false, "PPU address %04x out of range", addr);
	}
};
