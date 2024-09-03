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

	void write(u16 addr, u8 data) {
		memory[addr] = data;
	}

	u8 read(u16 addr) {
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

	void write(u16 addr, u8 data) {
		memory[addr] = data;
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

	template <typename T> void tick(T& ppu_mem) {
		if (pinout.rw == RW_READ)
			pinout.d = ((M*)this)->read(pinout.a, ppu_mem);
		else
			((M*)this)->write(pinout.a, pinout.d, ppu_mem);
	}
};

struct PPUMemory : Mem<PPUMemory> {
	ROM<0x2000> chr_rom;
	RAM<0x0800> vram;
	RAM<0x20> palette_ram;

	RAM<0x40> oam;

	u8& operator[](u16 addr) {
		if (addr < 0x2000)
			return chr_rom[addr];
		if (addr < 0x3000)
			return vram[addr % 0x800]; // TODO: mirroring modes
		if (addr < 0x3f00)
			return vram[addr % 0x800];
		if (addr < 0x4000)
			return palette_ram[addr % 0x20];
		static u8 dummy = 0;
		return dummy;
		//ASSERT(false, "PPU address %04x out of range", addr);
	}

	u8 read(u16 addr, PPUMemory& ppu_mem) { return read(addr); }
	u8 read(u16 addr) {
		return (*this)[addr];
	}

	void write(u16 addr, u8 data, PPUMemory& ppu_mem) { write(addr, data); }
	void write(u16 addr, u8 data) {
		(*this)[addr] = data;
	}
};

struct PPUReg : Mem<PPUReg> {
	u8 ppuctrl;
	u8 ppumask;
	u8 ppustatus;
	u8 oamaddr;
	u8 ppuscrollX;
	u8 ppuscrollY;
	u16 ppuaddr;

	u8 v = 0;
	u8 t = 0;
	u8 x = 0;
	u8 w = 0;

	u8 read(u16 addr) {

	}

	void inc_ppuaddr() {
		ppuaddr += (ppuctrl & (1<<2)) ? 1 : 32;
	}

	void write(u16 addr, u8 data, PPUMemory& ppu_mem) {
		switch (addr) {
		case 0x0:
			ppuctrl = data;
			break;
		case 0x1:
			ppumask = data;
			break;
		case 0x2:
			//ppustatus = data & (0xe0);
			break;
		case 0x3:
			oamaddr = data;
			break;
		case 0x4:
			ppu_mem.oam.write(oamaddr, data);
			++oamaddr;
			break;
		case 0x5:
			if (!w) {
				ppuscrollX = data;
				++w;
			} else {
				ppuscrollY = data;
				--w;
			}
			break;
		case 0x6:
			if (!w) {
				ppuaddr = ((u16)data)<<8 | ppuaddr&0xff; // write upper byte
				++w;
			} else {
				ppuaddr = ppuaddr&0xff00 | data;
				--w;
			}
			break;
		case 0x7:
			ppu_mem.write(ppuaddr, data);
			inc_ppuaddr();
			break;
		default:
			ASSERT(false, "Writing address out of PPU reg address $%04x", addr);
			break;
		}
	}

	// Read without modifying any state
	u8 debug_read(u16 addr, PPUMemory& ppu_mem) { return read(addr, ppu_mem, true); }
	u8 read(u16 addr, PPUMemory& ppu_mem, bool debug = false) {
		u8 junk_read = 0xea; // TODO: make this behave like the actual PPU bus?
		switch (addr) {
		case 0x0:
		case 0x1:
			return junk_read;
		case 0x2:
			if (!debug) w = 0;
			return ppustatus;
		case 0x3:
			return junk_read;
		case 0x4:
			ppu_mem.oam.read(oamaddr);
			break;
		case 0x5:
			return junk_read;
		case 0x6:
			return junk_read;
		case 0x7:
			ppu_mem.read(ppuaddr);
			if (!debug) inc_ppuaddr();
			break;
		default:
			ASSERT(false, "Writing address out of PPU reg address $%04x", addr);
			break;
		}
	}
};

struct CPUMemory : Mem<CPUMemory> {
	RAM<0x0800> internal_ram;
	PPUReg ppu_reg;
	RAM<0x18> apu_io_reg;
	RAM<0x08> apu_io_disabled;
	ROM<0x10000-Memory::CARTRIDGE_ROM_START> cartridge_rom;

	void write(u16 addr, u8 data, PPUMemory& ppu_mem) {
		if (addr < 0x2000)
			return internal_ram.write(addr % 0x800, data);
		if (addr < 0x4000)
			return ppu_reg.write(addr % 0x08, data, ppu_mem);
		if (addr < 0x4018)
			return apu_io_reg.write(addr % 0x18, data);
		if (addr < 0x4020)
			return apu_io_disabled.write(addr % 0x08, data);
		// can't write to cart rom (in mapper 0)
	}

	u8 debug_read(u16 addr, PPUMemory& ppu_mem) { return read(addr, ppu_mem, true); }
	u8 read(u16 addr, PPUMemory& ppu_mem, bool debug = false) {
		if (addr < 0x2000)
			return internal_ram[addr % 0x800];
		if (addr < 0x4000)
			return ppu_reg.read(addr % 0x08, ppu_mem, debug);
		if (addr < 0x4018)
			return apu_io_reg[addr % 0x18];
		if (addr < 0x4020)
			return apu_io_disabled[addr % 0x08];
		else
			return cartridge_rom[addr - 0x4020];
	}

	void debug_set_all_mem(u8 d, PPUMemory& ppu_mem) {
		for (int i = 0; i <= Memory::MEM_MAX; ++i)
			this->write(i, d, ppu_mem);
	}

	void debug_setmem(u16 a, u8 d, PPUMemory& ppu_mem) {
		this->write(a, d, ppu_mem);
	}
};
