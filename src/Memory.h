#pragma once

#include <types/Types.h>

#define RW_READ 1
#define RW_WRITE 0

template <int n>
struct RAM {
	u8 memory[n];

	u8& operator[](u16 addr) {
		ASSERT(addr < n, "RAM address out of range. Accessed $%04x. Max addr $%04x", addr, n-1);
		return memory[addr];
	}

	void write(u16 addr, u8 data) {
		ASSERT(addr < n, "RAM address out of range. Accessed $%04x. Max addr $%04x", addr, n-1);
		memory[addr] = data;
	}

	u8 read(u16 addr) {
		ASSERT(addr < n, "RAM address out of range. Accessed $%04x. Max addr $%04x", addr, n-1);
		return memory[addr];
	}
};

template <int n>
struct ROM {
	u8 memory[n];

	u8 read(u16 addr) {
		ASSERT(addr < n, "ROM address out of range. Accessed $%04x. Max addr $%04x", addr, n-1);
		return memory[addr];
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

enum class mirror_mode_t {
	horizontal,
	vertical,
	single_screen,
	quad_screen,
};

struct PPUMemory : Mem<PPUMemory> {
	static constexpr u16 NAMETABLE_BASE_ADDR = 0x2000;

	ROM<0x2000> chr_rom;
	RAM<0x0800> vram;
	RAM<0x20> palette_ram;

	// Not in main addr space
	RAM<0x100> oam;
	RAM<0x40> secondary_oam;

	mirror_mode_t mirror_mode;

	u16 nametable_addr(u16 addr) {
		switch (mirror_mode) {
		case mirror_mode_t::horizontal:
			return (addr & ~0xc00) | ((addr & 0x800) >> 1);
		case mirror_mode_t::vertical:
			return addr & ~0x800;
		case mirror_mode_t::single_screen:
			//fallthrough
		case mirror_mode_t::quad_screen:
			ASSERT(false, "Unimplemented mirror mode %d", mirror_mode);
			return addr % 0x400;
		default:
			ASSERT(false, "Invalid mirror mode %d", mirror_mode);
			return addr % 0x400;
		}
	}

	u8 read(u16 addr, PPUMemory& ppu_mem) { return read(addr); }
	u8 read(u16 addr) {
		if (addr < 0x2000)
			return chr_rom.read(addr);
		if (addr < 0x3000)
			return vram[nametable_addr(addr) - NAMETABLE_BASE_ADDR];
		if (addr < 0x3f00)
			return vram[addr % 0x800];
		if (addr < 0x4000)
			// Palette entry 0s are tied together
			return palette_ram[((addr & 0b11) == 0b00 ? addr & ~(1<<4) : addr) % 0x20];
		return 0x00;
		//ASSERT(false, "PPU address %04x out of range", addr);
	}

	void write(u16 addr, u8 data, PPUMemory& ppu_mem) { write(addr, data); }
	void write(u16 addr, u8 data) {
		if (addr < 0x2000)
			; // writing to rom
		else if (addr < 0x3000)
			vram[nametable_addr(addr) - NAMETABLE_BASE_ADDR] = data;
		else if (addr < 0x3f00)
			vram[addr % 0x800] = data;
		else if (addr < 0x4000)
			// Palette entry 0s are tied together
			palette_ram[((addr & 0b11) == 0b00 ? addr & ~(1<<4) : addr) % 0x20] = data;
		//ASSERT(false, "PPU address %04x out of range", addr);
	}

};

struct PPUReg : Mem<PPUReg> {
	enum ppuctrl_bit : u8 {
		base_nametable_addr_x = 0x1,
		base_nametable_addr_y = 0x2,
		vram_incr = 0x4,
		fg_pattern_table = 0x8,
		bg_pattern_table =  0x10,
		sprite_size = 0x20,
		colour_on_ext = 0x40,
		vblank_nmi = 0x80,
	};
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

	void inc_ppuaddr() {
		ppuaddr += (ppuctrl & vram_incr) ? 32 : 1;
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
			return ppu_mem.oam.read(oamaddr);
		case 0x5:
			return junk_read;
		case 0x6:
			return junk_read;
		case 0x7:
		{
			static u8 buffered;
			u8 out = buffered;
			if (!debug)
				buffered = ppu_mem.read(ppuaddr);
			return out;
		}
		default:
			ASSERT(false, "Writing address out of PPU reg address $%04x", addr);
			return junk_read;
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
			return cartridge_rom.read(addr - 0x4020);
	}

	void debug_set_all_mem(u8 d, PPUMemory& ppu_mem) {
		for (int i = 0; i <= Memory::MEM_MAX; ++i)
			this->write(i, d, ppu_mem);
	}

	void debug_setmem(u16 a, u8 d, PPUMemory& ppu_mem) {
		this->write(a, d, ppu_mem);
	}
};
