#pragma once

#include "Memory.h"
#include "6502.h"

struct PPU {
	u64 cycles_total = 0;

	u16 dot = 0;
	u16 scanline = 0;
	u32 frame = 0;

	void tick(cpu6502& cpu, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		++cycles_total;
		++dot;
		if (dot >= 341) {
			++scanline;
			dot = 0;
		}
		if (scanline >= 262) {
			++frame;
			scanline = 0;
		}

		if (scanline == 261) {
			if (dot == 1) {
				cpu_mem.ppu_reg.ppustatus &= ~(1<<7 | 1<<6);
			}
			// pre-render scanline
		} else if (scanline <= 239) {
			if (scanline == 10 && dot==0)
				cpu_mem.ppu_reg.ppustatus |= 1<<6; // fake a sprite 0 hit
			// visible scanlines
		} else if (scanline == 240) {
			// post-render scanline
		} else if (scanline >= 241 && scanline <= 260) {
			if (scanline == 241 && dot == 1) {
				cpu_mem.ppu_reg.ppustatus |= 1<<7; // set vblank flag
				if (cpu_mem.ppu_reg.ppuctrl & 1<<7)
					cpu.pinout.nmiN = false; // send nmi if nmi-enable bit set
			}
			// vblank
		}
	}
};
