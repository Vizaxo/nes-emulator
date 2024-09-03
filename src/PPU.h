#pragma once

#include "Memory.h"
#include "6502.h"

struct PPU {
	u64 cycles_total = 0;

	u16 cycles_scanline = 0;
	u16 scanline = 0;
	u32 frame = 0;

	void tick(cpu6502& cpu, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		++cycles_total;
		++cycles_scanline;
		if (cycles_scanline >= 341) {
			++scanline;
			cycles_scanline = 0;
		}
		if (scanline >= 262) {
			++frame;
			scanline = 0;
		}

		if (scanline == 261) {
			// pre-render scanline
		} else if (scanline <= 239) {
			// visible scanlines
		} else if (scanline == 240) {
			// post-render scanline
		} else if (scanline >= 241 && scanline <= 260) {
			if (scanline == 241 && cycles_scanline == 1) {
				cpu_mem.ppu_reg.ppustatus |= 1<<7; // set vblank flag
				if (cpu_mem.ppu_reg.ppuctrl & 1<<7)
					cpu.pinout.nmiN = false; // send nmi if nmi-enable bit set
			}
			// vblank
		}
	}
};
