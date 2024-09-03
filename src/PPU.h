#pragma once

#include <rhi/RHI.h>
#include <types/Pointers.h>
#include <renderer/ImGuiUtils.h>

#include "Memory.h"
#include "6502.h"

struct PPU {
	u64 cycles_total = 0;

	u16 dot = 0;
	u16 scanline = 0;
	u32 frame = 0;

	static constexpr u32 SCANLINES_PER_FRAME = 262;
	static constexpr u32 DOTS_PER_SCANLINE = 341; // TODO: odd scanlines?
	static constexpr u32 DOTS_PER_FRAME = DOTS_PER_SCANLINE * SCANLINES_PER_FRAME;

	Colour framebuffer[DOTS_PER_FRAME];
	u32 framebuffer_idx = 0;

	void tick(cpu6502& cpu, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		framebuffer[framebuffer_idx] = run_cycle(cpu, cpu_mem, ppu_mem);
		framebuffer_idx = (framebuffer_idx+1) % DOTS_PER_FRAME;
	}

	Colour run_cycle(cpu6502& cpu, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
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
			return Colour::GREEN;
		} else if (scanline <= 239) {
			if (scanline == 10 && dot==0)
				cpu_mem.ppu_reg.ppustatus |= 1<<6; // fake a sprite 0 hit
			// visible scanlines
			return Colour::BLUE;
		} else if (scanline == 240) {
			// post-render scanline
			return Colour::RED;
		} else if (scanline >= 241 && scanline <= 260) {
			if (scanline == 241 && dot == 1) {
				cpu_mem.ppu_reg.ppustatus |= 1<<7; // set vblank flag
				if (cpu_mem.ppu_reg.ppuctrl & 1<<7)
					cpu.pinout.nmiN = false; // send nmi if nmi-enable bit set
			}
			return Colour::BLACK;
			// vblank
		}
	}

	void draw_framebuffer(RefPtr<RHI> rhi) {
		if (ImGui::Begin("PPU display")) {
			static OwningPtr<RHI::Texture2D, true> fb_tex = nullptr;
			fb_tex = rhi->createTexture(RHICommon::R8G8B8A8, (u8*)framebuffer, sizeof(Colour), v2i{ DOTS_PER_SCANLINE, SCANLINES_PER_FRAME }, true).getNullable();

			ImGui::Text("Frame %d, scanline %d, dot %d", frame, scanline, dot);
			ImGui::RHITexture(fb_tex.getRef().getNonNull());
		}
		ImGui::End();
	}
};
