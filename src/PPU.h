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

	struct tile_t {
		u8 bp0;
		u8 bp1;
	};

	enum tile_type_t {
		background,
		foreground,
	};
	tile_t fetch_tile_row(u8 tile, tile_type_t tile_type, u8 y_offset, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		u16 pattern_table;
		//ASSERT(!cpu_mem.ppu_reg.ppuctrl & PPUReg::sprite_size, "8x16 sprites not yet supported");
		switch (tile_type) {
		case foreground:
			pattern_table = cpu_mem.ppu_reg.ppuctrl & PPUReg::fg_pattern_table ? 0x1000 : 0x0000;
			break;
		case background:
			pattern_table = cpu_mem.ppu_reg.ppuctrl & PPUReg::bg_pattern_table ? 0x1000 : 0x0000;
			break;
		default:
			ASSERT(false, "Invalid tile_type %d", tile_type);
		}

		u16 tile_offset = tile*2;

		tile_t ret;
		ret.bp0 = ppu_mem.read(pattern_table + tile_offset + y_offset + 0);
		ret.bp1 = ppu_mem.read(pattern_table + tile_offset + y_offset + 1);
		return ret;
	}

	u16 get_nametable_addr(u8 index_x, u8 index_y) {
		return PPUMemory::NAMETABLE_BASE_ADDR + index_y * 32 + index_x;
	}


	Colour render_dot(CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		u16 scroll_offset_x = cpu_mem.ppu_reg.ppuscrollX + cpu_mem.ppu_reg.ppuctrl & PPUReg::base_nametable_addr_x;
		u16 scroll_offset_y = cpu_mem.ppu_reg.ppuscrollY + cpu_mem.ppu_reg.ppuctrl & PPUReg::base_nametable_addr_y;

		scroll_offset_x += dot;
		scroll_offset_y += scanline;

		u8 nametable_index_x = scroll_offset_x / 32;
		u8 nametable_index_y = scroll_offset_x / 30;
		u8 tile_offset_x = scroll_offset_x % 32;
		u8 tile_offset_y = scroll_offset_y % 30;

		u8 bg_tile = ppu_mem.read(get_nametable_addr(nametable_index_x, nametable_index_y));

		tile_t bg = fetch_tile_row(bg_tile, background, tile_offset_y, cpu_mem, ppu_mem);

		Colour example_palette[4] = {Colour::TRANSPARENT, Colour::RED, Colour::GREEN, Colour::BLUE};

		u8 palette_index_l = !!(bg.bp0 & (1 << (7 - tile_offset_x)));
		u8 palette_index_h = !!(bg.bp1 & (1 << (7 - tile_offset_x)));

		u8 palette_index = palette_index_h << 1 | palette_index_l;
		ASSERT(palette_index < 0x4, "Invalid palette index $%x", palette_index);
		return example_palette[palette_index];
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
			return render_dot(cpu_mem, ppu_mem);
			// visible scanlines
			return Colour::BLUE;
		} else if (scanline == 240) {
			// post-render scanline
			return Colour::RED;
		} else if (scanline >= 241 && scanline <= 260) {
			if (scanline == 241 && dot == 1) {
				cpu_mem.ppu_reg.ppustatus |= 1<<7; // set vblank flag
				if (cpu_mem.ppu_reg.ppuctrl & 1<<7)
					// TODO: pinout should be set on PPU; connected externally?
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
