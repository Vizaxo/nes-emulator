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
	static constexpr v2i PATTERN_TABLE_SIZE_TILES = {16, 16};
	static constexpr i32 PATTERN_TABLE_NUM_TILES = PATTERN_TABLE_SIZE_TILES.x*PATTERN_TABLE_SIZE_TILES.y;
	static constexpr v2i TILE_SIZE = {8,8};
	static constexpr v2i PATTERN_TABLE_SIZE_PIXELS = {16*8, 16*8};

	Colour framebuffer[DOTS_PER_FRAME];
	u32 framebuffer_idx = 0;

	void tick(cpu6502& cpu, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		framebuffer[framebuffer_idx] = run_cycle(cpu, cpu_mem, ppu_mem);
		framebuffer_idx = (framebuffer_idx+1) % DOTS_PER_FRAME;
	}

	struct tile_row_t {
		u8 bp0;
		u8 bp1;
	};

	enum tile_type_t {
		background,
		foreground,
	};

	u16 get_pattern_table_addr(u8 pattern_table_idx) {
		switch (pattern_table_idx) {
		case 0: return 0x0000;
		case 1: return 0x1000;
		default: ASSERT(false, "Invalid pattern table idx %x", pattern_table_idx);
		}
	}

	u16 get_pattern_table_addr(tile_type_t tile_type, CPUMemory& cpu_mem) {
		switch (tile_type) {
		case foreground:
			return get_pattern_table_addr((cpu_mem.ppu_reg.ppuctrl & PPUReg::fg_pattern_table) >> 3);
		case background:
			return get_pattern_table_addr((cpu_mem.ppu_reg.ppuctrl & PPUReg::bg_pattern_table) >> 4);
		default:
			ASSERT(false, "Invalid tile_type %d", tile_type);
		}
	}

	tile_row_t fetch_tile_row(u8 tile, u16 pattern_table_addr, u8 y_offset, PPUMemory& ppu_mem) {
		ASSERT(pattern_table_addr < 0x2000, "Invalid pattern table addr");
		ASSERT(y_offset < 8, "Y offset should be 0 <= y_offs < 8");
		//ASSERT(!cpu_mem.ppu_reg.ppuctrl & PPUReg::sprite_size, "8x16 sprites not yet supported");

		u16 tile_offset = tile*2*8;

		tile_row_t ret;
		ret.bp0 = ppu_mem.read(pattern_table_addr + tile_offset + y_offset + 0);
		ret.bp1 = ppu_mem.read(pattern_table_addr + tile_offset + y_offset + 1);
		return ret;
	}

	u16 get_nametable_addr(u8 index_x, u8 index_y) {
		return PPUMemory::NAMETABLE_BASE_ADDR + index_y * 32 + index_x;
	}


	int debug_scroll_x = 0;
	int debug_scroll_y = 0;
	Colour render_dot(u16 dot, u16 scanline, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		//u16 scroll_offset_x = cpu_mem.ppu_reg.ppuscrollX + cpu_mem.ppu_reg.ppuctrl & PPUReg::base_nametable_addr_x;
		//u16 scroll_offset_y = cpu_mem.ppu_reg.ppuscrollY + cpu_mem.ppu_reg.ppuctrl & PPUReg::base_nametable_addr_y;
		u16 scroll_offset_x = debug_scroll_x;
		u16 scroll_offset_y = debug_scroll_y;

		scroll_offset_x += dot;
		scroll_offset_y += scanline;

		/*
		u8 nametable_index_x = scroll_offset_x / 32;
		u8 nametable_index_y = scroll_offset_x / 30;
		u8 tile_offset_x = scroll_offset_x % 32;
		u8 tile_offset_y = scroll_offset_y % 30;
		*/
		u8 nametable_index_x = scroll_offset_x / TILE_SIZE.x;
		u8 nametable_index_y = scroll_offset_y / TILE_SIZE.y;
		u8 tile_offset_x = scroll_offset_x % TILE_SIZE.x;
		u8 tile_offset_y = scroll_offset_y % TILE_SIZE.y;

		u8 bg_tile = ppu_mem.read(get_nametable_addr(nametable_index_x, nametable_index_y));

		//tile_row_t bg = fetch_tile_row(bg_tile, get_pattern_table_addr(background, cpu_mem), tile_offset_y, ppu_mem);
		tile_row_t bg = fetch_tile_row(bg_tile, get_pattern_table_addr(1), tile_offset_y, ppu_mem);

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
			return render_dot(dot, scanline, cpu_mem, ppu_mem);
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

	OwningPtr<RHI::Texture2D> get_pattern_table_texture(RefPtr<RHI> rhi, u8 pattern_table_idx, PPUMemory& ppu_mem) {
		Colour pattern_table_buffer[PATTERN_TABLE_SIZE_PIXELS.x * PATTERN_TABLE_SIZE_PIXELS.y];
		// Draw pattern table 0

		struct tile_t {
			tile_row_t rows[TILE_SIZE.y];
		};
		tile_t tiles[PATTERN_TABLE_NUM_TILES];
		for (int tile = 0; tile < PATTERN_TABLE_NUM_TILES; ++tile)
			for (int y = 0; y < TILE_SIZE.y; ++y)
				// TODO: allow drawing specific pattern table, not bg/fg
				tiles[tile].rows[y] = fetch_tile_row(tile, get_pattern_table_addr(pattern_table_idx), y, ppu_mem);

		Colour greyscale_palette[4] = { Colour::BLACK, Colour::DARK_GREY, Colour::MID_GREY, Colour::WHITE };

		for (int x = 0; x < PATTERN_TABLE_SIZE_PIXELS.x; ++x) {
			for (int y = 0; y < PATTERN_TABLE_SIZE_PIXELS.y; ++y) {
				u16 x_tile = x / TILE_SIZE.x;
				u16 y_tile = y / TILE_SIZE.y;
				tile_t& tile = tiles[y_tile * TILE_SIZE.x + x_tile];
				tile_row_t& tile_row = tile.rows[y % TILE_SIZE.y];
				u8 tile_offset_x = x % TILE_SIZE.x;
				u8 palette_index_l = !!(tile_row.bp0 & (1 << (7 - tile_offset_x)));
				u8 palette_index_h = !!(tile_row.bp1 & (1 << (7 - tile_offset_x)));

				u8 palette_index = palette_index_h << 1 | palette_index_l;
				ASSERT(palette_index < 0x4, "Invalid palette index $%x", palette_index);
				pattern_table_buffer[y * PATTERN_TABLE_SIZE_PIXELS.x + x] = greyscale_palette[palette_index];
			}
		}

		return rhi->createTexture(RHICommon::R8G8B8A8, (u8*)pattern_table_buffer, sizeof(Colour), PATTERN_TABLE_SIZE_PIXELS, true);
	}

	void draw_framebuffer(RefPtr<RHI> rhi, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		if (ImGui::Begin("PPU display")) {
			ImGui::Text("Frame %d, scanline %d, dot %d", frame, scanline, dot);

			ImGui::SliderInt("Scroll x", &debug_scroll_x, 0, 511, "%03x");
			ImGui::SliderInt("Scroll y", &debug_scroll_y, 0, 479, "%03x");

			static OwningPtr<RHI::Texture2D, true> fb_tex = nullptr;
			fb_tex = rhi->createTexture(RHICommon::R8G8B8A8, (u8*)framebuffer, sizeof(Colour), v2i{ DOTS_PER_SCANLINE, SCANLINES_PER_FRAME }, true).getNullable();
			ImGui::RHITexture(fb_tex.getRef().getNonNull());


			{
				static OwningPtr<RHI::Texture2D, true> pattern_table_0_tex = nullptr;
				static OwningPtr<RHI::Texture2D, true> pattern_table_1_tex = nullptr;
				pattern_table_0_tex = get_pattern_table_texture(rhi, 0, ppu_mem).getNullable();
				pattern_table_1_tex = get_pattern_table_texture(rhi, 1, ppu_mem).getNullable();
				ImGui::RHITexture(pattern_table_0_tex.getRef().getNonNull(), 2.0f);
				ImGui::SameLine();
				ImGui::RHITexture(pattern_table_1_tex.getRef().getNonNull(), 2.0f);
			}
		}
		ImGui::End();
	}
};
