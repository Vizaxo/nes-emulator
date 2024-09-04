#pragma once

#include <rhi/RHI.h>
#include <types/Pointers.h>
#include <renderer/ImGuiUtils.h>

#include "Memory.h"
#include "6502.h"
#include "Palette.h"

struct PPU {
	u64 cycles_total = 0;

	u16 dot = 0;
	i16 scanline = -1;
	u32 frame = 0;

	static constexpr u32 SCANLINES_PER_FRAME = 262;
	static constexpr u32 DOTS_PER_SCANLINE = 341; // TODO: odd scanlines?
	static constexpr u32 DOTS_PER_FRAME = DOTS_PER_SCANLINE * SCANLINES_PER_FRAME;
	static constexpr v2i PATTERN_TABLE_SIZE_TILES = {16, 16};
	static constexpr i32 PATTERN_TABLE_NUM_TILES = PATTERN_TABLE_SIZE_TILES.x*PATTERN_TABLE_SIZE_TILES.y;
	static constexpr v2i TILE_SIZE = {8,8};
	static constexpr v2i PATTERN_TABLE_SIZE_PIXELS = {16*8, 16*8};

	Colour framebuffer[DOTS_PER_FRAME];
	u32 framebuffer_idx = 1;

	void tick(palette_t& palette, cpu6502& cpu, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		framebuffer[framebuffer_idx] = palette.colours[run_cycle(cpu, cpu_mem, ppu_mem)];
		framebuffer_idx = (framebuffer_idx+1) % DOTS_PER_FRAME;
	}

	struct tile_row_t {
		u8 bp0;
		u8 bp1;
	};

	enum tile_type_t {
		background,
		sprite,
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
		case sprite:
			return get_pattern_table_addr((cpu_mem.ppu_reg.ppuctrl & PPUReg::sprite_pattern_table) >> 3);
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
		ret.bp1 = ppu_mem.read(pattern_table_addr + tile_offset + y_offset + 8);
		return ret;
	}

	u16 get_nametable_addr(u8 index_x, u8 index_y) {
		u16 base_nametable_addr = 0x2000;
		if (index_x >= 64) {
			index_x -= 64;
		} else if (index_x >= 32) {
			base_nametable_addr += 0x400;
			index_x -= 32;
		}
		if (index_y >= 60) {
			index_y -= 60;
		} else if (index_y >= 60) {
			base_nametable_addr += 0x800;
			index_y -= 60;
		}
		u16 ret = base_nametable_addr + index_y * 32 + index_x;
		ASSERT(ret >= 0x2000 && ret < 0x3000, "Nametable addr out of range");
		return ret;
	}

	u16 get_attribute_table_addr(u8 index_x, u8 index_y) {
		u16 base_nametable_addr = 0x2000;
		if (index_x >= 16) {
			index_x -= 16;
		} else if (index_x >= 8) {
			base_nametable_addr += 0x400;
			index_x -= 8;
		}

		// We do this at a finer grain because of the half-tile at the bottom of the nametable
		index_y*=2;
		if (index_y >= 30) {
			index_y -= 30;
		} else if (index_y >= 15) {
			base_nametable_addr += 0x800;
			// TODO: y -= 0.5?
			index_y -= 15;
		}
		u16 ret = base_nametable_addr + 0x3c0 + index_y * 4 + index_x;
		ASSERT(ret >= 0x2000 && ret < 0x3000, "Attribute table addr out of range");
		return ret;
	}

	u8 get_palette_index(u8 tile, tile_type_t tile_type, i16 tile_offset_x, i16 tile_offset_y, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		if (tile_offset_x < 0 || tile_offset_x >=8)
			return 0; // tile off screen. transparent

		tile_row_t bg = fetch_tile_row(tile, get_pattern_table_addr(tile_type, cpu_mem), tile_offset_y, ppu_mem);

		u8 palette_index_l = !!(bg.bp0 & (1 << (7 - tile_offset_x)));
		u8 palette_index_h = !!(bg.bp1 & (1 << (7 - tile_offset_x)));

		u8 palette_index = palette_index_h << 1 | palette_index_l;
		ASSERT(palette_index < 0x4, "Invalid palette index $%x", palette_index);

		return palette_index;
	}

	u8 get_colour(u8 palette, u8 palette_index, tile_type_t tile_type, PPUMemory& ppu_mem) {
		if (palette_index == 0)
			// transparent
			return ppu_mem.read(0x3f00);
		else
			return ppu_mem.read(0x3f00 + palette*4 + palette_index + (tile_type == sprite ? 4 * 4 : 0));
	}

	int debug_scroll_x = 0;
	int debug_scroll_y = 0;
	bool use_debug_scroll = false;
	u8 fine_x_shr;
	u8 get_background_dot(u16 dot, u16 scanline, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		if (!(cpu_mem.ppu_reg.ppumask & PPUReg::show_bg)
			|| (!(cpu_mem.ppu_reg.ppumask & PPUReg::show_bg_left_col)) && dot <= 8)
			return 0x00;

		u16 scroll_offset_x;
		u16 scroll_offset_y;
		PPUReg& p = cpu_mem.ppu_reg;
		++fine_x_shr;
		if (use_debug_scroll) {
			scroll_offset_x = debug_scroll_x;
			scroll_offset_y = debug_scroll_y;

			scroll_offset_x += dot;
			scroll_offset_y += scanline;
		} else {
//			u8 base_nametable_addr_x = !!(cpu_mem.ppu_reg.ppuctrl & PPUReg::base_nametable_addr_x);
//			u8 base_nametable_addr_y = !!(cpu_mem.ppu_reg.ppuctrl & PPUReg::base_nametable_addr_y);
//			scroll_offset_x = cpu_mem.ppu_reg.ppuscrollX + base_nametable_addr_x*256;
//			scroll_offset_y = cpu_mem.ppu_reg.ppuscrollY + base_nametable_addr_y*240;
			// yyy NN YYYYY XXXXX
			// ||| || ||||| +++++-- coarse X scroll
			// ||| || +++++-------- coarse Y scroll
			// ||| ++-------------- nametable select
			// +++----------------- fine Y scroll

			// yyy NNYY YYYX XXXX
			u16 nametable_select_x = (p.v>>10) & 0x1;
			u16 nametable_select_y = (p.v>>11) & 0x1;
			u16 coarse_x = p.v&0x1f;
			u16 coarse_y = (p.v>>5)&0x1f;
			u16 fine_y = (p.v>>12)&0x7;

			// TODO: this is bad because it doesn't get reset on reset
			// Only temporary until we wire in proper v addressing anyway
			//u16 fine_x = (p.x + fine_x_shr) % 8;
			// TODO: plumb fine x back in

			scroll_offset_x = ((fine_x_shr) & 0x7) | (coarse_x<<3) | (nametable_select_x<<8);
			scroll_offset_y = fine_y | (coarse_y<<3) | (nametable_select_y<<8);
			if (scroll_offset_x < 24)
				if (scroll_offset_y >= 1)
					scroll_offset_y -= 1;
			scroll_offset_x -= 24; // simulate fetching 3 tiles previous
			scroll_offset_x = scroll_offset_x%512;
			scroll_offset_x += p.x;
		}

		u8 nametable_index_x = scroll_offset_x / TILE_SIZE.x;
		u8 nametable_index_y = scroll_offset_y / TILE_SIZE.y;
		if (!use_debug_scroll) {
			//ASSERT(nametable_index_x == ((p.v & 0x1f) | (((p.v >> 10) & 0x1) << 5)), "Nametable X calculation wrong");
			//ASSERT(nametable_index_y == (((p.v >> 5) & 0x1f) | (((p.v >> 11) & 0x1) << 5)), "Nametable X calculation wrong");
		}
		//ASSERT(nametable_index_y == (p.v>>11) & 0x1, "Nametable X calculation wrong");
		u8 tile_offset_x = scroll_offset_x % TILE_SIZE.x;
		u8 tile_offset_y = scroll_offset_y % TILE_SIZE.y;

		//u8 bg_tile = ppu_mem.read(p.v);
		u8 bg_tile = ppu_mem.read(get_nametable_addr(nametable_index_x, nametable_index_y));

		tile_row_t bg = fetch_tile_row(bg_tile, get_pattern_table_addr(background, cpu_mem), tile_offset_y, ppu_mem);

		u8 attribute_table_idx_x = scroll_offset_x / 32;
		u8 attribute_table_idx_y = scroll_offset_y / 32;
		u8 attribute_table_offs_x = scroll_offset_x % 32;
		u8 attribute_table_offs_y = scroll_offset_y % 32;
		u8 attr_table_idx = attribute_table_idx_y * 8 + attribute_table_idx_x;

		//u8 attribute = ppu_mem.read(PPUMemory::NAMETABLE_BASE_ADDR + 960 + attr_table_idx);
		u8 attribute = ppu_mem.read(get_attribute_table_addr(attribute_table_idx_x, attribute_table_idx_y));
		u8 attr_table_lr = attribute_table_offs_x >= 16; // 0: left. 1: right
		u8 attr_table_bt = attribute_table_offs_y >= 16; // 0: top. 1: bottom
		u8 attr_table_bit_offset = attr_table_lr | attr_table_bt << 1;

		u8 palette = (attribute >> (attr_table_bit_offset*2)) & 0x03;

		u8 palette_index_l = !!(bg.bp0 & (1 << (7 - tile_offset_x)));
		u8 palette_index_h = !!(bg.bp1 & (1 << (7 - tile_offset_x)));

		u8 palette_index = palette_index_h << 1 | palette_index_l;
		ASSERT(palette_index < 0x4, "Invalid palette index $%x", palette_index);

		if (palette_index == 0)
			// transparent
			return ppu_mem.read(0x3f00);
		else
			return ppu_mem.read(0x3f00 + palette*4 + palette_index /* + sprite offset*/);
	}

	static constexpr u8 SECONDARY_OAM_SPRITE_COUNT = 8;
	u8 secondary_sprites = 0;
	bool sprite_zero_in_secondary = false;
	bool sprite_zero_hit = true;
	u8 render_dot(u16 dot, u16 scanline, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		struct sprite_t {
			u8 y_coord;
			u8 tile_index;
			u8 attributes;
			u8 x_coord;
		};
		sprite_t* chosen_sprite = nullptr;
		u8 palette_index;
		ASSERT(secondary_sprites <= SECONDARY_OAM_SPRITE_COUNT, "Exceeding bounds of secondary oam");
		for (int i = 0; i < secondary_sprites; ++i) {
			if (!(cpu_mem.ppu_reg.ppumask & PPUReg::show_sprites_left_col) && dot <= 8)
				break;

			sprite_t& s = *((sprite_t*)ppu_mem.secondary_oam.memory + i);
			i16 x_pix = (i16)dot - (i16)s.x_coord - 1;
			ASSERT(s.y_coord < scanline && scanline <= s.y_coord+8, "");
			if (s.attributes & (1 << 6))
				x_pix = 7 - x_pix;
			i16 y_pix = (i16)scanline - (i16)s.y_coord - 1;
			ASSERT(y_pix >= 0 && y_pix < 8, "Sprite should not be being rendered");
			if (s.attributes & (1 << 7))
				y_pix = 7 - y_pix;
			palette_index = get_palette_index(s.tile_index, sprite, x_pix, y_pix, cpu_mem, ppu_mem);
			if (palette_index != 0) {
				chosen_sprite = &s;
				break;
			}
		}

		u8 bg_color = get_background_dot(dot, scanline, cpu_mem, ppu_mem);
		if (!chosen_sprite) {
			return bg_color;
		}

		u8 sprite_color = get_colour(chosen_sprite->attributes & 0x3, palette_index, sprite, ppu_mem);
		if (sprite_zero_in_secondary && chosen_sprite == (sprite_t*)ppu_mem.secondary_oam.memory && bg_color != 0x0 && sprite_color != 0x0)
			sprite_zero_hit = true;

		if (sprite_color == 0)
			return bg_color;
		if (!(chosen_sprite->attributes & (1<<5))) // priority
			return sprite_color;

		if (bg_color != 0)
			return bg_color;
		else
			return sprite_color;
	}

	void prepare_secondary_oam(i16 scanline, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		if (dot == 260) {
			secondary_sprites = 0;
			if (!(cpu_mem.ppu_reg.ppumask & PPUReg::show_sprites))
				return;
			for (int i = 0; i < 64; ++i) {
				i32 y_coord = ppu_mem.oam.read(i*4 + 0);
				i16 y_pix = scanline - y_coord; // Do this on the scanline before, so don't subtract 1
				if (y_coord <= scanline && scanline < y_coord + 8) {
					// hit
					memcpy(ppu_mem.secondary_oam.memory + 4*secondary_sprites, ppu_mem.oam.memory + i*4, 4);

					struct sprite_t {
						u8 y_coord;
						u8 tile_index;
						u8 attributes;
						u8 x_coord;
					};
					sprite_t& s = *((sprite_t*)ppu_mem.secondary_oam.memory + secondary_sprites);
					i16 x_pix = (i16)dot - (i16)s.x_coord - 1;
					if (s.attributes & (1 << 6))
						x_pix = 7 - x_pix;
					i16 y_pix_2 = (i16)scanline - (i16)s.y_coord - 1;
					//ASSERT(y_pix == y_pix_2 + 1, "Sprite should not be being rendered");
					//ASSERT(y_pix_2 >= 0 && y_pix_2 < 8, "Sprite should not be being rendered");
					++secondary_sprites;
					if (i == 0)
						sprite_zero_in_secondary = true;
				}
				if (secondary_sprites >= SECONDARY_OAM_SPRITE_COUNT)
					break;
			}
		}
	}

	u8 run_cycle(cpu6502& cpu, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		++cycles_total;
		++dot;
		if (dot >= 341) {
			++scanline;
			dot = 0;
		}
		if (scanline >= 261) {
			++frame;
			scanline = -1;
		}
		PPUReg& p = cpu_mem.ppu_reg;

		bool rendering_enabled = (p.ppumask & PPUReg::show_bg) || (p.ppumask & PPUReg::show_sprites);

		if (scanline == -1) {
			if (!rendering_enabled)
				return 0x00;

			prepare_secondary_oam(scanline, cpu_mem, ppu_mem);
			if (dot == 1) {
				cpu_mem.ppu_reg.ppustatus &= ~(1<<7 | 1<<6);
				sprite_zero_hit = 0;
			}
			if (280 <= dot && dot <= 304) {
				p.v = (p.v & ~0x7be0) | (p.t & 0x7be0);
			}
			// pre-render scanline
			return 0x2b;
		} else if (scanline <= 239) {
			if (!rendering_enabled)
				return 0x00;

			if (dot == 257) {
				// v: ....A.. ...BCDEF <- t: ....A.. ...BCDEF
				p.v = (p.v & ~(0x041f)) | (p.t & 0x041f); // A.. ...BCDEF
			}

			if (dot <= 256 || dot >= 328) {
				// horizontal increment of v
				// fine x increment?
				// p.x = (++p.x) % 8;
				if (dot % 8 == 0) {
					// corse x increment
					if ((p.v & 0x1f) == 31) {
						p.v = p.v & ~0x1f; // coarse x = 0
						p.v = p.v ^ (0x0400); // flip x nametable
					}
					else {
						++p.v; // incr coarse x
					}
				}
			}

			if (dot == 256) {
				// fine y increment
				if ((p.v & 0x7000) != 0x7000) {
					p.v += 0x1000; // fine y < 7
				} else {
					p.v = p.v & ~0x7000;
					int coarse_y = (p.v & 0x3e0) >> 5;
					if (coarse_y == 29) {
						coarse_y = 0;
						p.v = p.v ^ 0x800; // flip y nametable
					} else if (coarse_y == 31) {
						coarse_y = 0; // overflow
					} else {
						++coarse_y;
					}
					p.v = (p.v & ~0x3e0) | (coarse_y<<5);
				}
			}

			prepare_secondary_oam(scanline, cpu_mem, ppu_mem);
			// Visible scanlines
			if (dot < 256) {
				u8 ret = render_dot(dot, scanline, cpu_mem, ppu_mem);
				if (sprite_zero_hit)
					cpu_mem.ppu_reg.ppustatus |= 1 << 6; // fake a sprite 0 hit
				return ret;
			} else {
				return 0x0f;
			}
		} else if (scanline == 240) {
			// post-render scanline
			return 0x16;
		} else if (scanline >= 241 && scanline <= 260) {
			if (scanline == 241 && dot == 1) {
				cpu_mem.ppu_reg.ppustatus |= 1<<7; // set vblank flag
				if (cpu_mem.ppu_reg.ppuctrl & 1<<7)
					// TODO: pinout should be set on PPU; connected externally?
					cpu.pinout.nmiN = false; // send nmi if nmi-enable bit set
			}
			return 0x0f;
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

	void draw_ppu_vram(PPUMemory& ppu_mem) {
		ImGui::Begin("VRAM");
		static ImGuiTableFlags table_flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg
			| ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_Resizable;

		static int num_columns = 8;
		ImGui::InputInt("Num columns", &num_columns);

		ImGui::BeginTable("##mem", num_columns+1, table_flags);

		for (int i = 0; i < num_columns; i++) {
			ImGui::PushID(i);
			ImGui::TableSetupColumn("");
			ImGui::PopID();
		}
		ImGui::TableHeadersRow();

		ImGuiListClipper clipper;
		int num_rows = (Memory::MEM_MAX+1) / num_columns + ((Memory::MEM_MAX+1) % num_columns != 0 ? 1 : 0);
		clipper.Begin(num_rows);

		while (clipper.Step()) {
			for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
				u16 addr = i * num_columns;
				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0);
				ImGui::Text("%04x", addr);
				for (int j = 0; j < num_columns; j++) {
					ImGui::PushID(addr+j);
					ImGui::TableSetColumnIndex(j + 1);
					u8 byte = ppu_mem.read(addr + j);
					if (ImGui::InputScalar("##byte", ImGuiDataType_U8, &byte, 0, 0, "%02x"))
						ppu_mem.write(addr + j, byte);
					ImGui::PopID();
				}
			}
		}

		ImGui::EndTable();
		ImGui::End();
	}

	void draw_framebuffer(RefPtr<RHI> rhi, CPUMemory& cpu_mem, PPUMemory& ppu_mem) {
		if (ImGui::Begin("PPU display")) {
			ImGui::Text("Frame %d, scanline %d, dot %d", frame, scanline, dot);
			ImGui::InputScalar("ppuctrl", ImGuiDataType_U8, &cpu_mem.ppu_reg.ppuctrl, 0, 0, "%02x");
			ImGui::InputScalar("ppumask", ImGuiDataType_U8, &cpu_mem.ppu_reg.ppumask, 0, 0, "%02x");
			ImGui::InputScalar("ppustatus", ImGuiDataType_U8, &cpu_mem.ppu_reg.ppustatus, 0, 0, "%02x");
			u16 v = cpu_mem.ppu_reg.v;
			ImGui::InputScalar("ppuaddr (v)", ImGuiDataType_U16, &v, 0, 0, "%04x");
			cpu_mem.ppu_reg.v = v;
			u16 t = cpu_mem.ppu_reg.t;
			ImGui::InputScalar("t", ImGuiDataType_U16, &t, 0, 0, "%04x");
			cpu_mem.ppu_reg.v = v;
			ImGui::InputScalar("ppuscrollX", ImGuiDataType_U8, &cpu_mem.ppu_reg.ppuscrollX, 0, 0, "%02x");
			ImGui::InputScalar("ppuscrollY", ImGuiDataType_U8, &cpu_mem.ppu_reg.ppuscrollY, 0, 0, "%02x");

			if (ImGui::SliderInt("Scroll x", &debug_scroll_x, 0, 511, "%03x"))
				use_debug_scroll = true;
			if (ImGui::SliderInt("Scroll y", &debug_scroll_y, 0, 479, "%03x"))
				use_debug_scroll = true;
			ImGui::Checkbox("Debug scroll", &use_debug_scroll);

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
