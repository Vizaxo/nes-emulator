#pragma once

#include <types/Colour.h>
#include <types/str.h>

#include <cstdio>
#include <stdio.h>

struct palette_t {
	static constexpr int PALETTE_SIZE = 64;
	Colour colours[PALETTE_SIZE];

	static inline palette_t load_palette(str path) {
		palette_t ret;
#pragma warning (disable : 4996)
		FILE* f = std::fopen(path.s, "rb");
		ASSERT(f, "Failed to open file %s", path);

		for (int i = 0; i < PALETTE_SIZE; ++i) {
			ASSERT(fread((void*)&ret.colours[i], 1, 3, f) == 3, "Failed to read 3 bytes from palette file");
			ret.colours[i].a = 0xff;
		}
		return ret;
	}
};
