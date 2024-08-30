#pragma once

#include <types/Types.h>
#include <core/Assert.h>
#include <core/Log.h>

#define RW_READ 1
#define RW_WRITE 0

static inline Log::Channel cpuChan = {"CPU"};

struct Memory {
	static constexpr u32 MEM_MAX = 0xffff;
	u8 memory[MEM_MAX+1];

	u8& operator[](u16 addr) {
		// TODO: different memory banks
		return memory[addr];
	}

	struct Pinout {
		u16 a;
		u8 d;
		u8 rw : 1; // read/write. If high CPU is reading
	} pinout;

	void debug_setmem(u16 a, u8 d) {
		memory[a] = d;
	}

	void tick() {
		if (pinout.rw == RW_READ)
			pinout.d = memory[pinout.a];
		else
			memory[pinout.a] = pinout.d;
	}
};

#define WRITE_LOW_BYTE(reg, val) do { reg = (reg & 0xff00) | (val & 0x00ff); } while(0);
#define WRITE_HIGH_BYTE(reg, val) do { reg = (reg & 0x00ff) | ((u16)(val & 0x00ff) << 8); } while(0);

struct cpu6502 {
	u8 a;
	u8 x;
	u8 y;
	union {
		u16 pc;
	};
	u8 s; // pointer to low byte of stack addr

	union {
		u8 p; // processor flags
		struct {
			// Not guaranteed to be correct bit order. Might need to adjust if bit order is needed (i.e. if P needs to be read as a unit).

			// Bit 7 to bit 0
			u8 n : 1; // negative
			u8 v : 1; // overflow
			u8 _ : 1; // ignored
			u8 b : 1; // break
			u8 d : 1; // bcd
			u8 i : 1; // irq disable
			u8 z : 1; // zero
			u8 c : 1; // carry
		};
	};

	struct pinout {
		u16 a; // ADDR
		u8 d;  // DATA
		u8 rdy : 1; // READY
		u8 irqN : 1; // Interrupt request
		u8 nmiN : 1; // Non-maskable interrupt requset
		u8 sync : 1; // Goes high during opcode fetch
		u8 resN : 1; // RESET
		u8 so : 1; // Set overflow input
		u8 rw : 1; // Read/write. High to read
	} pinout;

	enum UOP_ID {
		READ_MEM,
		WRITE_PCL,
		WRITE_PCH,
		WRITE_A,
		FETCH,
		DECODE,
		EXECUTE,
	};
	struct uop {
		UOP_ID uop_id;
		u16 data;
	};

	static constexpr uint UOP_QUEUE_NUM = 16;
	uop uops[UOP_QUEUE_NUM];
	uint uop_num;

	void queue_uop(uop u) {
		ASSERT(uop_num < UOP_QUEUE_NUM, "Too many uops in uop queue");
		uops[uop_num++] = u;
	}

	void queue_uop(enum UOP_ID uid, u16 data = 0x0) {
		queue_uop({uid, data});
	}

	uop pop_uop() {
		ASSERT(uop_num > 0, "Trying to pop from empty uop queue");
		uop ret = uops[0];
		for (int i = 0; i < uop_num-1; i++) {
			uops[i] = uops[i+1];
		}
		--uop_num;
		return ret;
	}

	void clear_uop_queue () {
		uop_num = 0;
	}

	void fetch_pc_byte() {
		queue_uop(READ_MEM, pc);
		++pc;
	}

	void decode(u8 opcode) {
		switch (opcode) {
		case 0xA9: // LDA #oper
			fetch_pc_byte();
			queue_uop(WRITE_A);
		}
	}
	
	void tick() {
		if (!pinout.resN) {
			// initialise cpu
			// TODO: 7-cycle reset sequence

			// no guaranteed reset values. Done by the program.
			// a = x = y = pc = s = p = 0;

			clear_uop_queue();
			queue_uop(READ_MEM, 0xfffc);
			queue_uop(WRITE_PCL);
			queue_uop(READ_MEM, 0xfffd);
			queue_uop(WRITE_PCH);
			queue_uop(FETCH);
			return;
		}

		ASSERT(uop_num > 0, "No uops to execute!");

		bool end_cycle = false;
		while (!end_cycle) {
			uop u = pop_uop();
			LOG(Log::INFO, cpuChan, "uOP execute: %d (data %d)", u.uop_id, u.data);
			switch (u.uop_id) {
			case READ_MEM:
				pinout.a = u.data;
				pinout.rw = RW_READ;
				end_cycle = true;
				break;
			case WRITE_PCL:
				WRITE_LOW_BYTE(pc, pinout.d);
				break;
			case WRITE_PCH:
				WRITE_HIGH_BYTE(pc, pinout.d);
				break;
			case FETCH:
				LOG(Log::INFO, cpuChan, "Instruction fetch: %x", pc);
				fetch_pc_byte();
				queue_uop(DECODE);
				break;
			case DECODE:
				decode(pinout.d);
				LOG(Log::INFO, cpuChan, "Instruction decode: %x", pinout.d);
				queue_uop(FETCH);
				break;
			case WRITE_A:
				LOG(Log::INFO, cpuChan, "Wrote %d to register A", pinout.d);
				a = pinout.d;
				break;
			default:
				ASSERT(false, "Unimplemented uop %d", u.uop_id);
				break;
			}
		}
	}
};
