#pragma once

#include <types/Types.h>
#include <core/Assert.h>
#include <core/Log.h>

#define RW_READ 1
#define RW_WRITE 0

static inline Log::Channel cpuChan = {"CPU"};

// Get the 8-bit two's complement sign bit. Takes 16-bit to account for overflows.
u8 sign8(u16 d) {
	return d & 0x80 ? 1 : 0;
}

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

	void debug_set_all_mem(u8 d) {
		for (int i = 0; i <= MEM_MAX; ++i)
			memory[i] = d;
	}

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

		OR_A,
		AND_A,
		XOR_A,
		ADC_A,
		STORE_A,
		LOAD_A,
		CMP_A,
		SBC_A,

		FETCH,
		DECODE,
		EXECUTE,
	};
	struct uop {
		UOP_ID uop_id;
		u16 data;
	};

	struct op {
		enum op_type_t {
			ORA,
			AND,
			EOR,
			ADC,
			STA,
			LDA,
			CMP,
			SBC,
			NOP,
			NOT_IMPLEMENTED,
		} op_type;
		enum addr_mode_t {
			A,
			abs,
			absX,
			absY,
			imm,
			impl,
			ind,
			Xind,
			indY,
			rel,
			zpg,
			zpgX,
			zpgY,
		} addr_mode;
	};
	static constexpr u16 NUM_OPCODES = 0x100;
	op opcode_table[NUM_OPCODES];

	enum pattern_type_t {
		pattern8,
	} pattern_type;
	struct pattern_op_def {
		op::op_type_t op_type;
		u8 base;
		pattern_type_t pattern_type;
	};

	Array<pattern_op_def> pattern_ops = {
		{op::ORA, 0x01, pattern8},
		{op::AND, 0x21, pattern8},
		{op::EOR, 0x41, pattern8},
		{op::ADC, 0x61, pattern8},
		{op::STA, 0x81, pattern8},
		{op::LDA, 0xA1, pattern8},
		{op::CMP, 0xC1, pattern8},
		{op::SBC, 0xE1, pattern8},
	};
	op::addr_mode_t pattern_8_addr_modes[8] = {
		op::Xind, op::zpg, op::imm, op::abs,
		op::indY, op::zpgX, op::absY, op::absX,
	};

	void build_opcode_table() {
		for (int i = 0; i < NUM_OPCODES; ++i)
			opcode_table[i] = {op::NOT_IMPLEMENTED, op::A};

		for (pattern_op_def pattern_op : pattern_ops) {
			switch (pattern_op.pattern_type) {
			case pattern8:
				for (int i = 0; i < 8; ++i)
					opcode_table[pattern_op.base + i * 4] = { pattern_op.op_type, pattern_8_addr_modes[i] };
				break;
			}
		}

		opcode_table[0xEA] = {op::NOP, op::impl};
	}

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
		op instruction = opcode_table[opcode];

		switch (instruction.addr_mode) {
		case op::imm:
			fetch_pc_byte();
			break;
		case op::impl:
			break;
		default:
			ASSERT(false, "Unimplemented addressing mode %d (opcode %x)", instruction.addr_mode, opcode);
		}

		switch(instruction.op_type) {
		case op::ORA:
			queue_uop(OR_A);
			break;
		case op::AND:
			queue_uop(AND_A);
			break;
		case op::EOR:
			queue_uop(XOR_A);
			break;
		case op::ADC:
			queue_uop(ADC_A);
			break;
		case op::STA:
			queue_uop(STORE_A);
			break;
		case op::LDA:
			queue_uop(LOAD_A);
			break;
		case op::CMP:
			queue_uop(CMP_A);
			break;
		case op::SBC:
			queue_uop(SBC_A);
			break;
		case op::NOP:
			break;
		default:
			ASSERT(false, "Unimplemented opcode %d (opcode %x)", instruction.op_type, opcode);
			break;
		}
	}

	struct alu {
		enum alu_op {
			adc,
			sbc,
			or_,
			and_,
			xor_,
			inc,
			dec,
			load,
		};
	};

	void alu_op(alu::alu_op op, u8* dest, u8 src) {
		u16 ret;
		bool setNZ = true;
		bool setCV = false;

		switch (op) {
		case alu::adc:
			ret = (u16)(*dest) + (u16)c + (u16)src;
			setCV = true;
			break;
		case alu::sbc:
			ret = (u16)(*dest) - (u16)c - (u16)src;
			setCV = true;
			break;
		case alu::or_:
			ret = *dest | src;
			break;
		case alu::and_:
			ret = *dest & src;
			break;
		case alu::xor_:
			ret = *dest ^ src;
			break;
		case alu::inc:
			ret = *dest + 1;
			break;
		case alu::dec:
			ret = *dest - 1;
			break;
		case alu::load:
			ret = src;
			break;
		default:
			ASSERT(false, "Unimplemented ALU operation %d", op);
			break;
		}

		*dest = (u8)(ret & 0xff);

		if (setNZ) {
			n = sign8(a);
			z = a == 0;
		}
		if (setCV) {
			c = ret > 0xff;
			// overflow only occurs when both inputs are the same sign, and the output is a different sign
			v = sign8(*dest) == sign8(src) ? sign8(ret) != sign8(*dest) : 0;
		}

	}

	void init() {
		build_opcode_table();
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
				LOG(Log::INFO, cpuChan, "Instruction decode: %x", pinout.d);
				decode(pinout.d);
				queue_uop(FETCH);
				break;

			case OR_A:
				alu_op(alu::or_, &a, pinout.d);
				break;
			case AND_A:
				alu_op(alu::and_, &a, pinout.d);
				break;
			case XOR_A:
				alu_op(alu::xor_, &a, pinout.d);
				break;
			case ADC_A:
				alu_op(alu::adc, &a, pinout.d);
				break;
			case SBC_A:
				alu_op(alu::sbc, &a, pinout.d);
				break;
			case STORE_A:
				pinout.d = a;
				break;
			case LOAD_A:
				alu_op(alu::load, &a, pinout.d);
				break;
			default:
				ASSERT(false, "Unimplemented uop %d", u.uop_id);
				break;
			}
		}
	}
};
