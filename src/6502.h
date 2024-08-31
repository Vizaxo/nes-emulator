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
	u16 tmp_internal;
	u16 tmp_b;

	union {
		u8 p; // processor flags
		struct {
			// Not guaranteed to be correct bit order. Might need to adjust if bit order is needed (i.e. if P needs to be read as a unit).

			/*
			// Bit 7 to bit 0
			u8 n : 1; // negative
			u8 v : 1; // overflow
			u8 _ : 1; // ignored
			u8 b : 1; // break
			u8 d : 1; // bcd
			u8 i : 1; // irq disable
			u8 z : 1; // zero
			u8 c : 1; // carry
			*/
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
		READ_ZPG,
		LDIMM16,

		FETCH,
		DECODE,
		SINGLE_BYTE_INS_DELAY,
		CATCH_INFINITE_LOOP,

		// EXECUTE uOPs
		OR,
		AND,
		XOR,
		ADC,
		ADC16_NOFLAG,
		ADD_NOFLAG,
		MOV,
		MOV16,
		CMP,
		SBC,
		ASL,
		ROL,
		LSR,
		ROR,
		DEC,
		INC,
		INC16,
		INC_NOFLAG,
		NOP,
		CLEAR_FLAG,
		SET_FLAG,
		BRANCH_FLAG_SET,
		BRANCH_FLAG_UNSET,
	};
	enum uop_target {
		A,
		X,
		Y,
		S,
		pc16,
		pcl,
		pch,
		tmp,
		tmp_high,
		tmp16,
		tmp_bl,
		tmp_bh,
		tmp_b16,
		mem,
	};
	enum flag {
		C = 0,
		Z,
		I,
		D,
		B,
		_, //ignored
		V,
		N,
	};
	static_assert(N == 7);

	u8 get_flag_bit(flag f) {
		return (u8)f;
	}

	u8 get_flag(flag f) {
		return (p >> get_flag_bit(f)) & 1u;
	}

	void set_flag(flag f, bool set) {
		if (set)
			p = p | (1 << get_flag_bit(f));
		else
			p = p & ~(1 << get_flag_bit(f));
	}

	struct uop {
		UOP_ID uop_id;
		uop_target target;
		union {
			u16 data;
			uop_target src;
		};
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
			ASL,
			ROL,
			LSR,
			ROR,
			STX,
			LDX,
			STY,
			LDY,
			DEC,
			INC,
			BIT,
			JMP,
			CPY,
			CPX,
			BPL,
			BMI,
			BVC,
			BVS,
			BCC,
			BCS,
			BNE,
			BEQ,
			BRK,
			JSR,
			RTI,
			RTS,
			PHP,
			PLP,
			PHA,
			PLA,
			DEY,
			TAY,
			INY,
			INX,
			CLC,
			SEC,
			CLI,
			SEI,
			TYA,
			CLV,
			CLD,
			SED,
			TXA,
			TXS,
			TAX,
			TSX,
			DEX,
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
			NOT_IMPLEMENTED_ADDR,
		} addr_mode;
	};
	static constexpr u16 NUM_OPCODES = 0x100;
	op opcode_table[NUM_OPCODES];

	enum pattern_type_t {
		group_one,
		group_two,
	} pattern_type;
	struct pattern_op_def {
		op::op_type_t op_type;
		u8 base;
		pattern_type_t pattern_type;
	};

	Array<pattern_op_def> pattern_ops = {
		{op::ORA, 0x01, group_one},
		{op::AND, 0x21, group_one},
		{op::EOR, 0x41, group_one},
		{op::ADC, 0x61, group_one},
		{op::STA, 0x81, group_one},
		{op::LDA, 0xA1, group_one},
		{op::CMP, 0xC1, group_one},
		{op::SBC, 0xE1, group_one},

		{op::ASL, 0x06, group_two},
		{op::ROL, 0x26, group_two},
		{op::LSR, 0x46, group_two},
		{op::ROR, 0x66, group_two},
		{op::STX, 0x86, group_two},
		{op::LDX, 0xA6, group_two},
		{op::DEC, 0xC6, group_two},
		{op::INC, 0xE6, group_two},
	};
	op::addr_mode_t group_one_addr_modes[8] = {
		op::Xind, op::zpg, op::imm, op::abs,
		op::indY, op::zpgX, op::absY, op::absX,
	};
	op::addr_mode_t group_two_addr_modes[7] = {
		op::zpg, op::A, op::abs,
		op::NOT_IMPLEMENTED_ADDR, op::zpgX, op::NOT_IMPLEMENTED_ADDR, op::absX,
	};

	void build_opcode_table() {
		for (int i = 0; i < NUM_OPCODES; ++i)
			opcode_table[i] = {op::NOT_IMPLEMENTED, op::A};

		for (pattern_op_def pattern_op : pattern_ops) {
			switch (pattern_op.pattern_type) {
			case group_one:
				for (int i = 0; i < 8; ++i)
					opcode_table[pattern_op.base + i * 4] = { pattern_op.op_type, group_one_addr_modes[i] };
				break;
			case group_two:
				for (int i = 0; i < 7; ++i)
					opcode_table[pattern_op.base + i * 4] = { pattern_op.op_type, group_two_addr_modes[i] };
				break;
			}
		}
		opcode_table[0xA2] = {op::LDX, op::imm}; // Technically part of group two, but easier to implement on its own

		// Unimplementing pattern instructions
		ASSERT(opcode_table[0x89].op_type == op::STA, "Expected 0x89=STA imm");
		ASSERT(opcode_table[0x89].addr_mode == op::imm, "Expected 0x89=STA imm");
		opcode_table[0x89] = { op::NOT_IMPLEMENTED }; // STA,imm
		// opcode_table[0x8A] = { op::NOT_IMPLEMENTED }; // STX,A -> TXA
		// opcode_table[0xAA] = { op::NOT_IMPLEMENTED }; // LDX,A -> TAX
		opcode_table[0xCA] = { op::NOT_IMPLEMENTED }; // DEC A
		opcode_table[0xEA] = { op::NOT_IMPLEMENTED }; // INC A

		// Group three

		opcode_table[0x24] = {op::BIT, op::zpg};
		opcode_table[0x2c] = {op::BIT, op::abs};
		opcode_table[0x4c] = {op::JMP, op::abs};
		opcode_table[0x6c] = {op::JMP, op::ind};
		opcode_table[0x84] = {op::STY, op::zpg};
		opcode_table[0x8C] = {op::STY, op::abs};
		opcode_table[0x94] = {op::STY, op::zpgX};

		opcode_table[0xA0] = {op::LDY, op::imm};
		opcode_table[0xA4] = {op::LDY, op::zpg};
		opcode_table[0xAC] = {op::LDY, op::abs};
		opcode_table[0xB4] = {op::LDY, op::zpgX};
		opcode_table[0xBC] = {op::LDY, op::absX};

		opcode_table[0xC0] = {op::CPY, op::imm};
		opcode_table[0xC4] = {op::CPY, op::zpg};
		opcode_table[0xCC] = {op::CPY, op::abs};

		opcode_table[0xE0] = {op::CPX, op::imm};
		opcode_table[0xE4] = {op::CPX, op::zpg};
		opcode_table[0xEC] = {op::CPX, op::abs};

		opcode_table[0x10] = {op::BPL, op::rel};
		opcode_table[0x30] = {op::BMI, op::rel};
		opcode_table[0x50] = {op::BVC, op::rel};
		opcode_table[0x70] = {op::BVS, op::rel};
		opcode_table[0x90] = {op::BCC, op::rel};
		opcode_table[0xB0] = {op::BCS, op::rel};
		opcode_table[0xD0] = {op::BNE, op::rel};
		opcode_table[0xF0] = {op::BEQ, op::rel};

		opcode_table[0x00] = {op::BRK, op::impl};
		opcode_table[0x20] = {op::JSR, op::abs};
		opcode_table[0x40] = {op::RTI, op::impl};
		opcode_table[0x40] = {op::RTS, op::impl};


		// Rest of single-byte instructions
		opcode_table[0x08] = {op::PHP, op::impl};
		opcode_table[0x28] = {op::PLP, op::impl};
		opcode_table[0x48] = {op::PHA, op::impl};
		opcode_table[0x68] = {op::PLA, op::impl};
		opcode_table[0x88] = {op::DEY, op::impl};
		opcode_table[0xA8] = {op::TAY, op::impl};
		opcode_table[0xC8] = {op::INY, op::impl};
		opcode_table[0xE8] = {op::INX, op::impl};
		opcode_table[0x18] = {op::CLC, op::impl};
		opcode_table[0x38] = {op::SEC, op::impl};
		opcode_table[0x58] = {op::CLI, op::impl};
		opcode_table[0x78] = {op::SEI, op::impl};
		opcode_table[0x98] = {op::TYA, op::impl};
		opcode_table[0xB8] = {op::CLV, op::impl};
		opcode_table[0xD8] = {op::CLD, op::impl};
		opcode_table[0xF8] = {op::SED, op::impl};
		opcode_table[0x8A] = {op::TXA, op::impl};
		opcode_table[0x9A] = {op::TXS, op::impl};
		opcode_table[0xAA] = {op::TAX, op::impl};
		opcode_table[0xBA] = {op::TSX, op::impl};
		opcode_table[0xCA] = {op::DEX, op::impl};
		opcode_table[0xEA] = {op::NOP, op::impl};
	}

	static constexpr uint UOP_QUEUE_NUM = 16;
	uop uops[UOP_QUEUE_NUM];
	uint uop_num;

	void queue_uop(uop u) {
		ASSERT(uop_num < UOP_QUEUE_NUM, "Too many uops in uop queue");
		uops[uop_num++] = u;
	}

	void queue_uop(enum UOP_ID uid, uop_target target, u16 data = 0x0) {
		queue_uop({uid, target, data});
	}

	void queue_uop(enum UOP_ID uid, uop_target target, uop_target src) {
		// uOP validation
		if (uid==READ_MEM) ASSERT(target == mem, "READ_MEM target must be mem");

		uop u = { uid, target };
		u.src = src;
		queue_uop(u);
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

	u16* get_target_16(uop_target target) {
		switch (target) {
		case pc16:
			return &pc;
		case tmp16:
			return &tmp_internal;
		case tmp_b16:
			return &tmp_b;
		default:
			ASSERT(false, "Invalid 16-bit target"); return nullptr;
		}
	}

	u16 get_val_16(uop_target target) {
		return *get_target_16(target);
	}

	u8* get_target(uop_target target) {
		switch (target) {
		case A: return &a;
		case X: return &x;
		case Y: return &y;
		case S: return &s;
		case pch: return ((u8*)&pc)+1;
		case pcl: return (u8*)&pc; // Assuming little endian
		case tmp_high: return ((u8*)&tmp_internal)+1;
		case tmp: return (u8*)&tmp_internal;
		case tmp_bl: return ((u8*)&tmp_b)+1;
		case tmp_bh: return (u8*)&tmp_b;
		case mem: return &pinout.d;
		default: ASSERT(false, "Target not implemented"); return nullptr;
		}
	}

	u8 get_val(uop_target target) {
		return *get_target(target);
	}

	void fetch_pc_byte() {
		queue_uop(READ_MEM, mem, pc16);
		queue_uop(INC16, pc16);
	}

	void decode(u8 opcode) {
		op instruction = opcode_table[opcode];

		// Handle jumps first because addressing modes behave slightly differently
		switch (instruction.op_type) {
		case op::JMP:
			switch (instruction.addr_mode) {
			case op::abs:
				fetch_pc_byte();
				queue_uop(MOV, tmp, mem);
				fetch_pc_byte();
				queue_uop(MOV, tmp_high, mem);
				queue_uop(CATCH_INFINITE_LOOP, tmp16);
				queue_uop(MOV16, pc16, tmp16);
				break;
			case op::ind:
				fetch_pc_byte();
				queue_uop(MOV, tmp, mem);
				fetch_pc_byte();
				queue_uop(MOV, tmp_high, mem);
				queue_uop(READ_MEM, mem, tmp16);
				queue_uop(MOV, tmp_bl, mem);
				queue_uop(INC16, tmp16);
				queue_uop(READ_MEM, mem, tmp16);
				queue_uop(MOV, tmp_bh, mem);
				queue_uop(MOV, pc16, tmp_b);
				break;
			default:
				ASSERT(false, "Illegal JMP addressing mode %d", instruction.addr_mode);
				break;
			}
			return; // Fully processed jump
		default:
			break;
			// Fallthrough to non-jmp instructions
		}

		switch (instruction.addr_mode) {
		case op::A:
			queue_uop(SINGLE_BYTE_INS_DELAY, (uop_target)0x0, 0x0);
			break;
		case op::abs:
			fetch_pc_byte();
			queue_uop(MOV, tmp, mem);
			fetch_pc_byte();
			queue_uop(MOV, tmp_high, mem);
			queue_uop(READ_MEM, mem, tmp16);
			break;
		case op::absX:
			fetch_pc_byte();
			queue_uop(MOV, tmp, mem);
			fetch_pc_byte();
			queue_uop(MOV, tmp_high, mem);
			queue_uop(ADC16_NOFLAG, tmp16, x);
			queue_uop(READ_MEM, mem, tmp16);
			break;
		case op::absY:
			fetch_pc_byte();
			queue_uop(MOV, tmp, mem);
			fetch_pc_byte();
			queue_uop(MOV, tmp_high, mem);
			queue_uop(ADC16_NOFLAG, tmp16, x);
			queue_uop(READ_MEM, mem, tmp16);
			break;
		case op::imm:
			fetch_pc_byte();
			break;
		case op::impl:
			queue_uop(SINGLE_BYTE_INS_DELAY, (uop_target)0x0, 0x0);
			break;
		case op::ind:
			fetch_pc_byte();
			queue_uop(MOV, tmp, mem);
			fetch_pc_byte();
			queue_uop(MOV, tmp_high, mem);
			queue_uop(READ_MEM, mem, tmp16);
			queue_uop(MOV, tmp16, mem);
			queue_uop(READ_MEM, mem, tmp16); // indirection
			break;
		case op::Xind:
			fetch_pc_byte();
			queue_uop(MOV, tmp, mem);
			queue_uop(ADD_NOFLAG, tmp, x);
			queue_uop(READ_ZPG, mem, tmp);
			queue_uop(MOV, tmp_bl, mem);
			queue_uop(INC_NOFLAG, tmp);
			queue_uop(READ_ZPG, mem, tmp);
			queue_uop(MOV, tmp_bh, mem);
			queue_uop(READ_MEM, mem, tmp_b16);
			break;
		case op::indY:
			fetch_pc_byte();
			queue_uop(MOV, tmp, mem);
			queue_uop(READ_ZPG, mem, tmp);
			queue_uop(MOV, tmp_bl, mem);
			queue_uop(INC_NOFLAG, tmp);
			queue_uop(READ_ZPG, mem, tmp);
			queue_uop(MOV, tmp_bh, mem);
			queue_uop(ADC16_NOFLAG, tmp_b16, y);
			queue_uop(READ_MEM, mem, tmp_b16);
			break;
		case op::rel:
			fetch_pc_byte();
			break;
		case op::zpg:
			fetch_pc_byte();
			queue_uop(MOV, tmp, mem);
			queue_uop(READ_ZPG, mem, tmp);
			break;
		case op::zpgX:
			fetch_pc_byte();
			queue_uop(MOV, tmp, mem);
			queue_uop(ADD_NOFLAG, tmp, x);
			queue_uop(READ_ZPG, mem, tmp);
			break;
		case op::zpgY:
			fetch_pc_byte();
			queue_uop(MOV, tmp, mem);
			queue_uop(ADD_NOFLAG, tmp, y);
			queue_uop(READ_ZPG, mem, tmp);
			break;
		default:
			ASSERT(false, "Unimplemented addressing mode %d (opcode %x)", instruction.addr_mode, opcode);
			break;
		}

		switch (instruction.op_type) {
		case op::ORA:
			queue_uop(OR, A, mem);
			break;
		case op::AND:
			queue_uop(AND, A, mem);
			break;
		case op::EOR:
			queue_uop(XOR, A, mem);
			break;
		case op::ADC:
			queue_uop(ADC, A, mem);
			break;
		case op::STA:
			queue_uop(MOV, mem, A);
			break;
		case op::LDA:
			queue_uop(MOV, A, mem);
			break;
		case op::CMP:
			queue_uop(CMP, A, mem);
			break;
		case op::SBC:
			queue_uop(SBC, A, mem);
			break;
		case op::ASL:
			queue_uop(ASL, A, mem);
			break;
		case op::ROL:
			queue_uop(ROL, A, mem);
			break;
		case op::LSR:
			queue_uop(LSR, A, mem);
			break;
		case op::ROR:
			queue_uop(ROR, A, mem);
			break;
		case op::STX:
			queue_uop(MOV, mem, X);
			break;
		case op::LDX:
			queue_uop(MOV, X, mem);
			break;
		case op::DEC:
			queue_uop(DEC, tmp, mem);
			queue_uop(MOV, mem, tmp);
			break;
		case op::INC:
			queue_uop(INC, tmp, mem);
			queue_uop(MOV, mem, tmp);
			break;
		case op::TAY:
			queue_uop(MOV, Y, A);
			break;
		case op::TYA:
			queue_uop(MOV, A, Y);
			break;
		case op::TXA:
			queue_uop(MOV, A, X);
			break;
		case op::TXS:
			queue_uop(MOV, S, X);
			break;
		case op::TAX:
			queue_uop(MOV, X, A);
			break;
		case op::TSX:
			queue_uop(MOV, X, S);
			break;
		case op::NOP:
			break;
		case op::CLD:
			queue_uop(CLEAR_FLAG, (uop_target)0xff, flag::D);
			break;
		case op::BNE:
			queue_uop(BRANCH_FLAG_UNSET, mem, (u16)flag::Z);
			break;
		default:
			ASSERT(false, "Unimplemented instruction %d (opcode 0x%x)", instruction.op_type, opcode);
			break;
		}
	}

	struct alu {
		enum alu_op {
			adc,
			sbc,
			sub,
			or_,
			and_,
			xor_,
			inc,
			dec,
			load,
			asl,
			rol,
			lsr,
			ror,
		};
	};

	void alu_op(alu::alu_op op, u8* dest, u8 src, bool setflags = true, bool write_res = true) {
		u16 ret;
		bool setNZ = true;
		bool setC = false;
		bool setV = false;

		switch (op) {
		case alu::adc:
			ret = (u16)(*dest) + (u16)get_flag(flag::C) + (u16)src;
			setC = true;
			setV = true;
			break;
		case alu::sbc:
			ret = (u16)(*dest) - (u16)get_flag(flag::C) - (u16)src;
			setC = true;
			setV = true;
			break;
		case alu::sub:
			ret = *dest - src;
			setC = true;
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
		case alu::asl:
			ret = *dest << 1;
			setC = true;
			break;
		case alu::rol:
			ret = *dest << 1;
			ret |= 0x1 & get_flag(flag::C);
			setC = true;
			break;
		case alu::lsr:
			ret = *dest >> 1;
			set_flag(flag::C, 0x1 & *dest);
			break;
		case alu::ror:
			ret = *dest >> 1;
			ret |= (get_flag(flag::C) & 0x1) << 7;
			set_flag(flag::C, 0x1 & *dest);
			break;
		default:
			ASSERT(false, "Unimplemented ALU operation %d", op);
			break;
		}

		if (setflags && setV)
			// overflow only occurs when both inputs are the same sign, and the output is a different sign
			set_flag(flag::V, sign8(*dest) == sign8(src) ? sign8(ret) != sign8(*dest) : 0);

		if (write_res)
			*dest = (u8)(ret & 0xff);

		if (setflags && setNZ) {
			set_flag(flag::N, sign8(*dest));
			set_flag(flag::Z, *dest == 0);
		}

		if (setflags && setC)
			set_flag(flag::C, ret > 0xff);
	}

	void execute(op::op_type_t op) {
		switch (op) {
		default:
			ASSERT(false, "Unimplemented instruction %d", op);
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
			queue_uop(LDIMM16, tmp16, 0xfffc);
			queue_uop(READ_MEM, mem, tmp16);
			queue_uop(MOV, pcl, mem);
			queue_uop(LDIMM16, tmp16, 0xfffd);
			queue_uop(READ_MEM, mem, tmp16);
			queue_uop(MOV, pch, mem);
			return;
		}

		bool end_cycle = false;
		while (!end_cycle) {
			if (uop_num == 0)
				queue_uop(FETCH, mem);

			uop u = pop_uop();
			switch (u.uop_id) {
			case LDIMM16:
				*get_target_16(u.target) = u.data;
				break;
			case READ_MEM:
				ASSERT(u.target == mem, "READ_MEM must read into mem");
				pinout.a = get_val_16(u.src);
				pinout.rw = RW_READ;
				end_cycle = true;
				break;
			case READ_ZPG:
				ASSERT(u.target == mem, "READ_ZPG must read into mem");
				pinout.a = (u16)get_val(u.src);
				pinout.rw = RW_READ;
				end_cycle = true;
				break;
			case FETCH:
				ASSERT(u.target == mem, "FETCH must fetch from mem")
				LOG(Log::INFO, cpuChan, "Instruction fetch: %x", pc);
				fetch_pc_byte();
				queue_uop(DECODE, mem);
				break;
			case DECODE:
				ASSERT(u.target == mem, "DECODE must decode from memory");
				LOG(Log::INFO, cpuChan, "Instruction decode: %x", get_val(u.target));
				decode(get_val(u.target));
				break;
			case SINGLE_BYTE_INS_DELAY:
				end_cycle = true;
				break;
			case OR:
				alu_op(alu::or_, get_target(u.target), get_val(u.src));
				break;
			case AND:
				alu_op(alu::and_, get_target(u.target), get_val(u.src));
				break;
			case XOR:
				alu_op(alu::xor_, get_target(u.target), get_val(u.src));
				break;
			case ADC16_NOFLAG:
			{
				u16 tgt = get_val_16(u.target);
				*get_target_16(u.target) = get_val_16(u.target) + get_val(u.src);
				if (tgt & 0xff00 != get_val_16(u.target) & 0xff00)
					// High byte affected, takes an extra cycle
					end_cycle = true;
				break;
			}
			case ADD_NOFLAG:
				*get_target(u.target) = get_val(u.target) + get_val(u.src);
				break;
			case ADC:
				alu_op(alu::adc, get_target(u.target), get_val(u.src));
				break;
			case SBC:
				alu_op(alu::sbc, get_target(u.target), get_val(u.src));
				break;
			case MOV:
				if (u.target == A || u.target == X || u.target == Y)
					// So flags are set
					alu_op(alu::load, get_target(u.target), get_val(u.src));
				else
					*get_target(u.target) = get_val((uop_target)u.data);
				break;
			case MOV16:
				*get_target_16(u.target) = get_val_16((uop_target)u.data);
				break;
			case ASL:
				alu_op(alu::asl, get_target(u.target), get_val(u.src));
				break;
			case ROL:
				alu_op(alu::rol, get_target(u.target), get_val(u.src));
				break;
			case LSR:
				alu_op(alu::lsr, get_target(u.target), get_val(u.src));
				break;
			case ROR:
				alu_op(alu::ror, get_target(u.target), get_val(u.src));
				break;
			case DEC:
				alu_op(alu::dec, get_target(u.target), get_val(u.src));
				break;
			case INC:
				alu_op(alu::inc, get_target(u.target), get_val(u.src));
				break;
			case INC16:
				++(*get_target_16(u.target));
				break;
			case INC_NOFLAG:
				++(*get_target(u.target));
				break;
			case NOP:
				break;
			case CMP:
				alu_op(alu::sub, get_target(u.target), get_val(u.src), true, false);
				break;
			case CLEAR_FLAG:
				set_flag((flag)u.data, false);
				break;
			case SET_FLAG:
				set_flag((flag)u.data, true);
				break;
			case BRANCH_FLAG_SET:
				if (get_flag((flag)u.data)) {
					i8 offset = get_val(u.target);
					pc = (i16)pc + offset;
					// TODO: extra cycle for page transition
				}
				break;
			case BRANCH_FLAG_UNSET:
				if (!get_flag((flag)u.data)) {
					i8 offset = get_val(u.target);
					pc = (i16)pc + offset;
					// TODO: extra cycle for page transition
				}
				break;
			case CATCH_INFINITE_LOOP:
			{
				u16 jmp_target = get_val_16(u.target);
				if (jmp_target == pc-3)
					// TODO: this is valid in non-test code as interrupts can escape it.
					ASSERT(false, "Infinite loop detected");
				break;
			}
			default:
				ASSERT(false, "Unimplemented uop %d", u.uop_id);
				break;
			}
		}
	}
};
