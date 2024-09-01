#include "Application.h"

#include <core/Log.h>

#include "NES.h"

Log::Channel appChan = {"App"};

struct App : Application {
	App() : Application(L"App") {}
	~App() override {}
	Scene scene;

	NES nes;

	bool single_step_debugging = true;
	bool first_tick = true;
	u16 executing_addr = 0x0000;
	u64 cycle_count = 0;
	u64 instruction_count = 0;

	bool mem_scroll_to_focus = 0;
	u16 mem_focus_addr  = 0x0;
	u16 data_break_addr = 0x0;
	u8 data_break_val;
	u8 break_opcode;
	int ticks_per_frame = 1000;
	bool break_on_opcode = false;
	enum data_break_mode_t {
		none = 0x0,
		read = 0x1,
		write = 0x2,
		specific_val = 0x4,
	} data_break_mode = none;

	struct disas_entry {
		bool breakpoint;
		u8 ins_len;
		u8 ins_bytes[3];
	};
	disas_entry ins_metadata[Memory::MEM_MAX+1];

	struct ins_history_entry {
		u16 addr;
		u8 ins_len;
		u8 ins_bytes[3];
		u32 repeat_count = 1;

		inline bool operator==(ins_history_entry& other) {
			if (addr != other.addr || ins_len != other.ins_len)
				return false;
			for (int i = 0; i < ins_len; ++i)
				if (ins_bytes[i] != other.ins_bytes[i])
					return false;
			return true;
		}
		inline bool operator!=(ins_history_entry& other) { return !(*this == other); }
	};
	Array<ins_history_entry> ins_history;
	Array<ins_history_entry> jump_list;

	void reset_nes() {
		nes.reset();
		executing_addr = nes.cpu.pc;
		single_step_debugging = true;
		single_step();
		ins_history.reset();
		jump_list.reset();
		instruction_count = 0;
		cycle_count = 0;
	}

	void init(RefPtr<Renderer> renderer, PAL::WindowHandle h) override {
		LOG(Log::INFO, appChan, "Initialized");
		scene.camera.cam2d = { {0.f,0.f}, 1.f, h };
		scene.camera.type = Camera::Cam2D;

		for (int i = 0; i < Memory::MEM_MAX + 1; ++i) {
			ins_metadata[i].breakpoint = false;
		}

		reset_nes();
	}

	void update_ins_lengths() {
		for (int addr = 0; addr < Memory::MEM_MAX; ++addr) {
			u8 len = cpu6502::get_ins_length(addr, &nes.mem);
			ins_metadata[addr].ins_len = len;
			ins_metadata[addr].ins_bytes[0] = nes.mem[addr];
			ins_metadata[addr].ins_bytes[1] = nes.mem[addr+1];
			ins_metadata[addr].ins_bytes[2] = nes.mem[addr+2];
		}
	}

	void add_ins_history_entry() {
		ins_history_entry entry = {executing_addr, cpu6502::get_ins_length(executing_addr, &nes.mem)};
		entry.ins_bytes[0] = nes.mem[executing_addr];
		entry.ins_bytes[1] = nes.mem[executing_addr+1];
		entry.ins_bytes[2] = nes.mem[executing_addr+2];

		bool add_new_entry = true;
		bool add_jump_entry = false;
		if (ins_history.num() >= 1) {
			ins_history_entry& last = ins_history[ins_history.num() - 1];
			if (last == entry) {
				++last.repeat_count;
				add_new_entry = false;
			}

			// Add jump list entry
			if (last.addr + last.ins_len != entry.addr)
				if (jump_list.num() > 0 && jump_list[jump_list.num()-1] == entry)
					++jump_list[jump_list.num()-1].repeat_count;
				else
					jump_list.add(last);
		}

		if (add_new_entry)
			ins_history.add(entry);
	}

	void single_step() {
		add_ins_history_entry();

		// Execute until the next instruction is fetched
		do {
			nes.tick();
			++cycle_count;

			// Data break
			if (nes.cpu.pinout.a == data_break_addr) {
				if ((nes.cpu.pinout.rw == RW_READ && data_break_mode & read)
					|| (nes.cpu.pinout.rw == RW_WRITE && data_break_mode & write)) {
					if (!(data_break_mode & specific_val) || nes.mem.pinout.d == data_break_val) {
						single_step_debugging = true;
						break;
					}
				}
			}
		} while (!nes.cpu.fetching);

		executing_addr = nes.cpu.pc;
		if (ins_metadata[nes.cpu.pc].breakpoint) {
			single_step_debugging = true;
		}
		if (break_on_opcode && nes.mem[executing_addr] == break_opcode) {
			single_step_debugging = true;
		}
		++instruction_count;
	}

	void tick(float deltaTime) override {
		update_ins_lengths();
		executing_addr = nes.cpu.pc;

		for (int i = 0; i < ticks_per_frame; ++i) {
			if (single_step_debugging)
				return;
			single_step();
		}
	}

	void draw_debugger() {
		ImGui::Begin("Debugger");

		ImGui::InputInt("Cycles per frame", &ticks_per_frame);

		if (ImGui::Button(single_step_debugging ? "Continue" : "Pause"))
			single_step_debugging = !single_step_debugging;

		ImGui::SameLine();
		if (ImGui::Button("Single step"))
			single_step(); //TODO: probably shouldn't be in the render function.

		ImGui::SameLine();
		static bool focus_on_pc;
		bool focus_on_address = false;
		static u16 focus_address;
		ImGui::SameLine();
		bool focus_on_pc_enabled_this_frame = ImGui::Checkbox("Focus PC", &focus_on_pc);
		if (focus_on_pc) {
			focus_on_address = true;
			focus_address = nes.cpu.pc;
		}

		float resetWidth = ImGui::CalcTextSize("Reset").x + ImGui::GetStyle().FramePadding.x*2.f;
		ImVec2 resetSize(resetWidth, 0.f);
		ImGui::SameLine();
		ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - resetSize.x);
		if (ImGui::Button("Reset", resetSize))
			reset_nes();

		ImGui::Text("Instructions: %ld\tCycles: %ld", instruction_count, cycle_count);

		bool focus_on_address_enabled_this_frame = ImGui::InputScalar("Focus addr", ImGuiDataType_U16, &focus_address, 0, 0, "%04x");
		if (focus_on_address_enabled_this_frame) {
			focus_on_address = true;
			focus_on_pc = false;
		}

		static ImGuiTableFlags table_flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg
			| ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_Resizable
			| ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable;
		ImGui::BeginTable("Disas", 5, table_flags);
		ImGui::TableSetupScrollFreeze(0, 1); // Pin top row
		ImGui::TableSetupColumn("brk", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize, 20.f);
		ImGui::TableSetupColumn("pc", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize, 20.f);
		ImGui::TableSetupColumn("Addr");
		ImGui::TableSetupColumn("Bytes");
		ImGui::TableSetupColumn("Disas");
		ImGui::TableHeadersRow();

		ImGuiListClipper clipper;

		u32 addr = 0;
		u32 focus_addr_row = 0;
		int num_rows = 0;
		while(addr <= Memory::MEM_MAX) {
			disas_entry& ret = ins_metadata[addr];
			addr += ret.ins_len;
			if (addr == focus_address)
				focus_addr_row = num_rows;
			++num_rows;
		}

		clipper.Begin(num_rows);

		addr = 0;

		while (clipper.Step()) {
			// Variable-length instructions means we have to simulate rendering the initial set of hidden rows
			// to know which rows to render in the visible portion.
			for (int row = 0; row < clipper.DisplayStart; ++row) {
				if (addr <= Memory::MEM_MAX) {
					disas_entry& ret = ins_metadata[addr];
					addr += ret.ins_len;
				}
			}

			for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; row++) {
				if (addr <= Memory::MEM_MAX) {
					ImGui::PushID(addr);
					disas_entry& ret = ins_metadata[addr];
					ImGui::TableNextRow();
					ImGui::TableSetColumnIndex(0);
					ImGui::Checkbox("##break", &ins_metadata[addr].breakpoint);

					if (addr == executing_addr) {
						ImGui::TableSetColumnIndex(1);
						ImGui::TextUnformatted("->");
					}
					ImGui::TableSetColumnIndex(2);
					ImGui::Text("%04x", addr);
					ImGui::TableSetColumnIndex(3);
					for (int j = 0; j < ret.ins_len; ++j) {
						ImGui::SameLine();
						ImGui::Text("%02x", ret.ins_bytes[j]);
					}
					ImGui::TableSetColumnIndex(4);
					ImGui::TextUnformatted(cpu6502::disassemble_instruction(addr, &nes.mem).s);
					addr += ret.ins_len;
					ImGui::PopID();
				}
			}

		}

		static float scroll_y = 0;
		scroll_y = ImGui::GetScrollY();
		static float last_requested_scroll_y = scroll_y;

		if (!focus_on_pc_enabled_this_frame && scroll_y != last_requested_scroll_y)
			focus_on_pc = false;

		if (focus_on_address) {
			last_requested_scroll_y = clipper.ItemsHeight * focus_addr_row;
			ImGui::SetScrollY(clipper.ItemsHeight * focus_addr_row);
		}


		ImGui::EndTable();
		ImGui::End();

	}

	void draw_registers() {
		ImGui::Begin("CPU State");
		ImGui::BeginTable("Registers", 4);
		ImGui::TableSetupColumn("Reg");
		ImGui::TableSetupColumn("dec");
		ImGui::TableSetupColumn("hex");
		ImGui::TableSetupColumn("Mem focus");
		ImGui::TableHeadersRow();

		struct reg_entry {
			u8* reg;
			str name;
		};
		static reg_entry regs[5];
		regs[0] = {&nes.cpu.a, "a"};
		regs[1] = {&nes.cpu.x, "x"};
		regs[2] = {&nes.cpu.y, "y"};
		regs[3] = {&nes.cpu.s, "s"};
		regs[4] = {&nes.cpu.p, "p"};

		for (int i = 0; i < 5; ++i) {
			ImGui::PushID(i);
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(0);
			ImGui::TextUnformatted(regs[i].name.s);
			ImGui::TableSetColumnIndex(1);
			ImGui::InputScalar("##dec", ImGuiDataType_U8, regs[i].reg, 0, 0, "%d");
			ImGui::TableSetColumnIndex(2);
			ImGui::InputScalar("##hex", ImGuiDataType_U8, regs[i].reg, 0, 0, "%02x");
			ImGui::PopID();
		}
		ImGui::PushID(5);
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::TextUnformatted("pc");
		ImGui::TableSetColumnIndex(1);
		ImGui::InputScalar("##dec", ImGuiDataType_U16, &nes.cpu.pc, 0, 0, "%d");
		ImGui::TableSetColumnIndex(2);
		ImGui::InputScalar("##hex", ImGuiDataType_U16, &nes.cpu.pc, 0, 0, "%02x");
		ImGui::TableSetColumnIndex(3);
		if (ImGui::Button("Focus")) {
			mem_scroll_to_focus = true;
			mem_focus_addr = nes.cpu.pc;
		}
		ImGui::PopID();

		ImGui::EndTable();

		bool flag;
		flag = nes.cpu.get_flag(cpu6502::flag::N);
		ImGui::Checkbox("N", &flag);
		nes.cpu.set_flag(cpu6502::flag::N, flag);

		flag = nes.cpu.get_flag(cpu6502::flag::V);
		ImGui::SameLine();
		ImGui::Checkbox("V", &flag);
		nes.cpu.set_flag(cpu6502::flag::V, flag);

		flag = nes.cpu.get_flag(cpu6502::flag::B);
		ImGui::SameLine();
		ImGui::Checkbox("B", &flag);
		nes.cpu.set_flag(cpu6502::flag::B, flag);

		flag = nes.cpu.get_flag(cpu6502::flag::D);
		ImGui::SameLine();
		ImGui::Checkbox("D", &flag);
		nes.cpu.set_flag(cpu6502::flag::D, flag);

		flag = nes.cpu.get_flag(cpu6502::flag::I);
		ImGui::SameLine();
		ImGui::Checkbox("I", &flag);
		nes.cpu.set_flag(cpu6502::flag::I, flag);

		flag = nes.cpu.get_flag(cpu6502::flag::Z);
		ImGui::SameLine();
		ImGui::Checkbox("Z", &flag);
		nes.cpu.set_flag(cpu6502::flag::Z, flag);

		flag = nes.cpu.get_flag(cpu6502::flag::C);
		ImGui::SameLine();
		ImGui::Checkbox("C", &flag);
		nes.cpu.set_flag(cpu6502::flag::C, flag);

		ImGui::End();
	}

	void draw_memory_view() {
		ImGui::Begin("Memory");

		mem_scroll_to_focus |= ImGui::InputScalar("Focus addr", ImGuiDataType_U16, &mem_focus_addr, 0, 0, "%04x");
		ImGui::InputScalar("Data break", ImGuiDataType_U16, &data_break_addr, 0, 0, "%04x");
		bool data_break_read = data_break_mode & read;
		ImGui::SameLine();
		ImGui::Checkbox("R", &data_break_read);
		data_break_mode = (data_break_mode_t)((data_break_mode & ~read) | (data_break_read ? read : none));
		bool data_break_write = data_break_mode & write;
		ImGui::SameLine();
		ImGui::Checkbox("W", &data_break_write);
		data_break_mode = (data_break_mode_t)((data_break_mode & ~write) | (data_break_write ? write : none));

		ImGui::InputScalar("##breakval", ImGuiDataType_U8, &data_break_val, 0, 0, "%02x");
		ImGui::SameLine();
		bool data_break_val_only = data_break_mode & specific_val;
		ImGui::Checkbox("Break on val only", &data_break_val_only);
		data_break_mode = (data_break_mode_t)((data_break_mode & ~specific_val) | (data_break_val_only ? specific_val : none));

		ImGui::InputScalar("##breakopcode", ImGuiDataType_U8, &break_opcode, 0, 0, "%02x");
		ImGui::SameLine();
		ImGui::Checkbox("Break on opcode", &break_on_opcode);

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
					ImGui::InputScalar("##byte", ImGuiDataType_U8, &nes.mem.memory[addr + j], 0, 0, "%02x");
					ImGui::PopID();
				}
			}
		}

		if (mem_scroll_to_focus)
			ImGui::SetScrollY(clipper.ItemsHeight * floor(mem_focus_addr / num_columns));
		mem_scroll_to_focus = false;

		ImGui::EndTable();
		ImGui::End();
	}

	void draw_pinout_view() {
		ImGui::Begin("Pinout");
		ImGui::Text("a: %04x", nes.cpu.pinout.a);
		if (nes.cpu.pinout.a != nes.mem.pinout.a)
			ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "mem a: %04x", nes.mem.pinout.a);
		ImGui::Text("d: %02x", nes.cpu.pinout.d);
		if (nes.cpu.pinout.d != nes.mem.pinout.d)
			ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "mem d: %02x", nes.mem.pinout.d);
		ImGui::Text("r/w: %s", nes.cpu.pinout.rw ? "read" : "write");
		if (nes.cpu.pinout.rw != nes.mem.pinout.rw)
			ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "mem rw: %s", nes.mem.pinout.rw ? "read" : "write");
		ImGui::End();
	}

	void draw_ins_history() {
		ImGui::Begin("Instruction history");

		static bool show_jump_list = false;
		bool jump_list_updated_this_frame = ImGui::Checkbox("Jump list", &show_jump_list);

		ImGui::SameLine();
		static bool scroll_to_end = true;
		bool scroll_to_end_updated_this_frame = ImGui::Checkbox("Scroll to end", &scroll_to_end);
		static ImGuiTableFlags table_flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg
			| ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_Resizable;

		static u16 filter_list_addr;
		static bool should_filter_list;
		if (ImGui::InputScalar("##filteraddr", ImGuiDataType_U16, &filter_list_addr, 0, 0, "%04x"))
			should_filter_list = filter_list_addr != 0;

		ImGui::SameLine();
		bool should_filter_list_updated_this_frame = ImGui::Checkbox("Filter list", &should_filter_list);

		ImGui::BeginTable("##mem", 4, table_flags);

		ImGui::TableSetupColumn("Addr");
		ImGui::TableSetupColumn("Bytes");
		ImGui::TableSetupColumn("Disas");
		ImGui::TableSetupColumn("");
		ImGui::TableHeadersRow();

		ImGuiListClipper clipper;

		Array<ins_history_entry>& list_to_show = show_jump_list ? jump_list : ins_history;
		int show_num = 0;
		if (should_filter_list) {
			for (int i = 0; i < list_to_show.num(); ++i)
				if (list_to_show[i].addr == filter_list_addr)
					++show_num;
		} else {
			show_num = list_to_show.num();
		}
		clipper.Begin(show_num);

		while (clipper.Step()) {
			int list_index = 0;
			if (should_filter_list) {
				// Simulate hidden rows
				for (int row = 0; row < clipper.DisplayStart; ++row,++list_index)
					while (list_index < list_to_show.num() && list_to_show[list_index].addr != filter_list_addr)
						++list_index;
			} else {
				list_index = clipper.DisplayStart;
			}

			for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i,++list_index) {
				if (should_filter_list)
					while (list_index < list_to_show.num() && list_to_show[list_index].addr != filter_list_addr)
						++list_index;

				if (!should_filter_list)
					ASSERT(list_index == i, "For non-filtered list, index should be i");

				ins_history_entry& entry = list_to_show[list_index];
				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0);
				ImGui::Text("%04x", entry.addr);
				ImGui::TableSetColumnIndex(1);
				for (int j = 0; j < entry.ins_len; ++j) {
					ImGui::SameLine();
					ImGui::Text("%02x", entry.ins_bytes[j]);
				}
				ImGui::TableSetColumnIndex(2);
				ImGui::Text("%s", cpu6502::disassemble_instruction(entry.addr, &nes.mem).s);

				ImGui::TableSetColumnIndex(3);
				if (entry.repeat_count > 1)
					ImGui::Text("x%d", entry.repeat_count);
			}
		}

		float scroll_y = 0;
		scroll_y = ImGui::GetScrollY();
		static float last_requested_scroll_y = scroll_y;

		// TODO: this doesn't work. 1 frame out? Scroll max y changing between frames?
		if (!scroll_to_end_updated_this_frame
			&& !jump_list_updated_this_frame
			&& !should_filter_list_updated_this_frame
			&& scroll_y != last_requested_scroll_y)
			scroll_to_end = false;

		if (scroll_to_end) {
			int scroll_to = ImGui::GetScrollMaxY();
			ImGui::SetScrollY(scroll_to);
			last_requested_scroll_y = scroll_to;
		}

		ImGui::EndTable();
		ImGui::End();
	}

	void render(RefPtr<Renderer> renderer, CB::ViewCB viewCB) override {
		draw_debugger();
		draw_registers();
		draw_memory_view();
		draw_pinout_view();
		draw_ins_history();
	}

	void cleanup() override {}
	RefPtr<Scene> getScene() override { return &scene; }
	void mouseButtonDown(Mouse::Button b) override {}
	void mouseButtonUp(Mouse::Button b) override {}
	void handleEvent(EventQueue::Event ev) override {}
};

Application* const application = new App();
