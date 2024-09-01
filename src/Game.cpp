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

	bool mem_scroll_to_focus = 0;
	u16 mem_focus_addr;

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
	};
	Array<ins_history_entry> ins_history;

	void reset_nes() {
		nes.reset();
		executing_addr = nes.cpu.pc;
		single_step_debugging = true;
		ins_history.reset();
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
			ins_metadata[addr].ins_bytes[1] = nes.mem[addr+2];
			ins_metadata[addr].ins_bytes[2] = nes.mem[addr+3];
		}
	}

	void single_step() {
		ins_history_entry hist = {executing_addr, cpu6502::get_ins_length(executing_addr, &nes.mem)};
		hist.ins_bytes[0] = nes.mem[executing_addr];
		hist.ins_bytes[1] = nes.mem[executing_addr+1];
		hist.ins_bytes[2] = nes.mem[executing_addr+2];
		ins_history.add(hist);

		// Execute until the next instruction is fetched
		do {
			nes.tick();
		} while (!nes.cpu.fetching);

		executing_addr = nes.cpu.pc;
		if (ins_metadata[nes.cpu.pc].breakpoint) {
			single_step_debugging = true;
		}
	}

	void tick(float deltaTime) override {
		update_ins_lengths();
		executing_addr = nes.cpu.pc;

		static constexpr int TICKS_PER_FRAME = 1000;
		for (int i = 0; i < TICKS_PER_FRAME; ++i) {
			if (single_step_debugging)
				return;
			single_step();
		}
	}

	void draw_debugger() {
		ImGui::Begin("Debugger");

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

		static bool scroll_to_end = true;
		bool scroll_to_end_updated_this_frame = ImGui::Checkbox("Scroll to end", &scroll_to_end);
		static ImGuiTableFlags table_flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg
			| ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_Resizable;

		ImGui::BeginTable("##mem", 3, table_flags);

		ImGui::TableSetupColumn("Addr");
		ImGui::TableSetupColumn("Bytes");
		ImGui::TableSetupColumn("Disas");
		ImGui::TableHeadersRow();

		ImGuiListClipper clipper;
		clipper.Begin(ins_history.num());

		while (clipper.Step()) {
			for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
				ins_history_entry& entry = ins_history[i];
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
			}
		}

		float scroll_y = 0;
		scroll_y = ImGui::GetScrollY();
		static float last_requested_scroll_y = scroll_y;

		if (!scroll_to_end_updated_this_frame && scroll_y != last_requested_scroll_y)
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
