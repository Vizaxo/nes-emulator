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

	struct disas_entry {
		bool breakpoint;
		u8 ins_len;
		str disas;
	};
	disas_entry disassembly[Memory::MEM_MAX+1];

	void init(RefPtr<Renderer> renderer, PAL::WindowHandle h) override {
		LOG(Log::INFO, appChan, "Initialized");
		scene.camera.cam2d = { {0.f,0.f}, 1.f, h };
		scene.camera.type = Camera::Cam2D;

		nes.init();
	}

	void build_disas_table() {
		for (int addr = 0; addr < Memory::MEM_MAX; ++addr) {
			Product<str, u8> ret = cpu6502::disassemble_instruction(addr, &nes.mem);
			disassembly[addr] = {false, snd(ret), fst(ret)};
		}
	}

	void single_step() {
		// Execute until the next instruction is fetched
		do {
			nes.tick();
		} while (!nes.cpu.fetching);

		executing_addr = nes.cpu.fetch_addr;
		if (disassembly[nes.cpu.fetch_addr].breakpoint) {
			single_step_debugging = true;
		}
	}

	void tick(float deltaTime) override {
		if (single_step_debugging && !first_tick)
			return;

		static constexpr int TICKS_PER_FRAME = 1;
		for (int i = 0; i < TICKS_PER_FRAME; ++i) {
			single_step();
		}

		// todo: figure out when we need to rebuild this
		if (first_tick)
			build_disas_table();
		first_tick = false;
	}

	void draw_debugger() {
		ImGui::Begin("Debugger");
		if (ImGui::Button(single_step_debugging ? "Continue" : "Pause"))
			single_step_debugging = !single_step_debugging;

		ImGui::SameLine();
		if (ImGui::Button("Single step"))
			single_step(); //TODO: probably shouldn't be in the render function.

		static bool focus_on_pc;
		ImGui::SameLine();
		bool focus_on_pc_enabled_this_frame = ImGui::Checkbox("Focus PC", &focus_on_pc);

		static ImGuiTableFlags table_flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg
			| ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_Resizable
			| ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable;
		ImGui::BeginTable("Disas", 3, table_flags);
		ImGui::TableSetupScrollFreeze(0, 1); // Pin top row
		ImGui::TableSetupColumn("brk", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize, 20.f);
		ImGui::TableSetupColumn("pc", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize, 20.f);
		ImGui::TableSetupColumn("Disas");
		ImGui::TableHeadersRow();

		ImGuiListClipper clipper;

		u32 addr = 0;
		u32 pc_row = 0;
		int num_rows = 0;
		while(addr <= Memory::MEM_MAX) {
			disas_entry& ret = disassembly[addr];
			addr += ret.ins_len;
			if (addr == nes.cpu.pc)
				pc_row = num_rows;
			++num_rows;
		}

		clipper.Begin(num_rows);

		addr = 0;

		while (clipper.Step()) {
			// Variable-length instructions means we have to simulate rendering the initial set of hidden rows
			// to know which rows to render in the visible portion.
			for (int row = 0; row < clipper.DisplayStart; ++row) {
				if (addr <= Memory::MEM_MAX) {
					disas_entry& ret = disassembly[addr];
					addr += ret.ins_len;
				}
			}

			for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; row++) {
				if (addr <= Memory::MEM_MAX) {
					ImGui::PushID(addr);
					disas_entry& ret = disassembly[addr];
					ImGui::TableNextRow();
					ImGui::TableSetColumnIndex(0);
					ImGui::Checkbox("##break", &disassembly[addr].breakpoint);

					if (addr == executing_addr) {
						ImGui::TableSetColumnIndex(1);
						ImGui::TextUnformatted("->");
					}
					ImGui::TableSetColumnIndex(2);
					ImGui::TextUnformatted(ret.disas.s);
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

		if (focus_on_pc) {
			last_requested_scroll_y = clipper.ItemsHeight * pc_row;
			ImGui::SetScrollY(clipper.ItemsHeight * pc_row);
		}


		ImGui::EndTable();
		ImGui::End();

	}

	void draw_registers() {
		ImGui::Begin("CPU State");
		ImGui::BeginTable("Registers", 3);
		ImGui::TableSetupColumn("Reg");
		ImGui::TableSetupColumn("dec");
		ImGui::TableSetupColumn("hex");
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
			ImGui::InputScalar("##dec", ImGuiDataType_U8, &nes.cpu.a, 0, 0, "%d");
			ImGui::TableSetColumnIndex(2);
			ImGui::InputScalar("##hex", ImGuiDataType_U8, &nes.cpu.a, 0, 0, "%02x");
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
		ImGui::PopID();

		ImGui::EndTable();
		ImGui::End();
	}

	void draw_memory_view() {
		ImGui::Begin("Memory");

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
					ImGui::InputScalar("", ImGuiDataType_U8, &nes.mem.memory[addr + j], 0, 0, "%02x");
					ImGui::PopID();
				}
			}
		}

		ImGui::EndTable();
		ImGui::End();
	}

	void render(RefPtr<Renderer> renderer, CB::ViewCB viewCB) override {
		draw_debugger();
		draw_registers();
		draw_memory_view();
	}

	void cleanup() override {}
	RefPtr<Scene> getScene() override { return &scene; }
	void mouseButtonDown(Mouse::Button b) override {}
	void mouseButtonUp(Mouse::Button b) override {}
	void handleEvent(EventQueue::Event ev) override {}
};

Application* const application = new App();
