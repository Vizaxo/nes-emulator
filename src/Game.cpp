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

	void tick(float deltaTime) override {
		if (single_step_debugging && !first_tick)
			return;

		first_tick = false;
		for (int i = 0; i < 100; ++i) {
			nes.tick();
			executing_addr = nes.cpu.fetch_addr;
			if (nes.cpu.fetching) {
				if (disassembly[nes.cpu.fetch_addr].breakpoint) {
					single_step_debugging = true;
				}
			}
		}
		build_disas_table();
	}

	void render(RefPtr<Renderer> renderer, CB::ViewCB viewCB) override
	{
		ImGui::Begin("Debugger");
		ImGui::SliderInt("Break addr", &nes.cpu.debug_break_addr, -1, 0xffff, "%04x");

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
		int num_rows = 0;
		while(addr <= Memory::MEM_MAX) {
			disas_entry& ret = disassembly[addr];
			addr += ret.ins_len;
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
		ImGui::EndTable();
		ImGui::End();
	}
	void cleanup() override {}
	RefPtr<Scene> getScene() override { return &scene; }
	void mouseButtonDown(Mouse::Button b) override {}
	void mouseButtonUp(Mouse::Button b) override {}
	void handleEvent(EventQueue::Event ev) override {}
};

Application* const application = new App();
