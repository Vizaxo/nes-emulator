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
	u16 single_step_addr = 0x0000;

	struct disas_entry {
		bool breakpoint;
		u8 ins_len;
		str disas;
	};
	disas_entry disassembly[Memory::MEM_MAX];

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
			if (nes.cpu.fetching) {
				if (disassembly[nes.cpu.fetch_addr].breakpoint) {
					single_step_debugging = true;
					single_step_addr = nes.cpu.fetch_addr;
				}
			}
		}
		build_disas_table();
	}

	void render(RefPtr<Renderer> renderer, CB::ViewCB viewCB) override
	{
		ImGui::Begin("Debugger");
		ImGui::SliderInt("Break addr", &nes.cpu.debug_break_addr, -1, 0xffff, "%04x");

		u32 addr = 0;
		while (addr < Memory::MEM_MAX) {
			disas_entry& ret = disassembly[addr];
			//ImGui::TextUnformatted(ret.breakpoint ? "o\t" : " \t");
			ImGui::TextUnformatted(ret.disas.s);
			addr += ret.ins_len;
		}
		ImGui::End();
	}
	void cleanup() override {}
	RefPtr<Scene> getScene() override { return &scene; }
	void mouseButtonDown(Mouse::Button b) override {}
	void mouseButtonUp(Mouse::Button b) override {}
	void handleEvent(EventQueue::Event ev) override {}
};

Application* const application = new App();
