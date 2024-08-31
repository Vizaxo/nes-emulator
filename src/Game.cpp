#include "Application.h"

#include <core/Log.h>

#include "NES.h"

Log::Channel appChan = {"App"};

struct App : Application {
	App() : Application(L"App") {}
	~App() override {}
	Scene scene;

	NES nes;

	void init(RefPtr<Renderer> renderer, PAL::WindowHandle h) override {
		LOG(Log::INFO, appChan, "Initialized");
		scene.camera.cam2d = { {0.f,0.f}, 1.f, h };
		scene.camera.type = Camera::Cam2D;

		nes.init();
	}

	void tick(float deltaTime) override {
		nes.tick();
	}

	void render(RefPtr<Renderer> renderer, CB::ViewCB viewCB) override
	{
		ImGui::Begin("Debugger");
		ImGui::SliderInt("Break addr", &nes.cpu.debug_break_addr, -1, 0xffff, "%04x");

		u32 addr = 0;
		while (addr < Memory::MEM_MAX) {
			Product<str, u8> ret = cpu6502::disassemble_instruction(addr, &nes.mem);
			ImGui::Text("%s", fst(ret).s);
			addr += snd(ret);
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
