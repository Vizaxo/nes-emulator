#include "Application.h"

#include <core/Log.h>

#include "6502.h"

Log::Channel appChan = {"App"};

struct App : Application {
	App() : Application(L"App") {}
	~App() override {}
	Scene scene;

	cpu6502 cpu;
	Memory mem;

	void init(RefPtr<Renderer> renderer, PAL::WindowHandle h) override {
		LOG(Log::INFO, appChan, "Initialized");
		scene.camera.cam2d = { {0.f,0.f}, 1.f, h };
		scene.camera.type = Camera::Cam2D;

		cpu.pinout.resN = false;
		for (int i = 0; i < 20; i++)
			cpu.tick();
		mem.debug_setmem(0xfffc, 0x00);
		mem.debug_setmem(0xfffd, 0x01);
		mem.debug_setmem(0x0100, 0xA9);
		mem.debug_setmem(0x0101, 42);
	}
	void tick(float deltaTime) override {
		cpu.pinout.resN = true;
		cpu.tick();

		mem.pinout.a = cpu.pinout.a;
		mem.tick();

		mem.pinout.rw = cpu.pinout.rw;
		if (cpu.pinout.rw)
			cpu.pinout.d = mem.pinout.d; // cpu read
		else
			mem.pinout.d = cpu.pinout.d; // cpu write
	}
	void render(RefPtr<Renderer> renderer, CB::ViewCB viewCB) override {}
	void cleanup() override {}
	RefPtr<Scene> getScene() override { return &scene; }
	void mouseButtonDown(Mouse::Button b) override {}
	void mouseButtonUp(Mouse::Button b) override {}
	void handleEvent(EventQueue::Event ev) override {}
};

Application* const application = new App();
