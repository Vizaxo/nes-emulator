#include "Application.h"

#include "core/Log.h"

Log::Channel appChan = {"App"};

struct App : Application {
	App() : Application(L"App") {}
	~App() override {}
	Scene scene;

	void init(RefPtr<Renderer> renderer, PAL::WindowHandle h) override {
		LOG(Log::INFO, appChan, "Initialized");
		scene.camera.cam2d = { {0.f,0.f}, 1.f, h };
		scene.camera.type = Camera::Cam2D;
	}
	void tick(float deltaTime) override {}
	void render(RefPtr<Renderer> renderer, CB::ViewCB viewCB) override {}
	void cleanup() override {}
	RefPtr<Scene> getScene() override { return &scene; }
	void mouseButtonDown(Mouse::Button b) override {}
	void mouseButtonUp(Mouse::Button b) override {}
	void handleEvent(EventQueue::Event ev) override {}
};

Application* const application = new App();
