#include "Application.h"

#include "core/Log.h"

Log::Channel appChan = {"App"};

struct App : Application {
	App() : Application(L"App") {}
	~App() override {}
	Scene scene;

	void init(RefPtr<Renderer> renderer, PAL::WindowHandle* h) override { LOG(Log::INFO, appChan, "Initialized"); }
	void tick(float deltaTime) override {}
	void render(RefPtr<Renderer> renderer, CB::ViewCB viewCB) override {}
	void cleanup() override {}
	RefPtr<Scene> getScene() override { return &scene; }
	void mouseButtonDown(Mouse::Button b) override {}
	void mouseButtonUp(Mouse::Button b) override {}
	void handleEvent(EventQueue::Event ev) override {}
};

Application* const application = new App();
