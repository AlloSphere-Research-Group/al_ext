#include "al/app/al_DistributedApp.hpp"
#include "al/graphics/al_Font.hpp"
#include "al/math/al_Random.hpp"

#include "al_ext/statedistribution/al_NetworkBarrier.hpp"

using namespace al;

struct State {
  float value = 0.0;
};

struct MyApp : DistributedAppWithState<State> {

  double accumulator = 0.0;
  uint32_t counter = 1;

  void onCreate() override {
    if (isPrimary()) {
      if (!barrier.initServer(10467, "0.0.0.0")) {
        std::cout << "Error initializing barrier" << std::endl;
        quit();
      }
      fps(2);
      std::cout << "Waiting for connections ..." << std::endl;
      auto numConnections = barrier.waitForConnections(3, 15);
      if (numConnections != 3) {
        std::cout << "3 Connections required. Quitting." << std::endl;
        quit();
      }
    } else {
      if (!barrier.initClient(10467, "localhost")) {
        std::cout << "Error initializing client" << std::endl;
        quit();
      }
    }
  }

  void onAnimate(double dt) override {
    barrier.synchronize();
    accumulator += (counter % 10) * state().value;
    if (isPrimary()) {
      state().value = rnd::uniformS();
    }
    counter++;
  }

  void onDraw(Graphics &g) override {
    g.clear(0);
    FontRenderer::render(g, std::to_string(accumulator).c_str(), {0, 0, -4});
    FontRenderer::render(g, std::to_string(counter).c_str(), {0, -0.5, -4});
  }

  void onExit() override { barrier.cleanup(); }

  NetworkBarrier barrier;
};

int main() {
  MyApp app;
  app.start();

  return 0;
}
