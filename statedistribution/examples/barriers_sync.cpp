#include "al/app/al_DistributedApp.hpp"
#include "al_ext/statedistribution/al_NetworkBarrier.hpp"

using namespace al;

//#include <WS2tcpip.h>
//#include <WinSock2.h>

#include "al/math/al_Random.hpp"

struct MyApp : DistributedApp {
  void onInit() override {
    if (isPrimary()) {
      if (!barrier.initServer(10467, "0.0.0.0")) {
        std::cout << "Error initializing barrier" << std::endl;
        quit();
      }
      fps(2);
    } else {
      if (!barrier.initClient(10467, "localhost")) {
        std::cout << "Error initializing client" << std::endl;
        quit();
      }
      // Even though the secondary runs at 30, it will sync to the 2fps of
      // the server
      fps(30);
    }
  }

  void onAnimate(double dt) {
    barrier.synchronize();
    std::cout << "sync" << std::endl;
  }

  void onDraw(Graphics &g) override {
    // background changes randomly on every frame
    g.clear(rnd::uniform());
  }

  void onExit() override { barrier.cleanup(); }

  NetworkBarrier barrier;
};

int main() {
  MyApp app;
  app.start();

  return 0;
}
