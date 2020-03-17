#include "al/app/al_DistributedApp.hpp"
#include "al_ext/statedistribution/al_NetworkBarrier.hpp"

using namespace al;

#include <WS2tcpip.h>
#include <WinSock2.h>

struct MyApp : DistributedApp {
  void onInit() override {
    if (!barrier.initServer(10467, "0.0.0.0")) {
      std::cout << "Error initializing barrier" << std::endl;
      quit();
    }
    al_sleep(0.5);
    if (!barrier2.initClient(10467, "localhost")) {
      std::cout << "Error opening client" << std::endl;
    }
  }

  bool onKeyDown(Keyboard const &k) override {
    barrier2.ping();
    return true;
  }

  void onExit() override {
    barrier.cleanup();
    barrier2.cleanup();
  }

  SocketClient client;

  NetworkBarrier barrier;
  NetworkBarrier barrier2;
};

int main() {
  MyApp app;
  app.start();

  return 0;
}
