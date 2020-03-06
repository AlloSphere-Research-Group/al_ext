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
    al_sleep(1.0);  // Allow port to settle
    if (!barrierClient1.initClient(10467, "localhost")) {
      std::cout << "Error opening client 1" << std::endl;
    }
    if (!barrierClient2.initClient(10467, "localhost")) {
      std::cout << "Error opening client 2" << std::endl;
    }
  }

  bool onKeyDown(Keyboard const &k) override {
    barrier.pingClients();
    return true;
  }

  void onExit() override {
    barrier.cleanup();
    barrierClient1.cleanup();
    barrierClient2.cleanup();
  }

  NetworkBarrier barrier;
  NetworkBarrier barrierClient1;
  NetworkBarrier barrierClient2;
};

int main() {
  MyApp app;
  app.start();

  return 0;
}
