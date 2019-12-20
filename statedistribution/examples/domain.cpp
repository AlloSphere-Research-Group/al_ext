
#include "al/app/al_App.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al_ext/statedistribution/al_CuttleboneDomain.hpp"

using namespace al;

// This example shows usage of the CuttleboneDomain. Although in most cases
// you will want to use CuttleboneStateSimulationDomain to distribute state,
// Using the Cuttlebone domain directly requires a little more work, but gives
// you the flexibility to sync multiple states to or from different machines

struct State {
  al::Color backgroundColor{1.0f, 1.0f, 1.0f, 1.0f};
  al::Pose pose;
};

struct MyDistributedApp : public App {
  Mesh mesh;
  bool simulator{false};

  std::shared_ptr<CuttleboneDomain> cuttleboneDomain;
  std::shared_ptr<State> sharedState{new State};

  MyDistributedApp() {
    // Use the static function to insert the cuttlebone domain
    // We need to do this in the constructor as domains get activated on the
    // call to start() for the app
    cuttleboneDomain = CuttleboneDomain::enableCuttlebone(this);
    if (cuttleboneDomain) {
      int mtu = 1400;  // size of network packages
      if (simulator) {
        // On the simulator machine, we add a state sender
        auto sender = cuttleboneDomain->addStateSender("", sharedState);
        assert(sender);
        // Then we can configure the
        sender->configure(11300, "", "192.168.0.255", mtu);
      } else {
        // Same for the receiver nodes
        auto receiver = cuttleboneDomain->addStateReceiver("", sharedState);
        assert(receiver);
        receiver->configure(11300, "", "localhost", mtu);
      }
    } else {
      std::cerr << "ERROR creating cuttlebone domain. Quitting." << std::endl;
      quit();
    }
  }

  void onInit() override { addIcosahedron(mesh); }

  void onAnimate(double dt) override {
    if (simulator) {
      sharedState->backgroundColor.r = float(mouse().x()) / width();
      sharedState->backgroundColor.g = float(mouse().y()) / height();
      sharedState->pose = pose();
    } else {
      pose() = sharedState->pose;
    }
  }

  void onDraw(Graphics& g) override {
    g.clear(sharedState->backgroundColor);
    g.color(0.5);
    g.draw(mesh);
  }
};

int main(int argc, char* argv[]) {
  MyDistributedApp app;
  if (argc == 1 || strncmp(argv[1], "-r", 2)) {
    app.simulator = true;
  }
  app.start();
  return 0;
}
