
#include "al/app/al_DistributedApp.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"

using namespace al;

struct State {
  al::Color backgroundColor{1.0f, 1.0f, 1.0f, 1.0f};
  al::Pose pose;
};

struct MyDistributedApp : public DistributedAppWithState<State> {
  Mesh mesh;
  bool simulator{false};

  void onInit() override {
    auto cuttleboneDomain =
        CuttleboneStateSimulationDomain<State>::enableCuttlebone(this);
    if (!cuttleboneDomain) {
      std::cerr << "ERROR: Could not start Cuttlebone. Quitting." << std::endl;
      quit();
    }
    addIcosahedron(mesh);
  }

  void onAnimate(double dt) override {
    if (simulator) {
      state().backgroundColor.r = float(mouse().x()) / width();
      state().backgroundColor.g = float(mouse().y()) / height();
      state().pose = pose();
    } else {
      pose() = state().pose;
    }
  }

  void onDraw(Graphics& g) override {
    g.clear(state().backgroundColor);
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
