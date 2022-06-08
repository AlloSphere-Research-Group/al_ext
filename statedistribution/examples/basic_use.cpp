
#include "al/app/al_DistributedApp.hpp"
#include "al/graphics/al_Font.hpp"
#include "al/graphics/al_Shapes.hpp"

#include "al_ext/statedistribution/al_CuttleboneDomain.hpp"

// This example shows how to add cuttlebone state distribution in a
// DistributedAppWithState

using namespace al;

// Create the data structure you want to share
struct State {
  al::Color backgroundColor{1.0f, 1.0f, 1.0f, 1.0f};
  al::Pose pose;
  uint64_t counter{0};
};

// Inherit from DistributedAppWithState<State> using the data structure for the
// state as the template argument
struct MyDistributedApp : public DistributedAppWithState<State> {
  Mesh mesh;
  // You can keep a pointer to the cuttlebone domain
  // This can be useful to ask the domain if it is a sender or receiver
  // An alternative way to determine this is using DistributedApp::isPrimary()
  std::shared_ptr<CuttleboneDomain<State>> cuttleboneDomain;

  void onInit() override {
    cuttleboneDomain = CuttleboneDomain<State>::enableCuttlebone(this);
    if (!cuttleboneDomain) {
      std::cerr << "ERROR: Could not start Cuttlebone. Quitting." << std::endl;
      quit();
    }
    addIcosahedron(mesh);
    mesh.primitive(Mesh::LINES);
  }

  void onAnimate(double dt) override {
    // Check if we are a sender or a receiver and write or read from state
    // accordingly
    if (cuttleboneDomain->isSender()) {
      // Write state for sender
      state().backgroundColor.r = float(mouse().x()) / width();
      state().backgroundColor.g = float(mouse().y()) / height();
      state().pose = pose();
      state().counter++;
    } else {
      // Read state for receiver
      // Note that we don't need to read everything here as we will use the
      // values of backgroundColor and counter from the state directly in
      // onDraw()
      pose() = state().pose;
      std::cout << state().counter << std::endl;
    }
  }

  void onDraw(Graphics &g) override {
    // We use the state background color
    // This will work no matter if we are readers or writers of state.
    g.clear(state().backgroundColor);
    g.color(0.5);
    g.draw(mesh);
    FontRenderer::render(g, std::to_string(state().counter).c_str(),
                         Vec3f{0.25, 0.25, -2}, 0.25);
  }
};

int main() {
  MyDistributedApp app;
  app.start();
  return 0;
}
