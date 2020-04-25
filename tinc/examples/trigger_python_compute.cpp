#include "al/app/al_DistributedApp.hpp"
#include "al/ui/al_ControlGUI.hpp"

#include "al_ext/tinc/al_DiskBuffer.hpp"

using namespace al;

struct TincApp : DistributedApp {

  Parameter range{"range", "", 0.5, "", 0.0, 1.0};
  JsonDiskBuffer dataBuffer{"random_data", "rand_output.json"};

  ControlGUI gui;

  // Local data that will be filled from disk buffer
  VAOMesh mesh;

  void onInit() override {
    // Expose to network to allow receiving messages to load disk data
    dataBuffer.exposeToNetwork(parameterServer());

    parameterServer() << range;
    //    parameterServer().verbose();
  }

  void onCreate() override {
    gui.init();
    gui << range;
  }

  void onAnimate(double dt) override {
    if (dataBuffer.newDataAvailable()) {
      std::cout << "New data available. Swapping buffer" << std::endl;
      auto &data = *dataBuffer.get();
      // Make a mesh from the values in the lists
      mesh.reset();
      mesh.primitive(Mesh::LINE_STRIP);
      float x = -1.0;
      if (data.is_array() && data.size() == 10) {
        auto elements = data.get<std::vector<float>>();
        for (auto val : elements) {

          mesh.vertex(x, val, -4.0);
          x += 0.2;
        }
      }

      mesh.update();
    }
  }

  void onDraw(Graphics &g) override {
    g.clear(0);

    g.color(1.0);
    g.draw(mesh);
    gui.draw(g);
  }

  void onExit() override { gui.cleanup(); }
};

int main() {
  TincApp().start();
  return 0;
}
