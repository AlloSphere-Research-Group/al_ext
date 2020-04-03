#include "al/app/al_DistributedApp.hpp"
#include "al/graphics/al_Font.hpp"

#include "al_ext/tinc/al_DiskBuffer.hpp"

using namespace al;

struct TincApp : DistributedApp {
  DiskBuffer<> dataBuffer{"sine_data", "output.json"};
  DiskBuffer<> dataBuffer2{"background", "background.json"};

  std::string textString;
  std::string currentFile;
  Color bgColor{0.4};
  VAOMesh mesh;

  void onInit() override {
    dataBuffer.exposeToNetwork(parameterServer());
    dataBuffer2.exposeToNetwork(parameterServer());
    parameterServer().verbose();
  }

  void onAnimate(double dt) override {

    if (dataBuffer.newDataAvailable()) {
      std::cout << "New data available. Swapping buffer" << std::endl;
      auto &data = *dataBuffer.get();
      textString = data["random"].dump();
      mesh.reset();
      mesh.primitive(Mesh::LINE_STRIP);
      auto sines = data["sines"];
      int counter = 0;
      for (auto val : sines) {

        mesh.vertex(data["random"].at(counter++).get<double>(),
                    val.get<double>(), -4.0);
      }
      mesh.update();
    }
    if (dataBuffer2.newDataAvailable()) {
      // It's a good idea to validate
      auto data = dataBuffer2.get();
      if (data->is_array() && data->size() == 3) {
        auto bgList = dataBuffer2.get()->get<std::vector<float>>();
        bgColor = Color(bgList[0], bgList[1], bgList[2]);
      }
      currentFile = dataBuffer2.getCurrentFileName();
    }
  }

  void onDraw(Graphics &g) override {
    g.clear(bgColor);

    FontRenderer::render(g, textString.c_str(), {-1, -0.5, -4}, 0.1);

    FontRenderer::render(g, currentFile.c_str(), {-1, -0.7, -4}, 0.1);

    g.color(1.0);
    g.draw(mesh);
  }
};

int main() {
  TincApp().start();
  return 0;
}
