#include "al/app/al_DistributedApp.hpp"
#include "al/graphics/al_Image.hpp"
#include "al/ui/al_ControlGUI.hpp"

#include "al_ext/tinc/al_DiskBuffer.hpp"

using namespace al;

struct TincApp : DistributedApp {

  Parameter param{"param", "", 0.5, "", 0.0, 1.0};

  ImageDiskBuffer dataBuffer{"graph", "output.png"};

  ControlGUI gui;

  Texture graphTex;

  void onInit() override {
    parameterServer() << param;
    dataBuffer.exposeToNetwork(parameterServer());
  }

  void onCreate() override {
    gui.init();
    gui << param;

    graphTex.create2D(512, 512);
  }

  void onAnimate(double dt) override {
    if (dataBuffer.newDataAvailable()) {
      if (dataBuffer.get()->array().size() == 0) {
        std::cout << "failed to load image" << std::endl;
      }
      std::cout << "loaded image size: " << dataBuffer.get()->width() << ", "
                << dataBuffer.get()->height() << std::endl;

      graphTex.resize(dataBuffer.get()->width(), dataBuffer.get()->height());
      graphTex.submit(dataBuffer.get()->array().data(), GL_RGBA,
                      GL_UNSIGNED_BYTE);

      graphTex.filter(Texture::LINEAR);
    }
  }

  void onDraw(Graphics &g) override {
    g.clear(0);

    g.pushMatrix();
    g.translate(0, 0, -4);
    g.quad(graphTex, -1, 1, 2, -1.5);
    g.popMatrix();
    gui.draw(g);
  }

  void onExit() override { gui.cleanup(); }
};

int main() {
  TincApp().start();
  return 0;
}
