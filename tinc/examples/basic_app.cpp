#include "al/app/al_DistributedApp.hpp"
#include "al/ui/al_ControlGUI.hpp"
#include "al/ui/al_Parameter.hpp"

using namespace al;

struct TincApp : DistributedApp {
  Parameter value{"value", "", 0.0, "", 0.0, 1.0};
  ParameterString stringParam{"string", "", "default value"};
  ParameterInt intValue{"int"};
  Trigger resetString{"ResetString"};
  ControlGUI gui;

  void onCreate() {
    gui.init();
    gui << value << stringParam << resetString << intValue;
    parameterServer() << value << stringParam << intValue;
    parameterServer().print();
    parameterServer().verbose();

    resetString.registerChangeCallback(
        [this](bool value) { stringParam.reset(); });
  }

  void onAnimate(double dt) {}

  void onDraw(Graphics &g) {
    g.clear(0.2);
    gui.draw(g);
  }

  void onExit() { gui.cleanup(); }
};

int main() {
  TincApp().start();
  return 0;
}
