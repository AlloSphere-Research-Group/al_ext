//#include <string>

#include "al/app/al_DistributedApp.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/math/al_Random.hpp"

#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"
#include "al_ext/statedistribution/al_Serialize.hpp"

using namespace al;

// This example shows how to serialize a Mesh in a shared state

const size_t maxMeshDataSize = 4 * 1024 * 64;

struct SharedState {
  char meshData[maxMeshDataSize];
  size_t meshVertices = 0;
  size_t meshIndeces = 0;
  size_t meshColors = 0;
};

// Inherit from DistributedApp and template it on the shared
// state data struct
class MyApp : public DistributedAppWithState<SharedState> {
public:
  void onCreate() override {
    // Set the camera to view the scene
    nav().pos(Vec3d(0, 0, 8));
    // Prepare mesh to draw a cone
    addCone(mesh);
    mesh.primitive(Mesh::TRIANGLE_STRIP);
    navControl().active(false);

    auto cuttleboneDomain =
        CuttleboneDomain<SharedState>::enableCuttlebone(this);
    if (!cuttleboneDomain) {
      std::cerr << "ERROR: Could not start Cuttlebone. Quitting." << std::endl;
      quit();
    }
    std::cout << "Size of state: " << sizeof(SharedState) << std::endl;
    std::cout << "Bandwidth: "
              << graphicsDomain()->fps() * sizeof(SharedState) * 8 / 10e6
              << " Gb/s" << std::endl;
    if (!isPrimary()) {
      // Don't draw omni, draw regular view
      omniRendering->drawOmni = false;
    }
  }

  void onAnimate(double /*dt*/) override {
    if (isPrimary()) {
      ser::serializeMesh(mesh, state().meshData, state().meshVertices,
                         state().meshIndeces, state().meshColors,
                         maxMeshDataSize);
      //      navControl().active(!isImguiUsingInput());
    } else {
      ser::deserializeMesh(mesh, state().meshData, state().meshVertices,
                           state().meshIndeces, state().meshColors);
    }
  }

  void onDraw(Graphics &g) override {
    g.clear(0);
    g.polygonFill();
    g.meshColor();
    g.draw(mesh); // Draw the mesh
  }

  bool onKeyDown(Keyboard const &k) override {
    if (k.key() == ' ') {
      // The space bar will turn off omni rendering
      if (omniRendering) {
        omniRendering->drawOmni = !omniRendering->drawOmni;
      } else {
        std::cout << "Not doing omni rendering" << std::endl;
      }
    } else {
      mesh.reset();

      for (int i = 0; i < maxMeshDataSize / (4 * 3 * 4); i++) {
        mesh.vertex(rnd::uniformS(), rnd::uniformS(), rnd::uniformS());
        mesh.color(rnd::uniform(), rnd::uniform(), rnd::uniform());
      }
    }
    return true;
  }

private:
  Mesh mesh;
};

int main() {
  MyApp app;
  app.start();
  return 0;
}
