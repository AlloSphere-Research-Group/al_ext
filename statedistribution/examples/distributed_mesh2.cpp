//#include <string>

#include "al/app/al_DistributedApp.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/math/al_Random.hpp"
#include "al/scene/al_DistributedScene.hpp"

#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"
#include "al_ext/statedistribution/al_Serialize.hpp"

// This example shows using shared distributed state to control voices from
// DynamicScene
// For more information on dynamic scene and PolySynth see
// allolib/examples/scene or allolib_playground/tutorials/sequencing

// Because DynamicScene allows using more voices that the state can potentially
// hold, only a subset of the voices mesh are sent on every frame
using namespace al;

const size_t maxMeshDataSize = 256;
const size_t maxVoices = 3;

struct SerializedMesh {
  uint16_t id = 0;
  char meshData[maxMeshDataSize];
  size_t meshVertices = 0;
  size_t meshIndeces = 0;
  size_t meshColors = 0;
};

// Shared state sent to renderer application
struct SharedState {
  SerializedMesh meshes[maxVoices];
};

// Define a voice to draw a mesh
struct MeshVoice : public PositionedVoice {
  Mesh mesh;

  void init() override {
    mesh.primitive(Mesh::TRIANGLE_STRIP);
    registerParameter(parameterPose()); // Send pose to replica instances
  }

  void onTriggerOn() override { setPose(Vec3d(-2, 0, 0)); }

  void update(double dt) override {
    if (!mIsReplica) {
      auto p = pose();
      p.pos().x = p.pos().x + 0.01;
      if (p.pos().x >= 2) {
        free();
      }
      setPose(p);
    }
  }

  void onProcess(Graphics &g) override {
    g.polygonFill();
    g.meshColor();
    g.draw(mesh); // Draw the mesh
  }
};

class MyApp : public DistributedAppWithState<SharedState> {
public:
  DistributedScene scene{TimeMasterMode::TIME_MASTER_GRAPHICS};

  void onCreate() override {
    // Set the camera to view the scene
    nav().pos(Vec3d(0, 0, 8));

    navControl().active(false);

    scene.registerSynthClass<MeshVoice>();
    registerDynamicScene(scene);

    auto cuttleboneDomain =
        CuttleboneDomain<SharedState>::enableCuttlebone(this);

    if (!cuttleboneDomain) {
      std::cerr << "ERROR: Could not start Cuttlebone. Quitting." << std::endl;
      quit();
    }
  }

  void onAnimate(double dt) override {
    scene.update(dt);
    if (isPrimary()) {
      // We can get away with this here as the master clock is the graphics
      // clock. There is no guarantee otherwise that this linked list will not
      // change while we are using it.
      auto *voice = scene.getActiveVoices();
      size_t counter = 0;
      while (voice && counter < maxVoices) {
        // serialize mesh into state
        if (!ser::serializeMesh(
                ((MeshVoice *)voice)->mesh, state().meshes[counter].meshData,
                state().meshes[counter].meshVertices,
                state().meshes[counter].meshIndeces,
                state().meshes[counter].meshColors, maxMeshDataSize)) {
          std::cerr << "ERROR: could not serialize mesh" << std::endl;
        } else {
          // Prepare the next voice if allowed
          if (counter < maxVoices) {
            state().meshes[counter].id = voice->id();
            voice = voice->next;
          }
        }
        counter++;
      }
    } else {
      // For the receiver, we write the mesh according to voice id
      auto *voice = scene.getActiveVoices();

      while (voice) {
        SerializedMesh *m = nullptr;
        for (size_t i = 0; i < maxVoices; i++) {
          if (state().meshes[i].id == voice->id()) {
            m = &state().meshes[i];
            break;
          }
        }
        if (m) {
          ser::deserializeMesh(((MeshVoice *)voice)->mesh, m->meshData,
                               m->meshVertices, m->meshIndeces, m->meshColors);
        }
        voice = voice->next;
      }
    }
  }

  void onDraw(Graphics &g) override {
    g.clear(0);
    scene.render(g);
  }

  bool onKeyDown(Keyboard const &k) override {
    if (k.key() == 'o') {
      // 'o' will turn off omni rendering
      if (omniRendering) {
        omniRendering->drawOmni = !omniRendering->drawOmni;
      } else {
        std::cout << "Not doing omni rendering" << std::endl;
      }
    } else {
      // If this is primary node, then trigger a new moving mesh
      if (isPrimary()) {
        auto *voice = scene.getVoice<MeshVoice>();
        voice->mesh.reset();

        for (int i = 0; i < maxMeshDataSize / (4 * 7); i++) {
          voice->mesh.vertex(rnd::uniformS(), rnd::uniformS(), rnd::uniformS());
          voice->mesh.color(rnd::uniform(), rnd::uniform(), rnd::uniform(),
                            1.0f);
        }

        scene.triggerOn(voice);
      }
    }
    return true;
  }

private:
};

int main() {
  MyApp app;
  app.start();
  return 0;
}
