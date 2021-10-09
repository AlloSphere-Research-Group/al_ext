
#include "al/app/al_DistributedApp.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/ui/al_ControlGUI.hpp"

#include "al_ext/ogre3D/al_Ogre3D.hpp"

using namespace al;

struct State {
  al::Color backgroundColor{1.0f, 1.0f, 1.0f, 1.0f};
  al::Pose pose;
};

struct MyDistributedApp : public App {
  std::shared_ptr<Ogre3DDomain> ogreDomain;
  Ogre::SceneNode *ogreNode;

  Parameter size{"size", "", 1.0f, 0.1f, 10.f};
  ControlGUI gui;

  void onInit() override {
    // Prepend the ogre domain so Ogre draws before allolib
    ogreDomain = defaultWindowDomain()->newSubDomain<Ogre3DDomain>(true);
    ogreDomain->init(defaultWindowDomain().get());
    createOgreScene();

    gui << size;
    gui.init();
  }

  void onAnimate(double dt) override { ogreNode->setScale(size, size, size); }

  void onDraw(Graphics &g) override { gui.draw(g); }

  bool onKeyDown(Keyboard const &k) override { return true; }

  void onExit() override { gui.cleanup(); }

  void createOgreScene() {

    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
        "..\\..\\Media\\models\\", "FileSystem", "General");

    Ogre::SceneManager *sceneManager = ogreDomain->getSceneManager();

    // To add a mesh, first load it into an "Entity"
    Ogre::v1::Entity *ogreEntity = sceneManager->createEntity("knot.mesh");

    // Then create a node for the scene graph
    ogreNode = ogreDomain->getSceneManager()
                   ->getRootSceneNode()
                   ->createChildSceneNode();
    // And attach the mesh entity to the node
    ogreNode->attachObject(ogreEntity);

    // You can control position of the mesh through the node.
    // Ogre uses a diferent scale than allolib. While allolib uses "meters",
    // Ogre uses "cm", so there is something like a 1:100 ratio for scale.
    // This is more of a rule of thumb.
    ogreNode->translate(0, -300, -1000);

    // Lighting
    sceneManager->setAmbientLight(Ogre::ColourValue(.5, .5, .5),
                                  Ogre::ColourValue(.5, .5, .5),
                                  Ogre::Vector3(0, 0, 1));
    //    if(0)
    {
      Ogre::Light *light = sceneManager->createLight();
      Ogre::SceneNode *ogreNode =
          sceneManager->getRootSceneNode()->createChildSceneNode();

      ogreNode->attachObject(light);
      ogreNode->setPosition(20, 80, -500);
    }
  }
};

int main() {

  MyDistributedApp app;

  app.start();
  return 0;
}
