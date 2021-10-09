#ifndef CUTTLEBONEDOMAIN_H
#define CUTTLEBONEDOMAIN_H

#include <cassert>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

//#include "al/app/al_App.hpp"
#include "al/app/al_ComputationDomain.hpp"
//#include "al/app/al_StateDistributionDomain.hpp"
//#include "al/spatial/al_Pose.hpp"

#include "Compositor/OgreCompositorWorkspaceDef.h"
#include "OgreEntity.h"
#include "OgreFrameListener.h"
#include "OgreRoot.h"
#include "OgreSceneManager.h"
#include "OgreWindow.h"
#include "OgreWindowEventUtilities.h"

// TODOs
// Resize ogre view on window resize

namespace al {

class GLFWOpenGLWindowDomain;

class Ogre3DDomain : public SynchronousDomain, public Ogre::FrameListener {
public:
  bool init(ComputationDomain *parent = nullptr) override;

  bool cleanup(ComputationDomain *parent = nullptr) override;

  bool tick() override;

  Ogre::SceneManager *getSceneManager();

private:
  bool bQuit{false};
  Ogre::Root *root;

  const Ogre::String workspaceName{"AlloOgre Workspace"};
  Ogre::SceneManager *mSceneManager;
  Ogre::CompositorManager2 *mCompositorManager;
  Ogre::Camera *camera;

  GLFWOpenGLWindowDomain *mOpenGLWindowParent;
};
} // namespace al

#endif // CUTTLEBONEDOMAIN_H
