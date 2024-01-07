#ifndef CUTTLEBONEDOMAIN_H
#define CUTTLEBONEDOMAIN_H

#include <cassert>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "al/app/al_App.hpp"
#include "al/app/al_ComputationDomain.hpp"
#include "al/app/al_DistributedApp.hpp"
#include "al/app/al_StateDistributionDomain.hpp"
#include "al/spatial/al_Pose.hpp"

#ifdef AL_USE_CUTTLEBONE
#include "Cuttlebone/Cuttlebone.hpp"
#endif

namespace al {

template <class TSharedState, unsigned PACKET_SIZE, unsigned PORT>
class CuttleboneReceiveDomain;

template <class TSharedState, unsigned PACKET_SIZE, unsigned PORT>
class CuttleboneSendDomain;

/**
 * @brief CuttleboneDomain class
 * @ingroup App
 *
 * Class to manage cuttlebone sending and receiving of state.
 *
 * Use the enableCuttlebone() function to set things up for a
 * DistributedAppWithState
 */
template <class TSharedState = DefaultState, unsigned PACKET_SIZE = 1400,
          unsigned PORT = 63059>
class CuttleboneDomain : public StateDistributionDomain<TSharedState> {
public:
  virtual bool init(ComputationDomain *parent = nullptr);

  static std::shared_ptr<CuttleboneDomain<TSharedState, PACKET_SIZE, PORT>>
  enableCuttlebone(DistributedAppWithState<TSharedState> *app,
                   bool prepend = true);

  virtual std::shared_ptr<CuttleboneSendDomain<TSharedState, PACKET_SIZE, PORT>>
  addStateSender(std::string id = "",
                 std::shared_ptr<TSharedState> statePtr = nullptr);

  virtual std::shared_ptr<
      CuttleboneReceiveDomain<TSharedState, PACKET_SIZE, PORT>>
  addStateReceiver(std::string id = "",
                   std::shared_ptr<TSharedState> statePtr = nullptr);

  static bool canUseCuttlebone() {
#ifdef AL_USE_CUTTLEBONE
    return true;
#else
    return false;
#endif
  }
};

template <class TSharedState = DefaultState, unsigned PACKET_SIZE = 1400,
          unsigned PORT = 63059>
class CuttleboneReceiveDomain : public StateReceiveDomain<TSharedState> {
public:
  bool init(ComputationDomain *parent = nullptr) override;

  bool tick() override {
    this->tickSubdomains(true);

    assert(this->mState); // State must have been set at this point
#ifdef AL_USE_CUTTLEBONE
    assert(mTaker);
    this->mQueuedStates = mTaker->get(*(this->mState.get()));
    return true;
#else
    return false;
#endif
  }

  bool cleanup(ComputationDomain * /*parent*/ = nullptr) override {
    this->cleanupSubdomains(true);
#ifdef AL_USE_CUTTLEBONE
    if (mTaker) {
      mTaker->stop();
      mTaker = nullptr;
    }
    this->cleanupSubdomains(false);
    return true;
#else
    //    mState = nullptr;
    return false;
#endif
  }

private:
#ifdef AL_USE_CUTTLEBONE
  std::unique_ptr<cuttlebone::Taker<TSharedState, PACKET_SIZE, PORT>> mTaker;
#endif
};

// Implementation

template <class TSharedState, unsigned PACKET_SIZE, unsigned PORT>
bool CuttleboneDomain<TSharedState, PACKET_SIZE, PORT>::init(
    ComputationDomain *parent) {
  return StateDistributionDomain<TSharedState>::init(parent);
}

template <class TSharedState, unsigned PACKET_SIZE, unsigned PORT>
std::shared_ptr<CuttleboneDomain<TSharedState, PACKET_SIZE, PORT>>
CuttleboneDomain<TSharedState, PACKET_SIZE, PORT>::enableCuttlebone(
    DistributedAppWithState<TSharedState> *app, bool prepend) {
  std::shared_ptr<CuttleboneDomain<TSharedState, PACKET_SIZE, PORT>> cbDomain =
      app->graphicsDomain()
          ->template newSubDomain<
              CuttleboneDomain<TSharedState, PACKET_SIZE, PORT>>(prepend);
  app->graphicsDomain()->removeSubDomain(app->simulationDomain());
  if (cbDomain) {
    app->mSimulationDomain = cbDomain;

    cbDomain->simulationFunction =
        std::bind(&App::onAnimate, app, std::placeholders::_1);
    if (app->hasCapability(CAP_STATE_SEND)) {
      auto sender = cbDomain->addStateSender("", cbDomain->statePtr());
      assert(sender);
      if (app->additionalConfig.find("broadcastAddress") !=
          app->additionalConfig.end()) {
        sender->setAddress(app->additionalConfig["broadcastAddress"]);
      } else {
        sender->setAddress("127.0.0.1");
      }
    } else if (app->hasCapability(CAP_STATE_RECEIVE)) {
      auto receiver = cbDomain->addStateReceiver("", cbDomain->statePtr());

      assert(receiver);
      if (app->additionalConfig.find("broadcastAddress") !=
          app->additionalConfig.end()) {
        receiver->setAddress(app->additionalConfig["broadcastAddress"]);
      } else {
        receiver->setAddress("127.0.0.1");
      }
    } else {
      std::cerr << "Cuttlebone domain enabled, but application has no state "
                   "distribution capabilties enabled"
                << std::endl;
    }
    if (!cbDomain->init(nullptr)) {
      cbDomain = nullptr;
      return nullptr;
    }

  } else {
    std::cerr << "ERROR creating cuttlebone domain" << std::endl;
  }
  return cbDomain;
}

template <class TSharedState, unsigned PACKET_SIZE, unsigned PORT>
bool CuttleboneReceiveDomain<TSharedState, PACKET_SIZE, PORT>::init(
    ComputationDomain *parent) {
  if (!ComputationDomain::mInitialized) {
    bool ret = this->initializeSubdomains(true);
    assert(parent != nullptr);

#ifdef AL_USE_CUTTLEBONE
    if (!mTaker) {
      mTaker = std::make_unique<
          cuttlebone::Taker<TSharedState, PACKET_SIZE, PORT>>();
      mTaker->start();
    }
    ComputationDomain::callInitializeCallbacks();
    ret &= this->initializeSubdomains(false);
    ComputationDomain::mInitialized = true;
    return ret;
  }
  return true;
#else
    return false;
#endif
}

template <class TSharedState = DefaultState, unsigned PACKET_SIZE = 1400,
          unsigned PORT = 63059>
class CuttleboneSendDomain : public StateSendDomain<TSharedState> {
public:
  bool init(ComputationDomain * /*parent*/ = nullptr) override {
    if (!ComputationDomain::mInitialized) {
      this->initializeSubdomains(true);

#ifdef AL_USE_CUTTLEBONE
      mBroadcaster.init(PACKET_SIZE, this->mAddress.c_str(), PORT, false);
      mFrame = 0;

      std::cout << "CuttleboneSendDomain: " << this->mAddress << ":" << PORT
                << std::endl;
      mRunThread = true;
      if (!mSendThread) {
        mSendThread = std::make_unique<std::thread>([&]() {
          while (mRunThread) {
            std::unique_lock<std::mutex> lk(mSendLock);
            mSendCondition.wait(lk);
            cuttlebone::PacketMaker<TSharedState,
                                    cuttlebone::Packet<PACKET_SIZE>>
                packetMaker(*this->mState, mFrame);
            while (packetMaker.fill(p))
              mBroadcaster.send((unsigned char *)&p);
            //          std::cout << "Sent frame " << mFrame << std::endl;
            mFrame++;
          }
        });
      }
      bool ret = this->initializeSubdomains(false);
      ComputationDomain::mInitialized = true;
      return ret;
    }
#else
        return false;
#endif
  }

  bool tick() override {
    this->tickSubdomains(true);

    assert(this->mState); // State must have been set at this point
#ifdef AL_USE_CUTTLEBONE
    mSendCondition.notify_one();
    this->tickSubdomains(false);
    return true;
#else
        return false;
#endif
  }

  bool cleanup(ComputationDomain * /*parent*/ = nullptr) override {
    this->cleanupSubdomains(true);
#ifdef AL_USE_CUTTLEBONE
    //    if (mMaker) {
    //      mMaker->stop();
    //      mMaker = nullptr;
    //    }
    if (mSendThread) {
      mRunThread = false;
      mSendCondition.notify_one();
      mSendThread->join();
      mSendThread = nullptr;
    }
    this->cleanupSubdomains(false);
    return true;
#else
        //    mState = nullptr;
        return false;
#endif
  }

private:
#ifdef AL_USE_CUTTLEBONE

  cuttlebone::Broadcaster mBroadcaster;
  cuttlebone::Packet<PACKET_SIZE> p;
  int mFrame = 0;

  std::mutex mSendLock;
  std::condition_variable mSendCondition;
  bool mRunThread;
  std::unique_ptr<std::thread> mSendThread;
//  std::unique_ptr<cuttlebone::Maker<TSharedState, PACKET_SIZE, PORT>> mMaker;
#endif
};

template <class TSharedState, unsigned PACKET_SIZE, unsigned PORT>
std::shared_ptr<CuttleboneSendDomain<TSharedState, PACKET_SIZE, PORT>>
CuttleboneDomain<TSharedState, PACKET_SIZE, PORT>::addStateSender(
    std::string id, std::shared_ptr<TSharedState> statePtr) {
  auto newDomain = this->template newSubDomain<
      CuttleboneSendDomain<TSharedState, PACKET_SIZE, PORT>>(false);
  newDomain->setId(id);
  newDomain->setStatePointer(statePtr);
  this->mIsSender = true;
  return newDomain;
}

template <class TSharedState, unsigned PACKET_SIZE, unsigned PORT>
std::shared_ptr<CuttleboneReceiveDomain<TSharedState, PACKET_SIZE, PORT>>
CuttleboneDomain<TSharedState, PACKET_SIZE, PORT>::addStateReceiver(
    std::string id, std::shared_ptr<TSharedState> statePtr) {
  auto newDomain = this->template newSubDomain<
      CuttleboneReceiveDomain<TSharedState, PACKET_SIZE, PORT>>(true);
  newDomain->setId(id);
  newDomain->setStatePointer(statePtr);
  return newDomain;
}

} // namespace al

#endif // CUTTLEBONEDOMAIN_H
