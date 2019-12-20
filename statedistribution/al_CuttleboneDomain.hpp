#ifndef CUTTLEBONEDOMAIN_H
#define CUTTLEBONEDOMAIN_H

#include <cassert>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stack>
#include <vector>

#include "al/app/al_App.hpp"
#include "al/app/al_ComputationDomain.hpp"
#include "al/spatial/al_Pose.hpp"

#ifdef AL_USE_CUTTLEBONE
#include "Cuttlebone/Cuttlebone.hpp"
#endif

namespace al {

template <class TSharedState>
class CuttleboneReceiveDomain;

template <class TSharedState>
class CuttleboneSendDomain;

/**
 * @brief CuttleboneDomain class
 * @ingroup App
 */
class CuttleboneDomain : public SynchronousDomain {
 public:
  template <class TSharedState = DefaultState>
  std::shared_ptr<CuttleboneSendDomain<TSharedState>> addStateSender(
      std::string id = "", std::shared_ptr<TSharedState> statePtr = nullptr);

  template <class TSharedState = DefaultState>
  std::shared_ptr<CuttleboneReceiveDomain<TSharedState>> addStateReceiver(
      std::string id = "", std::shared_ptr<TSharedState> statePtr = nullptr);

  static std::shared_ptr<CuttleboneDomain> enableCuttlebone(App *app) {
#ifdef AL_USE_CUTTLEBONE
    auto cbDomain = app->graphicsDomain()->newSubDomain<CuttleboneDomain>(true);
    cbDomain->initialize(app->graphicsDomain().get());
    return cbDomain;
#else
    (void)app;
    std::cout << "Cuttlebone support not available. Ignoring enableCuttlebone()"
              << std::endl;
    return nullptr;
#endif
  }

 private:
};

template <class TSharedState = DefaultState>
class CuttleboneReceiveDomain : public StateReceiveDomain<TSharedState> {
 public:
  bool initialize(ComputationDomain *parent = nullptr) override;

  bool tick() override {
    tickSubdomains(true);

    assert(mState);  // State must have been set at this point
#ifdef AL_USE_CUTTLEBONE
    assert(mTaker);
    mQueuedStates = mTaker->get(mState);
    return true;
#else
    return false;
#endif
  }

  bool cleanup(ComputationDomain *parent = nullptr) override {
    cleanupSubdomains(true);
#ifdef AL_USE_CUTTLEBONE
    mTaker->stop();
    mTaker = nullptr;
    cleanupSubdomains(false);
    return true;
#else
    mState = nullptr;
    return false;
#endif
  }

  void configure(uint16_t port = 10100, std::string id = "state",
                 std::string address = "localhost",
                 uint16_t packetSize = 1400) {
    mPort = port;
    mId = id;
    mAddress = address;
    mPacketSize = packetSize;
  }

  std::shared_ptr<TSharedState> state() { return mState; }

  void setStatePointer(std::shared_ptr<TSharedState> ptr) { mState = ptr; }

  void lockState() {
#ifdef AL_USE_CUTTLEBONE
    mRecvLock.lock();
#endif
  }

  void unlockState() {
#ifdef AL_USE_CUTTLEBONE
    mRecvLock.unlock();
#endif
  }
  int newStates() { return mQueuedStates; }

  std::string id() const { return mId; }

  void setId(const std::string &id) { mId = id; }

 private:
  std::shared_ptr<TSharedState> mState;
  int mQueuedStates{1};

  std::string mAddress{"localhost"};
  uint16_t mPort = 10100;
  uint16_t mPacketSize = 1400;
  std::string mId;

#ifdef AL_USE_CUTTLEBONE
  std::unique_ptr<cuttlebone::Taker<TSharedState>> mTaker;
#endif
};

template <class TSharedState>
bool CuttleboneReceiveDomain<TSharedState>::initialize(
    ComputationDomain *parent) {
  initializeSubdomains(true);
  assert(parent != nullptr);

#ifdef AL_USE_CUTTLEBONE
  mTaker =
      std::make_unique<cuttlebone::Taker<TSharedState, mPacketSize, mPort>>();
  mTaker->start();
  initializeSubdomains(false);
  return true;
#else
  return false;
#endif
}

template <class TSharedState = DefaultState>
class CuttleboneSendDomain : public StateSendDomain<TSharedState> {
 public:
  bool initialize(ComputationDomain *parent = nullptr) override {
    initializeSubdomains(true);

#ifdef AL_USE_CUTTLEBONE
    mMaker =
        std::make_unique<cuttlebone::Taker<TSharedState, mPacketSize, mPort>>();
    mMaker->start();
    initializeSubdomains(false);
    return true;
#else
    return false;
#endif
  }

  bool tick() override {
    tickSubdomains(true);

    assert(mState);  // State must have been set at this point
#ifdef AL_USE_CUTTLEBONE
    assert(mMaker);
    mMaker->set(mState);
    tickSubdomains(false);
    return true;
#else
    return false;
#endif
  }

  bool cleanup(ComputationDomain *parent = nullptr) override {
    cleanupSubdomains(true);
#ifdef AL_USE_CUTTLEBONE
    if (mMaker) {
      mMaker->stop();
    }
    cleanupSubdomains(false);
    return true;
#else
    mState = nullptr;
    return false;
#endif
  }

  void configure(uint16_t port, std::string id = "state",
                 std::string address = "localhost",
                 uint16_t packetSize = 1400) {
    mPort = port;
    mId = id;
    mAddress = address;
    mPacketSize = packetSize;
  }

  std::shared_ptr<TSharedState> state() { return mState; }

  //  void lockState() { mStateLock.lock(); }
  //  void unlockState() {mStateLock.unlock();}

  void setStatePointer(std::shared_ptr<TSharedState> ptr) {
    mStateLock.lock();
    mState = ptr;
    mStateLock.unlock();
  }

  int newStates() { return mQueuedStates; }

  std::string id() const { return mId; }

  void setId(const std::string &id) { mId = id; }

 private:
#ifdef AL_USE_CUTTLEBONE
  std::unique_ptr<cuttlebone::Maker<TSharedState>> mMaker;
#endif

#ifdef AL_USE_CUTTLEBONE
  if (role() & ROLE_SIMULATOR) {
    std::string broadcastAddress = configLoader.gets("broadcastAddress");
    mMaker = std::make_unique<cuttlebone::Maker<TSharedState>>(
        broadcastAddress.c_str());
    mMaker->start();
  } else if (role() & ROLE_RENDERER) {
  }

#endif

  std::string mId = "";
  uint16_t mPort = 10100;
  std::string mAddress{"localhost"};
  uint16_t mPacketSize = 1400;

  std::shared_ptr<TSharedState> mState;
  std::mutex mStateLock;
  int mQueuedStates{0};
};

template <class TSharedState>
std::shared_ptr<CuttleboneSendDomain<TSharedState>>
CuttleboneDomain::addStateSender(std::string id,
                                 std::shared_ptr<TSharedState> statePtr) {
  auto newDomain = newSubDomain<CuttleboneSendDomain<TSharedState>>(false);
  newDomain->setId(id);
  return newDomain;
}

template <class TSharedState>
std::shared_ptr<CuttleboneReceiveDomain<TSharedState>>
CuttleboneDomain::addStateReceiver(std::string id,
                                   std::shared_ptr<TSharedState> statePtr) {
  auto newDomain = newSubDomain<CuttleboneReceiveDomain<TSharedState>>(true);
  newDomain->setId(id);

  return newDomain;
}

}  // namespace al

#endif  // CUTTLEBONEDOMAIN_H
