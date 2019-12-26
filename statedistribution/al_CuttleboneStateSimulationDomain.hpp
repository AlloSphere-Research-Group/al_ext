#ifndef INCLUDE_AL_CUTTLEBONEAPP_HPP
#define INCLUDE_AL_CUTTLEBONEAPP_HPP

/* Keehong Youn, 2017, younkeehong@gmail.com
 * Andres Cabrera, 2018, 2019, mantaraya36@gmail.com
 */

#include <iostream>
#include <map>

#include "al/app/al_DistributedApp.hpp"
#include "al/app/al_SimulationDomain.hpp"
#include "al_ext/statedistribution/al_CuttleboneDomain.hpp"

/*
 * MPI and cuttlebone are optional.
 */
#ifdef AL_BUILD_MPI
#include <mpi.h>
#include <unistd.h>
#endif

#ifdef AL_USE_CUTTLEBONE
#include "Cuttlebone/Cuttlebone.hpp"
#endif

namespace al {

template <class TSharedState>
class CuttleboneStateSimulationDomain
    : public StateSimulationDomain<TSharedState> {
 public:
  virtual bool initialize(ComputationDomain *parent = nullptr) {
    return StateSimulationDomain<TSharedState>::initialize(parent);
  }

  TSharedState &state() { return *mState; }

  std::shared_ptr<TSharedState> statePtr() { return mState; }

  virtual std::shared_ptr<StateSendDomain<TSharedState>> addStateSender(
      std::string id = "") {
    auto newDomain =
        this->template newSubDomain<CuttleboneSendDomain<TSharedState>>(false);
    newDomain->setId(id);
    newDomain->setStatePointer(statePtr());
    return newDomain;
  }

  virtual std::shared_ptr<StateReceiveDomain<TSharedState>> addStateReceiver(
      std::string id = "") {
    auto newDomain =
        this->template newSubDomain<CuttleboneReceiveDomain<TSharedState>>(
            true);
    newDomain->setId(id);
    newDomain->setStatePointer(statePtr());
    return newDomain;
  }

  static std::shared_ptr<CuttleboneStateSimulationDomain<TSharedState>>
  enableCuttlebone(DistributedAppWithState<TSharedState> *app) {
    std::shared_ptr<CuttleboneStateSimulationDomain<TSharedState>> cbDomain =
        app->graphicsDomain()
            ->template newSubDomain<
                CuttleboneStateSimulationDomain<TSharedState>>(true);
    app->graphicsDomain()->removeSubDomain(app->simulationDomain());
    if (cbDomain) {
      //      cbDomain->A
      app->simulationDomain() = cbDomain;
      if (app->isPrimary()) {
        auto sender = cbDomain->addStateSender();
        if (app->additionalConfig.find("broadcastAddress") !=
            app->additionalConfig.end()) {
          sender->setAddress(app->additionalConfig["broadcastAddress"]);
        }
        assert(sender);
      } else {
        auto receiver = cbDomain->addStateReceiver();
        assert(receiver);
      }
      if (!cbDomain->initialize(nullptr)) {
        cbDomain = nullptr;
        return nullptr;
      }

    } else {
      std::cerr << "ERROR creating cuttlebone domain" << std::endl;
    }
    return cbDomain;
  }
};

}  // namespace al

#endif  // INCLUDE_AL_CUTTLEBONEAPP_HPP
