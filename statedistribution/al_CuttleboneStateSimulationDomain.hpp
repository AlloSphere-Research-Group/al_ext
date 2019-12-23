#ifndef INCLUDE_AL_CUTTLEBONEAPP_HPP
#define INCLUDE_AL_CUTTLEBONEAPP_HPP

/* Andres Cabrera, 2019, mantaraya36@gmail.com
 */

#include <iostream>
#include <map>

#include "al/app/al_DistributedApp.hpp"
#include "al/app/al_SimulationDomain.hpp"
#include "al_ext/statedistribution/al_CuttleboneDomain.hpp"

#ifdef AL_USE_CUTTLEBONE
#include "Cuttlebone/Cuttlebone.hpp"
#endif

namespace al {

template <class TSharedState>
class CuttleboneStateSimulationDomain
    : public StateDistributionDomain<TSharedState> {
 public:
  virtual bool initialize(ComputationDomain *parent = nullptr) {
    return StateDistributionDomain<TSharedState>::initialize(parent);
  }

  virtual std::shared_ptr<StateSendDomain<TSharedState>> addStateSender(
      std::string id = "") {
    auto newDomain =
        this->template newSubDomain<CuttleboneSendDomain<TSharedState>>(false);
    newDomain->setId(id);
    newDomain->setStatePointer(this->statePtr());
    this->mIsSender = true;
    return newDomain;
  }

  virtual std::shared_ptr<StateReceiveDomain<TSharedState>> addStateReceiver(
      std::string id = "") {
    auto newDomain =
        this->template newSubDomain<CuttleboneReceiveDomain<TSharedState>>(
            true);
    newDomain->setId(id);
    newDomain->setStatePointer(this->statePtr());
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
      app->mSimulationDomain = cbDomain;

      cbDomain->simulationFunction =
          std::bind(&App::onAnimate, app, std::placeholders::_1);
      if (app->isPrimary()) {
        auto sender = cbDomain->addStateSender();
        assert(sender);
        sender->setAddress("127.0.0.1");
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
