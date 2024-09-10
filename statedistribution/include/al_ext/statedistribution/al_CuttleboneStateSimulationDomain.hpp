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

static bool canUseCuttlebone() {
#ifdef AL_USE_CUTTLEBONE
  return true;
#else
  return false;
#endif
}

// For backward compatibility. Use CuttleboneDomain instead
template <class TSharedState, unsigned PACKET_SIZE = 1400,
          unsigned PORT = 63059>
class [[deprecated(
    "Use CuttleboneDomain instead")]] CuttleboneStateSimulationDomain
    : public CuttleboneDomain<TSharedState, PACKET_SIZE, PORT>{};

} // namespace al

#endif // INCLUDE_AL_CUTTLEBONEAPP_HPP
