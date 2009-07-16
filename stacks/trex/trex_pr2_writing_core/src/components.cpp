#include "TeleoReactor.hh"
#include "Assembly.hh"
#include "trex_ros/components.h"
#include <trex_pr2/topological_map.h>
#include <trex_pr2/door_domain_constraints.h>
#include <tf/transform_listener.h>

#include "trex_pr2_writing_core/master_reactor.h"

namespace trex_pr2_writing_core{
  void registerComponents(bool playback, const Assembly& assembly){
    ConstraintEngineId constraintEngine = ((ConstraintEngine*) assembly.getComponent("ConstraintEngine"))->getId();
    // See trex_pr2 for custom constraint/solver binfing
  }

  REGISTER_SCHEMA(trex_pr2_writing_core::registerComponents);

  void registerFactory(bool playback) {
    // Register special reactors
    new TREX::TeleoReactor::ConcreteFactory<trex_pr2_writing_core::MasterReactor>("trex_pr2_writing_core_MasterReactor");
  }

  REGISTER_FACTORY(trex_pr2_writing_core::registerFactory);
}
