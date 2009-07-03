#include "TeleoReactor.hh"
#include "Assembly.hh"
#include "trex_ros/components.h"
#include <trex_pr2/topological_map.h>
#include <trex_pr2/door_domain_constraints.h>
#include <tf/transform_listener.h>

#include "trex_temp_proj/master_reactor.h"

namespace trex_temp_proj{
  void registerComponents(bool playback, const Assembly& assembly){
    ConstraintEngineId constraintEngine = ((ConstraintEngine*) assembly.getComponent("ConstraintEngine"))->getId();
    // See trex_pr2 for custom constraint/solver binfing
  }

  REGISTER_SCHEMA(trex_temp_proj::registerComponents);

  void registerFactory(bool playback) {
    // Register special reactors
    new TREX::TeleoReactor::ConcreteFactory<trex_temp_proj::MasterReactor>("trex_temp_proj_MasterReactor");
  }

  REGISTER_FACTORY(trex_temp_proj::registerFactory);
}
