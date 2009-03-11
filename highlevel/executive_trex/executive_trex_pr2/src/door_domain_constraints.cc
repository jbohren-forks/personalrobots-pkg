#include <executive_trex_pr2/door_domain_constraints.hh>
#include <executive_trex_pr2/topological_map.h>
namespace executive_trex_pr2 {

  //*******************************************************************************************
  GetDoorDetectionBasePositionConstraint::GetDoorDetectionBasePositionConstraint(const LabelStr& name,
									 const LabelStr& propagatorName,
									 const ConstraintEngineId& constraintEngine,
									 const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _x(static_cast<IntervalDomain&>(getCurrentDomain(variables[0]))),
     _y(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _th(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     _current_x(static_cast<IntervalDomain&>(getCurrentDomain(variables[3]))),
     _current_y(static_cast<IntervalDomain&>(getCurrentDomain(variables[4]))),
     _current_th(static_cast<IntervalDomain&>(getCurrentDomain(variables[5]))),
     _x1(static_cast<IntervalDomain&>(getCurrentDomain(variables[6]))),
     _y1(static_cast<IntervalDomain&>(getCurrentDomain(variables[7]))),
     _x2(static_cast<IntervalDomain&>(getCurrentDomain(variables[8]))),
     _y2(static_cast<IntervalDomain&>(getCurrentDomain(variables[9]))){
    checkError(variables.size() == 10, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  /**
   * If the position is bound, we can make a region query. The result should be intersected on the domain.
   * @todo Integarte proper code from Wim. Also assumes good data
   */
  void GetDoorDetectionBasePositionConstraint::handleExecute(){
    _x.set(3);
    _y.set(2);
    _th.set(3.14 / 2);
  } 

  //*******************************************************************************************
  GetDoorGraspBasePositionConstraint::GetDoorGraspBasePositionConstraint(const LabelStr& name,
									 const LabelStr& propagatorName,
									 const ConstraintEngineId& constraintEngine,
									 const std::vector<ConstrainedVariableId>& variables)
    :Constraint(name, propagatorName, constraintEngine, variables),
     _x(static_cast<IntervalDomain&>(getCurrentDomain(variables[0]))),
     _y(static_cast<IntervalDomain&>(getCurrentDomain(variables[1]))),
     _th(static_cast<IntervalDomain&>(getCurrentDomain(variables[2]))),
     _current_x(static_cast<IntervalDomain&>(getCurrentDomain(variables[3]))),
     _current_y(static_cast<IntervalDomain&>(getCurrentDomain(variables[4]))),
     _current_th(static_cast<IntervalDomain&>(getCurrentDomain(variables[5]))),
     _x1(static_cast<IntervalDomain&>(getCurrentDomain(variables[6]))),
     _y1(static_cast<IntervalDomain&>(getCurrentDomain(variables[7]))),
     _x2(static_cast<IntervalDomain&>(getCurrentDomain(variables[8]))),
     _y2(static_cast<IntervalDomain&>(getCurrentDomain(variables[9]))){
    checkError(variables.size() == 10, "Invalid signature for " << name.toString() << ". Check the constraint signature in the model.");
    checkError(TopologicalMapAdapter::instance() != NULL, "Failed to allocate topological map accessor. Some configuration error.");
  }
    
  /**
   * If the position is bound, we can make a region query. The result should be intersected on the domain.
   * @todo Integarte proper code from Wim. Also assumes good data
   */
  void GetDoorGraspBasePositionConstraint::handleExecute(){    
    _x.set(3);
    _y.set(5);
    _th.set(3.14 / 2);
  }
}
