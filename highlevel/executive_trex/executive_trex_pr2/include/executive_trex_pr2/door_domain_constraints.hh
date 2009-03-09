
/**
 * @author Conor McGann
 * @brief Implements constraints for accessing functions in the door domain
 */

#ifndef EXECUTIVE_TREX_PR2_DOOR_DOMAIN_CONSTRAINTS_H_
#define EXECUTIVE_TREX_PR2_DOOR_DOMAIN_CONSTRAINTS_H_

#include "Constraint.hh"
#include "IntervalDomain.hh"

using namespace EUROPA;

namespace executive_trex_pr2 {

  /**
   * @brief A function to compute the base position. Can be derived from the door detection code which already implemented this.
   */
  class GetDoorDetectionBasePositionConstraint : public Constraint {
  public:
    
    GetDoorDetectionBasePositionConstraint(const LabelStr& name,
					   const LabelStr& propagatorName,
					   const ConstraintEngineId& constraintEngine,
					   const std::vector<ConstrainedVariableId>& variables);

    virtual void handleExecute();
    
  private:
    IntervalDomain& _x;
    IntervalDomain& _y;
    IntervalDomain& _th;
    IntervalDomain& _current_x;
    IntervalDomain& _current_y;
    IntervalDomain& _current_th;
    IntervalDomain& _x1;
    IntervalDomain& _y1;
    IntervalDomain& _x2;
    IntervalDomain& _y2;
  };

  /**
   * @brief A function to compute the base position. Can be derived from the door detection code which already implemented this.
   */
  class GetDoorGraspBasePositionConstraint : public Constraint {
  public:
    
    GetDoorGraspBasePositionConstraint(const LabelStr& name,
				       const LabelStr& propagatorName,
				       const ConstraintEngineId& constraintEngine,
				       const std::vector<ConstrainedVariableId>& variables);

    virtual void handleExecute();
    
  private:
    IntervalDomain& _x;
    IntervalDomain& _y;
    IntervalDomain& _th;
    IntervalDomain& _current_x;
    IntervalDomain& _current_y;
    IntervalDomain& _current_th;
    IntervalDomain& _x1;
    IntervalDomain& _y1;
    IntervalDomain& _x2;
    IntervalDomain& _y2;
  };

}

#endif
