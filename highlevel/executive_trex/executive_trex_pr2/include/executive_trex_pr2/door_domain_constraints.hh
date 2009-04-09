
/**
 * @author Conor McGann
 * @brief Implements constraints for accessing functions in the door domain
 */

#ifndef EXECUTIVE_TREX_PR2_DOOR_DOMAIN_CONSTRAINTS_H_
#define EXECUTIVE_TREX_PR2_DOOR_DOMAIN_CONSTRAINTS_H_

#include "Constraint.hh"
#include "IntervalDomain.hh"
#include "ConstrainedVariable.hh"
#include "Utilities.hh"
#include "Token.hh"
#include "StringDomain.hh"

using namespace EUROPA;

namespace executive_trex_pr2 {

  /**
   * @brief A function to compute the base position. Can be derived from the door detection code which already implemented this.
   */
  class GetRobotPoseForDoorConstraint : public Constraint {
  public:
    
    GetRobotPoseForDoorConstraint(const LabelStr& name,
				  const LabelStr& propagatorName,
				  const ConstraintEngineId& constraintEngine,
				  const std::vector<ConstrainedVariableId>& variables);

    static std::vector<ConstrainedVariableId> makeScopeForToken(const std::vector<ConstrainedVariableId>& variables);

    virtual void handleExecute();
    
  private:
    IntervalDomain& _x;
    IntervalDomain& _y;
    IntervalDomain& _th;
    const TokenId _token_id;
    IntervalDomain& _range;
  };

}

#endif
