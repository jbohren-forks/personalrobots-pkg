#include "executive_trex_pr2/calc_distance_constraint.h"
#include <math.h>

namespace TREX {

  CalcDistanceConstraint::CalcDistanceConstraint(const LabelStr& name,
					       const LabelStr& propagatorName,
					       const ConstraintEngineId& constraintEngine,
					       const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables), 
      m_numDimensions((variables.size() - 1)/2), 
      m_target(getCurrentDomain(variables[0])) {
    checkError(variables.size() % 2 == 1, "Invalid Arg Count: " << variables.size());
  }

  CalcDistanceConstraint::~CalcDistanceConstraint() {}
  
  void CalcDistanceConstraint::handleExecute() {
    double sum = 0.0;
    for (unsigned int i = 0; i< m_numDimensions; i++){
      AbstractDomain& p = getCurrentDomain(getScope()[1 + i]);
      AbstractDomain& q = getCurrentDomain(getScope()[1 + i + m_numDimensions]);
      if(!p.isSingleton() || !q.isSingleton())
	return;

      sum+= pow(p.getSingletonValue() - q.getSingletonValue(), 2);
    }

    double result = sqrt(sum);
    m_target.set(result);

    debugMsg("CalcDistanceConstraint:handleExecute", "After:" << toString());
  }
}
