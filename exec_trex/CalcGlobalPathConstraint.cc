#include "CalcGlobalPathConstraint.hh"
#include "map_helpers.h"
#include "WavefrontPlanner.hh"

namespace EUROPA {

  CalcGlobalPathConstraint::CalcGlobalPathConstraint(const LabelStr& name,
						     const LabelStr& propagatorName,
						     const ConstraintEngineId& constraintEngine,
						    const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables)
  {
  }
  
  void CalcGlobalPathConstraint::handleExecute() {
    BoolDomain& boolDom = static_cast<BoolDomain&>(getCurrentDomain(m_variables[0]));
    check_error(!boolDom.isOpen());
    
    if(!m_variables[LOCAL_X]->lastDomain().isSingleton() ||
       !m_variables[LOCAL_Y]->lastDomain().isSingleton() ||
       !m_variables[GOAL_X]->lastDomain().isSingleton()  ||
       !m_variables[GOAL_Y]->lastDomain().isSingleton()) {
      return;
    }

    debugMsg("CalcGlobalPathConstraint:handleExecute", "Goal x " << m_variables[GOAL_X]->lastDomain().getSingletonValue() 
	      << " y " << m_variables[GOAL_Y]->lastDomain().getSingletonValue());



    //if(WavefrontPlanner::Instance()->NeedNewGlobalPlan(m_variables[GOAL_X]->lastDomain().getSingletonValue(),
    //					       m_variables[GOAL_Y]->lastDomain().getSingletonValue())) {
    if(WavefrontPlanner::Instance()->GenerateGlobalPlan(m_variables[LOCAL_X]->lastDomain().getSingletonValue(),
							m_variables[LOCAL_Y]->lastDomain().getSingletonValue(),
							m_variables[GOAL_X]->lastDomain().getSingletonValue(),  
							m_variables[GOAL_Y]->lastDomain().getSingletonValue()) < 0) {
      boolDom.remove(true);
      check_error(boolDom.isSingleton());
      return;
    }
    boolDom.remove(false);
    return;
  }
}
