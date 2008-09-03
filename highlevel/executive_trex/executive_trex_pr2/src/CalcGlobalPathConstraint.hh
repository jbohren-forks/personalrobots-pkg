#ifndef _CALCGLOBALPATHCONSTRAINT_H_
#define _CALCGLOBALPATHCONSTRAINT_H_

#include "ConstraintEngineDefs.hh"
#include "Variable.hh"
#include "ConstrainedVariable.hh"
#include "ConstraintEngine.hh"
#include "Constraints.hh"
#include "Constraint.hh"
#include "IntervalDomain.hh"
#include "IntervalIntDomain.hh"
#include "BoolDomain.hh"
#include "Logger.hh"

using namespace EUROPA;
namespace TREX {
  
  class CalcGlobalPathConstraint : public Constraint {
    
  public:
    
    CalcGlobalPathConstraint(const LabelStr& name,
			     const LabelStr& propagatorName,
			     const ConstraintEngineId& constraintEngine,
			     const std::vector<ConstrainedVariableId>& variables);
    
    ~CalcGlobalPathConstraint();
    
    void handleExecute();
    
  private:
    LoggerId m_logger;
    
    static const unsigned int ARG_COUNT = 5;
    static const unsigned int PLAN_SUCC = 0;
    //static const unsigned int PLAN_DONE = 1;
    static const unsigned int LOCAL_X = 1;
    static const unsigned int LOCAL_Y = 2;
    static const unsigned int GOAL_X = 3;
    static const unsigned int GOAL_Y = 4;

  };
}

#endif
