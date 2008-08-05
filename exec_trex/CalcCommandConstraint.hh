#ifndef _CALCCOMMANDCONSTRAINT_H_
#define _CALCCOMMANDCONSTRAINT_H_

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
  
  class CalcCommandConstraint : public Constraint {
    
  public:
    
    CalcCommandConstraint(const LabelStr& name,
			  const LabelStr& propagatorName,
			  const ConstraintEngineId& constraintEngine,
			  const std::vector<ConstrainedVariableId>& variables);
    
    ~CalcCommandConstraint();
    
    void handleExecute();
    
  private:
    LoggerId m_logger;
    void logData(const unsigned int cycles, const bool bd, 
		 const double x_value, const double th_value);
    
    static const unsigned int ARG_COUNT = 9;
    static const unsigned int PLAN_SUCC = 0;
    //static const unsigned int PLAN_DONE = 1;
    static const unsigned int COM_VEL_X = 1;
    static const unsigned int COM_VEL_TH = 2;
    static const unsigned int LOCAL_X = 3;
    static const unsigned int LOCAL_Y = 4;
    static const unsigned int LOCAL_TH = 5;
    static const unsigned int GOAL_X = 6;
    static const unsigned int GOAL_Y = 7;
    static const unsigned int GOAL_TH = 8;  
    
  };
}

#endif
