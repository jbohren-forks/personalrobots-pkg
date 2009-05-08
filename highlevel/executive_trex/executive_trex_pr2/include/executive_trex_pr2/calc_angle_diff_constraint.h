#ifndef _CALCANGLEDIFFCONSTRAINT_H_
#define _CALCANGLEDIFFCONSTRAINT_H_

#include "ConstraintEngineDefs.hh"
#include "Variable.hh"
#include "ConstrainedVariable.hh"
#include "ConstraintEngine.hh"
#include "Constraints.hh"
#include "Constraint.hh"
#include "Domains.hh"
#include "executive_trex_pr2/logger.h"

using namespace EUROPA;
namespace TREX {
  
  class CalcAngleDiffConstraint : public Constraint {
    
  public:
    
    CalcAngleDiffConstraint(const LabelStr& name,
			    const LabelStr& propagatorName,
			    const ConstraintEngineId& constraintEngine,
			    const std::vector<ConstrainedVariableId>& variables);
    
    ~CalcAngleDiffConstraint();
    
    void handleExecute();
    
  private:
    LoggerId m_logger;
    
    static const unsigned int ARG_COUNT = 5;
    static const unsigned int ANGLE_DIFF = 0;
    static const unsigned int ANGLE_1 = 1;
    static const unsigned int ANGLE_2 = 2;

    AbstractDomain& m_diff;
    AbstractDomain& m_angle1;
    AbstractDomain& m_angle2;

  };
}

#endif
