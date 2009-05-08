#ifndef _CALCDISTANCECONSTRAINT_H_
#define _CALCDISTANCECONSTRAINT_H_

#include "ConstraintEngineDefs.hh"
#include "Variable.hh"
#include "ConstrainedVariable.hh"
#include "ConstraintEngine.hh"
#include "Constraints.hh"
#include "Constraint.hh"
#include "Domains.hh"

using namespace EUROPA;

namespace TREX {
  
  class CalcDistanceConstraint : public Constraint {
    
  public:
    
    CalcDistanceConstraint(const LabelStr& name,
			   const LabelStr& propagatorName,
			   const ConstraintEngineId& constraintEngine,
			   const std::vector<ConstrainedVariableId>& variables);
    
    ~CalcDistanceConstraint();
    
    void handleExecute();

  private:
    unsigned int m_numDimensions;
    AbstractDomain& m_target;
    
  };
}

#endif
