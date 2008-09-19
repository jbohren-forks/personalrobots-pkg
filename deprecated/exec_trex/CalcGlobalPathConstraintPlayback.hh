#ifndef _CALCGLOBALPATHCONSTRAINTPLAYBACK_H_
#define _CALCGLOBALPATHCONSTRAINTPLAYBACK_H_

#include "ConstraintEngineDefs.hh"
#include "Variable.hh"
#include "ConstrainedVariable.hh"
#include "ConstraintEngine.hh"
#include "Constraints.hh"
#include "Constraint.hh"
#include "IntervalDomain.hh"
#include "IntervalIntDomain.hh"
#include "BoolDomain.hh"
#include "Playback.hh"

using namespace EUROPA;
namespace TREX {
  
  class CalcGlobalPathConstraintPlayback : public Constraint {
    
  public:
    
    CalcGlobalPathConstraintPlayback(const LabelStr& name,
			     const LabelStr& propagatorName,
			     const ConstraintEngineId& constraintEngine,
			     const std::vector<ConstrainedVariableId>& variables);
    
    ~CalcGlobalPathConstraintPlayback();
    
    void handleExecute();
    
  private:
    PlaybackId m_playback;

  };
}

#endif
