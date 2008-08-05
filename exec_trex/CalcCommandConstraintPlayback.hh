#ifndef _CALCCOMMANDCONSTRAINTPLAYBACK_H_
#define _CALCCOMMANDCONSTRAINTPLAYBACK_H_

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
  
  class CalcCommandConstraintPlayback : public Constraint {
    
  public:
    
    CalcCommandConstraintPlayback(const LabelStr& name,
			  const LabelStr& propagatorName,
			  const ConstraintEngineId& constraintEngine,
			  const std::vector<ConstrainedVariableId>& variables);
    
    ~CalcCommandConstraintPlayback();
    
    void handleExecute();
    
  private:
    PlaybackId m_playback;
    
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
