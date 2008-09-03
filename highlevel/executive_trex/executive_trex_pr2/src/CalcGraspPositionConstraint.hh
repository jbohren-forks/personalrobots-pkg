#ifndef _CALCGRASPPOSITIONCONSTRAINT_H_
#define _CALCGRASPPOSITIONCONSTRAINT_H_

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
#include "ROSNode.hh"

using namespace EUROPA;
namespace TREX {
  
  class CalcGraspPositionConstraint : public Constraint {
    
  public:
    
    CalcGraspPositionConstraint(const LabelStr& name,
				const LabelStr& propagatorName,
				const ConstraintEngineId& constraintEngine,
				const std::vector<ConstrainedVariableId>& variables);
    
    ~CalcGraspPositionConstraint();
    
    void handleExecute();
    
  private:
    LoggerId m_logger;
    ROSNodeId m_rosNode;
    
    static const unsigned int ARG_COUNT = 16;
    static const unsigned int PLAN_SUCC = 0;
    static const unsigned int OBJECT_POS_X = 1;
    static const unsigned int OBJECT_POS_Y = 2;
    static const unsigned int OBJECT_POS_Z = 3;
    //eventually do base commands as well
    static const unsigned int ROT_FRAME_1_1 = 4;
    static const unsigned int ROT_FRAME_1_2 = 5;
    static const unsigned int ROT_FRAME_1_3 = 6;
    static const unsigned int ROT_FRAME_2_1 = 7;
    static const unsigned int ROT_FRAME_2_2 = 8;
    static const unsigned int ROT_FRAME_2_3 = 9;
    static const unsigned int ROT_FRAME_3_1 = 10;
    static const unsigned int ROT_FRAME_3_2 = 11;
    static const unsigned int ROT_FRAME_3_3 = 12;
    static const unsigned int TRANS_FRAME_1 = 13;
    static const unsigned int TRANS_FRAME_2 = 14;
    static const unsigned int TRANS_FRAME_3 = 15;
  };
}

#endif
