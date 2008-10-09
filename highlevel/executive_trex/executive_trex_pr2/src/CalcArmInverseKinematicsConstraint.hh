#ifndef _CALCARMINVERSEKINEMATICSCONSTRAINT_H_
#define _CALCARMINVERSEKINEMATICSCONSTRAINT_H_

#include "ROSNode.hh"
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
  
  class CalcArmInverseKinematicsConstraint : public Constraint {
    
  public:
    
    CalcArmInverseKinematicsConstraint(const LabelStr& name,
				       const LabelStr& propagatorName,
				       const ConstraintEngineId& constraintEngine,
				       const std::vector<ConstrainedVariableId>& variables);
    
    ~CalcArmInverseKinematicsConstraint();
    
    void handleExecute();
    
  private:
    LoggerId m_logger;
    ROSNodeId m_rosNode;    
    static const unsigned int ARG_COUNT = 20;
    static const unsigned int PLAN_SUCC = 0;
    static const unsigned int ROT_FRAME_1_1 = 1;
    static const unsigned int ROT_FRAME_1_2 = 2;
    static const unsigned int ROT_FRAME_1_3 = 3;
    static const unsigned int ROT_FRAME_2_1 = 4;
    static const unsigned int ROT_FRAME_2_2 = 5;
    static const unsigned int ROT_FRAME_2_3 = 6;
    static const unsigned int ROT_FRAME_3_1 = 7;
    static const unsigned int ROT_FRAME_3_2 = 8;
    static const unsigned int ROT_FRAME_3_3 = 9;
    static const unsigned int TRANS_FRAME_1 = 10;
    static const unsigned int TRANS_FRAME_2 = 11;
    static const unsigned int TRANS_FRAME_3 = 12;
    static const unsigned int JOINT_0 = 13;
    static const unsigned int JOINT_1 = 14;
    static const unsigned int JOINT_2 = 15;
    static const unsigned int JOINT_3 = 16;
    static const unsigned int JOINT_4 = 17;
    static const unsigned int JOINT_5 = 18;
    static const unsigned int JOINT_6 = 19;
  };
}

#endif
