#ifndef _CALCINTERPOLATEDENDEFFECTORPOSCONSTRAINTCONSTRAINT_H_
#define _CALCINTERPOLATEDENDEFFECTORPOSCONSTRAINTCONSTRAINT_H_

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
  
  class CalcInterpolatedEndEffectorPosConstraint : public Constraint {
    
  public:
    
    CalcInterpolatedEndEffectorPosConstraint(const LabelStr& name,
				const LabelStr& propagatorName,
				const ConstraintEngineId& constraintEngine,
				const std::vector<ConstrainedVariableId>& variables);
    
    ~CalcInterpolatedEndEffectorPosConstraint();
    
    void handleExecute();
    
  private:
    LoggerId m_logger;
    ROSNodeId m_rosNode;
    
    static const unsigned int ARG_COUNT = 37;
    static const unsigned int STEP_SIZE = 0;
    static const unsigned int CUR_ROT_1_1 = 1;
    static const unsigned int CUR_ROT_1_2 = 2;
    static const unsigned int CUR_ROT_1_3 = 3;
    static const unsigned int CUR_ROT_2_1 = 4;
    static const unsigned int CUR_ROT_2_2 = 5;
    static const unsigned int CUR_ROT_2_3 = 6;
    static const unsigned int CUR_ROT_3_1 = 7;
    static const unsigned int CUR_ROT_3_2 = 8;
    static const unsigned int CUR_ROT_3_3 = 9;
    static const unsigned int CUR_TRANS_1 = 10;
    static const unsigned int CUR_TRANS_2 = 11;
    static const unsigned int CUR_TRANS_3 = 12;
    static const unsigned int DES_ROT_1_1 = 13;
    static const unsigned int DES_ROT_1_2 = 14;
    static const unsigned int DES_ROT_1_3 = 15;
    static const unsigned int DES_ROT_2_1 = 16;
    static const unsigned int DES_ROT_2_2 = 17;
    static const unsigned int DES_ROT_2_3 = 18;
    static const unsigned int DES_ROT_3_1 = 19;
    static const unsigned int DES_ROT_3_2 = 20;
    static const unsigned int DES_ROT_3_3 = 21;
    static const unsigned int DES_TRANS_1 = 22;
    static const unsigned int DES_TRANS_2 = 23;
    static const unsigned int DES_TRANS_3 = 24;
    static const unsigned int INTER_ROT_1_1 = 25;
    static const unsigned int INTER_ROT_1_2 = 26;
    static const unsigned int INTER_ROT_1_3 = 27;
    static const unsigned int INTER_ROT_2_1 = 28;
    static const unsigned int INTER_ROT_2_2 = 29;
    static const unsigned int INTER_ROT_2_3 = 30;
    static const unsigned int INTER_ROT_3_1 = 31;
    static const unsigned int INTER_ROT_3_2 = 32;
    static const unsigned int INTER_ROT_3_3 = 33;
    static const unsigned int INTER_TRANS_1 = 34;
    static const unsigned int INTER_TRANS_2 = 35;
    static const unsigned int INTER_TRANS_3 = 36;
  };
}

#endif
