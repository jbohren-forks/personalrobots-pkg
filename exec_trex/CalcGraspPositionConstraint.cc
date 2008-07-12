#include "CalcGraspPositionConstraint.hh"
#include "Logger.hh"

//kinematics includes
#include "libKDL/kdl_kinematics.h"

using namespace KDL;
using namespace PR2;
using namespace std;

namespace TREX {

  CalcGraspPositionConstraint::CalcGraspPositionConstraint(const LabelStr& name,
							   const LabelStr& propagatorName,
							   const ConstraintEngineId& constraintEngine,
							   const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables) {
    m_logger = TREX::Logger::request();
  }

  CalcGraspPositionConstraint::~CalcGraspPositionConstraint() {
    m_logger->release();
  }
  
  void CalcGraspPositionConstraint::handleExecute() {
    static unsigned int sl_cycles = 0;
    sl_cycles++;
    
    BoolDomain& boolDom = static_cast<BoolDomain&>(getCurrentDomain(m_variables[0]));
    check_error(!boolDom.isOpen());

    //desired end effector frame arguments
    if(!m_variables[OBJECT_POS_X]->lastDomain().isSingleton() ||
       !m_variables[OBJECT_POS_Y]->lastDomain().isSingleton() ||
       !m_variables[OBJECT_POS_Z]->lastDomain().isSingleton()) {
      return;
    }

    Rotation r = Rotation::RotZ(DTOR(0));
    Vector v(.75,-.25,-.15);

    //for now we hard-code
    getCurrentDomain(m_variables[ROT_FRAME_1_1]).set(r.data[0]);
    getCurrentDomain(m_variables[ROT_FRAME_1_2]).set(r.data[1]);
    getCurrentDomain(m_variables[ROT_FRAME_1_3]).set(r.data[2]);
    getCurrentDomain(m_variables[ROT_FRAME_2_1]).set(r.data[3]);
    getCurrentDomain(m_variables[ROT_FRAME_2_2]).set(r.data[4]);
    getCurrentDomain(m_variables[ROT_FRAME_2_3]).set(r.data[5]);
    getCurrentDomain(m_variables[ROT_FRAME_3_1]).set(r.data[6]);
    getCurrentDomain(m_variables[ROT_FRAME_3_2]).set(r.data[7]);
    getCurrentDomain(m_variables[ROT_FRAME_3_3]).set(r.data[8]);    
   
    getCurrentDomain(m_variables[TRANS_FRAME_1]).set(v.data[0]);
    getCurrentDomain(m_variables[TRANS_FRAME_2]).set(v.data[1]);
    getCurrentDomain(m_variables[TRANS_FRAME_3]).set(v.data[2]);
    
    boolDom.set(true);

    FILE* log = m_logger->getFile();
    if (log) {
      //fprintf(log, "\t<CalcGlobalPathConstraint time=\"%u\" value=\"%d\"/>\n", sl_cycles, res);
    }
  }

}
