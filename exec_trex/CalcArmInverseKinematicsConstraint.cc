#include "CalcArmInverseKinematicsConstraint.hh"
#include "Logger.hh"

//kinematics includes
#include "libKDL/kdl_kinematics.h"

using namespace KDL;
using namespace PR2;
using namespace std;

namespace TREX {

  CalcArmInverseKinematicsConstraint::CalcArmInverseKinematicsConstraint(const LabelStr& name,
									 const LabelStr& propagatorName,
									 const ConstraintEngineId& constraintEngine,
									 const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables) {
    m_logger = TREX::Logger::request();
  }

  CalcArmInverseKinematicsConstraint::~CalcArmInverseKinematicsConstraint() {
    m_logger->release();
  }
    
  void CalcArmInverseKinematicsConstraint::handleExecute() {
    static unsigned int sl_cycles = 0;
    sl_cycles++;
    
    BoolDomain& boolDom = static_cast<BoolDomain&>(getCurrentDomain(m_variables[0]));
    check_error(!boolDom.isOpen());

    //desired end effector frame arguments
    if(!m_variables[ROT_FRAME_1_1]->lastDomain().isSingleton() ||
       !m_variables[ROT_FRAME_1_2]->lastDomain().isSingleton() ||
       !m_variables[ROT_FRAME_1_3]->lastDomain().isSingleton() ||
       !m_variables[ROT_FRAME_2_1]->lastDomain().isSingleton() ||
       !m_variables[ROT_FRAME_2_2]->lastDomain().isSingleton() ||
       !m_variables[ROT_FRAME_2_3]->lastDomain().isSingleton() ||
       !m_variables[ROT_FRAME_3_1]->lastDomain().isSingleton() ||
       !m_variables[ROT_FRAME_3_2]->lastDomain().isSingleton() ||
       !m_variables[ROT_FRAME_3_3]->lastDomain().isSingleton() ||
       !m_variables[TRANS_FRAME_1]->lastDomain().isSingleton() ||
       !m_variables[TRANS_FRAME_2]->lastDomain().isSingleton() ||
       !m_variables[TRANS_FRAME_3]->lastDomain().isSingleton()) {
      return;
    }

    //if we've run once no need to run again
    if(m_variables[JOINT_0]->lastDomain().isSingleton() &&
       m_variables[JOINT_1]->lastDomain().isSingleton() &&
       m_variables[JOINT_2]->lastDomain().isSingleton() &&
       m_variables[JOINT_3]->lastDomain().isSingleton() &&
       m_variables[JOINT_4]->lastDomain().isSingleton() &&
       m_variables[JOINT_5]->lastDomain().isSingleton() &&
       m_variables[JOINT_6]->lastDomain().isSingleton()) {
      return;
    }
       

    //filling out frame
    Frame f;
    
    //f.p is the translation vector
    f.p.data[0] = m_variables[TRANS_FRAME_1]->lastDomain().getSingletonValue();
    f.p.data[1] = m_variables[TRANS_FRAME_2]->lastDomain().getSingletonValue();
    f.p.data[2] = m_variables[TRANS_FRAME_3]->lastDomain().getSingletonValue();
    f.p.data[2] = m_variables[TRANS_FRAME_3]->lastDomain().getSingletonValue();

    //f.M is the rotation matrix -- hope we get order right
    f.M.data[0] = m_variables[ROT_FRAME_1_1]->lastDomain().getSingletonValue();
    f.M.data[1] = m_variables[ROT_FRAME_1_2]->lastDomain().getSingletonValue();
    f.M.data[2] = m_variables[ROT_FRAME_1_3]->lastDomain().getSingletonValue();
    f.M.data[3] = m_variables[ROT_FRAME_2_1]->lastDomain().getSingletonValue();
    f.M.data[4] = m_variables[ROT_FRAME_2_2]->lastDomain().getSingletonValue();
    f.M.data[5] = m_variables[ROT_FRAME_2_3]->lastDomain().getSingletonValue();
    f.M.data[6] = m_variables[ROT_FRAME_3_1]->lastDomain().getSingletonValue();
    f.M.data[7] = m_variables[ROT_FRAME_3_2]->lastDomain().getSingletonValue();
    f.M.data[8] = m_variables[ROT_FRAME_3_3]->lastDomain().getSingletonValue();

    //cout << "Frame " << f << endl;

    PR2_kinematics pr2_kin;
    
    //initial guesses - assume mostly straight out for now
    JntArray q_init = JntArray(pr2_kin.nJnts);
    q_init(0) = 0.1, q_init(1) = -1, q_init(2) = 0.3, q_init(3) = 0.3;
    q_init(4) = 0.2, q_init(5) = 0.5, q_init(6) = 0.0;

    bool res;
    JntArray q_out = JntArray(pr2_kin.nJnts);
    if (pr2_kin.IK(q_init, f, q_out) == true) {
      cout<<"IK result:"<<q_out<<endl;
      boolDom.set(true);
      res = true;
    } else {
      cout<<"Could not compute Inv Kin."<<endl;
      boolDom.set(false);
      res = false;
    }

    //filling out joint domains
    getCurrentDomain(m_variables[JOINT_0]).set(q_out(0));
    getCurrentDomain(m_variables[JOINT_1]).set(q_out(1));
    getCurrentDomain(m_variables[JOINT_2]).set(q_out(2));
    getCurrentDomain(m_variables[JOINT_3]).set(q_out(3));
    getCurrentDomain(m_variables[JOINT_4]).set(q_out(4));
    getCurrentDomain(m_variables[JOINT_5]).set(q_out(5));
    getCurrentDomain(m_variables[JOINT_6]).set(q_out(6));
    
    FILE* log = m_logger->getFile();
    if (log) {
      fprintf(log, "\t<CalcGlobalPathConstraint time=\"%u\" value=\"%d\"/>\n", sl_cycles, res);
    }
  }
  





}
