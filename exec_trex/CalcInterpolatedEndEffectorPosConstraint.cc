#include "CalcInterpolatedEndEffectorPosConstraint.hh"
#include "Logger.hh"

//kinematics includes
#include <libKDL/kdl_kinematics.h>
#include <kdl/rotational_interpolation_sa.hpp>

using namespace KDL;
using namespace PR2;
using namespace std;

namespace TREX {

  CalcInterpolatedEndEffectorPosConstraint::CalcInterpolatedEndEffectorPosConstraint(const LabelStr& name,
									 const LabelStr& propagatorName,
									 const ConstraintEngineId& constraintEngine,
									 const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables) {
    m_logger = TREX::Logger::request();
  }

  CalcInterpolatedEndEffectorPosConstraint::~CalcInterpolatedEndEffectorPosConstraint() {
    m_logger->release();
  }
    
  void CalcInterpolatedEndEffectorPosConstraint::handleExecute() {
    static unsigned int sl_cycles = 0;
    sl_cycles++;
    
    //desired end effector frame arguments
    if(!m_variables[STEP_SIZE]->lastDomain().isSingleton() ||
       !m_variables[CUR_ROT_1_1]->lastDomain().isSingleton() ||
       !m_variables[CUR_ROT_1_2]->lastDomain().isSingleton() ||
       !m_variables[CUR_ROT_1_3]->lastDomain().isSingleton() ||
       !m_variables[CUR_ROT_2_1]->lastDomain().isSingleton() ||
       !m_variables[CUR_ROT_2_2]->lastDomain().isSingleton() ||
       !m_variables[CUR_ROT_2_3]->lastDomain().isSingleton() ||
       !m_variables[CUR_ROT_3_1]->lastDomain().isSingleton() ||
       !m_variables[CUR_ROT_3_2]->lastDomain().isSingleton() ||
       !m_variables[CUR_ROT_3_3]->lastDomain().isSingleton() ||
       !m_variables[CUR_TRANS_1]->lastDomain().isSingleton() ||
       !m_variables[CUR_TRANS_2]->lastDomain().isSingleton() ||
       !m_variables[CUR_TRANS_3]->lastDomain().isSingleton() ||
       !m_variables[DES_ROT_1_1]->lastDomain().isSingleton() ||
       !m_variables[DES_ROT_1_2]->lastDomain().isSingleton() ||
       !m_variables[DES_ROT_1_3]->lastDomain().isSingleton() ||
       !m_variables[DES_ROT_2_1]->lastDomain().isSingleton() ||
       !m_variables[DES_ROT_2_2]->lastDomain().isSingleton() ||
       !m_variables[DES_ROT_2_3]->lastDomain().isSingleton() ||
       !m_variables[DES_ROT_3_1]->lastDomain().isSingleton() ||
       !m_variables[DES_ROT_3_2]->lastDomain().isSingleton() ||
       !m_variables[DES_ROT_3_3]->lastDomain().isSingleton() ||
       !m_variables[DES_TRANS_1]->lastDomain().isSingleton() ||
       !m_variables[DES_TRANS_2]->lastDomain().isSingleton() ||
       !m_variables[DES_TRANS_3]->lastDomain().isSingleton()) {
      return;
    }

    //if we've run once no need to run again    
    if(m_variables[INTER_ROT_1_1]->lastDomain().isSingleton() &&
       m_variables[INTER_ROT_1_2]->lastDomain().isSingleton() &&
       m_variables[INTER_ROT_1_3]->lastDomain().isSingleton() &&
       m_variables[INTER_ROT_2_1]->lastDomain().isSingleton() &&
       m_variables[INTER_ROT_2_2]->lastDomain().isSingleton() &&
       m_variables[INTER_ROT_2_3]->lastDomain().isSingleton() &&
       m_variables[INTER_ROT_3_1]->lastDomain().isSingleton() &&
       m_variables[INTER_ROT_3_2]->lastDomain().isSingleton() &&
       m_variables[INTER_ROT_3_3]->lastDomain().isSingleton() &&
       m_variables[INTER_TRANS_1]->lastDomain().isSingleton() &&
       m_variables[INTER_TRANS_2]->lastDomain().isSingleton() &&
       m_variables[INTER_TRANS_3]->lastDomain().isSingleton()) {
      return;
    }

    //filling out the frames
    Frame curf, desf;
    
    //curf.p is the translation vector
    curf.p.data[0] = m_variables[CUR_TRANS_1]->lastDomain().getSingletonValue();
    curf.p.data[1] = m_variables[CUR_TRANS_2]->lastDomain().getSingletonValue();
    curf.p.data[2] = m_variables[CUR_TRANS_3]->lastDomain().getSingletonValue();

    //curf.M is the rotation matrix
    curf.M.data[0] = m_variables[CUR_ROT_1_1]->lastDomain().getSingletonValue();
    curf.M.data[1] = m_variables[CUR_ROT_1_2]->lastDomain().getSingletonValue();
    curf.M.data[2] = m_variables[CUR_ROT_1_3]->lastDomain().getSingletonValue();
    curf.M.data[3] = m_variables[CUR_ROT_2_1]->lastDomain().getSingletonValue();
    curf.M.data[4] = m_variables[CUR_ROT_2_2]->lastDomain().getSingletonValue();
    curf.M.data[5] = m_variables[CUR_ROT_2_3]->lastDomain().getSingletonValue();
    curf.M.data[6] = m_variables[CUR_ROT_3_1]->lastDomain().getSingletonValue();
    curf.M.data[7] = m_variables[CUR_ROT_3_2]->lastDomain().getSingletonValue();
    curf.M.data[8] = m_variables[CUR_ROT_3_3]->lastDomain().getSingletonValue();

    //desfp is the translation vector
    desf.p.data[0] = m_variables[DES_TRANS_1]->lastDomain().getSingletonValue();
    desf.p.data[1] = m_variables[DES_TRANS_2]->lastDomain().getSingletonValue();
    desf.p.data[2] = m_variables[DES_TRANS_3]->lastDomain().getSingletonValue();

    //desfM is the rotation matrix
    desf.M.data[0] = m_variables[DES_ROT_1_1]->lastDomain().getSingletonValue();
    desf.M.data[1] = m_variables[DES_ROT_1_2]->lastDomain().getSingletonValue();
    desf.M.data[2] = m_variables[DES_ROT_1_3]->lastDomain().getSingletonValue();
    desf.M.data[3] = m_variables[DES_ROT_2_1]->lastDomain().getSingletonValue();
    desf.M.data[4] = m_variables[DES_ROT_2_2]->lastDomain().getSingletonValue();
    desf.M.data[5] = m_variables[DES_ROT_2_3]->lastDomain().getSingletonValue();
    desf.M.data[6] = m_variables[DES_ROT_3_1]->lastDomain().getSingletonValue();
    desf.M.data[7] = m_variables[DES_ROT_3_2]->lastDomain().getSingletonValue();
    desf.M.data[8] = m_variables[DES_ROT_3_3]->lastDomain().getSingletonValue();
 
    Vector move_vec = desf.p-curf.p;
    double dist = move_vec.Norm();
    
    move_vec = move_vec/dist;
    
    double step_size = m_variables[STEP_SIZE]->lastDomain().getSingletonValue();
    
    Frame retframe;
    std::cout << "Norm is " << dist << std::endl;
    std::cout << "Cur trans " << curf.p.data[0] << " " << curf.p.data[1] << " " << curf.p.data[2] << " " << std::endl;
    std::cout << "Des trans " << desf.p.data[0] << " " << desf.p.data[1] << " " << desf.p.data[2] << " " << std::endl;

    //figure out if we're within the step size of the destination
    if(dist < step_size) {
      retframe.p = desf.p;
      retframe.M = desf.M;
    } else {

      RotationalInterpolation_SingleAxis rotInterpolater;
      rotInterpolater.SetStartEnd(curf.M, desf.M);
      double total_angle = rotInterpolater.Angle();
      //	printf("Angle: %f\n", rotInterpolater.Angle());
      
      Vector target;
      int nSteps = (int)(dist/step_size);
      double angle_step = total_angle/nSteps;

      retframe.p = curf.p+move_vec*step_size;
      retframe.M = rotInterpolater.Pos(angle_step);
    }

    //filling out return variables
    getCurrentDomain(m_variables[INTER_ROT_1_1]).set(retframe.M.data[0]);
    getCurrentDomain(m_variables[INTER_ROT_1_2]).set(retframe.M.data[1]);
    getCurrentDomain(m_variables[INTER_ROT_1_3]).set(retframe.M.data[2]);
    getCurrentDomain(m_variables[INTER_ROT_2_1]).set(retframe.M.data[3]);
    getCurrentDomain(m_variables[INTER_ROT_2_2]).set(retframe.M.data[4]);
    getCurrentDomain(m_variables[INTER_ROT_2_3]).set(retframe.M.data[5]);
    getCurrentDomain(m_variables[INTER_ROT_3_1]).set(retframe.M.data[6]);
    getCurrentDomain(m_variables[INTER_ROT_3_2]).set(retframe.M.data[7]);
    getCurrentDomain(m_variables[INTER_ROT_3_3]).set(retframe.M.data[8]);

    getCurrentDomain(m_variables[INTER_TRANS_1]).set(retframe.p.data[0]);
    getCurrentDomain(m_variables[INTER_TRANS_2]).set(retframe.p.data[1]);
    getCurrentDomain(m_variables[INTER_TRANS_3]).set(retframe.p.data[2]);

    //FILE* log = m_logger->getFile();
    //if (log) {
    //  fprintf(log, "\t<CalcGlobalPathConstraint time=\"%u\" value=\"%d\"/>\n", sl_cycles, res);
    //}
  }
  





}
