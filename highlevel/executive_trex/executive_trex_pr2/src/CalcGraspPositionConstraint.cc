#include "CalcGraspPositionConstraint.hh"
#include "Logger.hh"

//kinematics includes
#include <robot_kinematics/robot_kinematics.h>

//for transform
#include <rosTF/rosTF.h>

using namespace KDL;
using namespace PR2;
using namespace std;

#define DTOR(d) ((d)*M_PI/180)

namespace TREX {

  CalcGraspPositionConstraint::CalcGraspPositionConstraint(const LabelStr& name,
							   const LabelStr& propagatorName,
							   const ConstraintEngineId& constraintEngine,
							   const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables){
    m_logger = TREX::Logger::request();
    m_rosNode = ROSNode::request();
  }

  CalcGraspPositionConstraint::~CalcGraspPositionConstraint() {
    m_rosNode->release();
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

    ROSNodeId rni = ROSNode::request();

    //getting shoulder origin in world coordinates
    libTF::TFPose aPose;
    aPose.x = 0;
    aPose.y = 0;
    aPose.z = 0;
    aPose.roll = 0;
    aPose.pitch = 0;
    aPose.yaw = 0;
    aPose.time = 0;
    aPose.frame = "FRAMEID_ARM_R_SHOULDER";

    libTF::TFPose inShoulderFrame = rni->tf.transformPose("FRAMEID_ODOM", aPose);

    //std::cout << "In shoulder frame in base x " << inShoulderFrame.x << std::endl;
    //std::cout << "In shoulder frame in base y " << inShoulderFrame.y << std::endl;
    //std::cout << "In shoulder frame in base z " << inShoulderFrame.z << std::endl;


    //now we have the shoulder translation in world coordinates
    double objPosX = m_variables[OBJECT_POS_X]->lastDomain().getSingletonValue()-inShoulderFrame.x;
    double objPosY = m_variables[OBJECT_POS_Y]->lastDomain().getSingletonValue()-inShoulderFrame.y;
    double objPosZ = m_variables[OBJECT_POS_Z]->lastDomain().getSingletonValue()-inShoulderFrame.z;

    //take object x,y,z in world and convert to shoulder coordinates
    libTF::TFPose bPose;
    bPose.x = 0;
    bPose.y = 0;
    bPose.z = 0;
    bPose.roll = 0;
    bPose.pitch = 0;
    bPose.yaw = 0;
    bPose.time = 0;
    bPose.frame = "FRAMEID_ARM_R_HAND";
    
    libTF::TFPose inHandFrame = rni->tf.transformPose("FRAMEID_ODOM", bPose);
 
    //std::cout << "In hand frame in base x " << inHandFrame.x << std::endl;
    //std::cout << "In hand frame in base y " << inHandFrame.y << std::endl;
    //std::cout << "In hand frame in base z " << inHandFrame.z << std::endl;
   
    Rotation r = Rotation::RotZ(DTOR(0));
    Vector v(objPosX,objPosY,objPosZ);
    
    std::cout << "In shoulder frame x " << objPosX << std::endl;
    std::cout << "In shoulder frame y " << objPosY << std::endl;
    std::cout << "In shoulder frame z " << objPosZ << std::endl;

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

    rni->release();
  }

}
