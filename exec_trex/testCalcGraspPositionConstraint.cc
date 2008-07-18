#include "IntervalIntDomain.hh"

#include "TestSupport.hh"
#include "CalcGraspPositionConstraint.hh"
#include "CalcArmInverseKinematicsConstraint.hh"
#include "ROSNode.hh"

#include "libKDL/kdl_kinematics.h"

using namespace EUROPA;
using namespace TREX;

static const double JointMin = -M_PI;
static const double JointMax = M_PI;

int main(int argc, char** argv) {

  double obj_x = .60;
  double obj_y = -.5;
  double obj_z = 1.25;
  
  BoolDomain bothDom1;
  Variable<BoolDomain> bothVar1(ENGINE, bothDom1);

  BoolDomain bothDom2;
  Variable<BoolDomain> bothVar2(ENGINE, bothDom2);

  Variable<IntervalDomain> objx_var(ENGINE,IntervalDomain(obj_x));
  Variable<IntervalDomain> objy_var(ENGINE,IntervalDomain(obj_y));
  Variable<IntervalDomain> objz_var(ENGINE,IntervalDomain(obj_z));

  Variable<IntervalDomain> rot_frame_1_1(ENGINE,IntervalDomain(-M_PI,M_PI));
  Variable<IntervalDomain> rot_frame_1_2(ENGINE,IntervalDomain(-M_PI,M_PI));
  Variable<IntervalDomain> rot_frame_1_3(ENGINE,IntervalDomain(-M_PI,M_PI));
  Variable<IntervalDomain> rot_frame_2_1(ENGINE,IntervalDomain(-M_PI,M_PI));
  Variable<IntervalDomain> rot_frame_2_2(ENGINE,IntervalDomain(-M_PI,M_PI));
  Variable<IntervalDomain> rot_frame_2_3(ENGINE,IntervalDomain(-M_PI,M_PI));
  Variable<IntervalDomain> rot_frame_3_1(ENGINE,IntervalDomain(-M_PI,M_PI));
  Variable<IntervalDomain> rot_frame_3_2(ENGINE,IntervalDomain(-M_PI,M_PI));
  Variable<IntervalDomain> rot_frame_3_3(ENGINE,IntervalDomain(-M_PI,M_PI));

  Variable<IntervalDomain> trans_frame_1(ENGINE,IntervalDomain(-100.0, 100.0));
  Variable<IntervalDomain> trans_frame_2(ENGINE,IntervalDomain(-100.0, 100.0));
  Variable<IntervalDomain> trans_frame_3(ENGINE,IntervalDomain(-100.0, 100.0));

  Variable<IntervalDomain> joint_1(ENGINE,IntervalDomain(JointMin,JointMax));
  Variable<IntervalDomain> joint_2(ENGINE,IntervalDomain(JointMin,JointMax));
  Variable<IntervalDomain> joint_3(ENGINE,IntervalDomain(JointMin,JointMax));
  Variable<IntervalDomain> joint_4(ENGINE,IntervalDomain(JointMin,JointMax));
  Variable<IntervalDomain> joint_5(ENGINE,IntervalDomain(JointMin,JointMax));
  Variable<IntervalDomain> joint_6(ENGINE,IntervalDomain(JointMin,JointMax));
  Variable<IntervalDomain> joint_7(ENGINE,IntervalDomain(JointMin,JointMax));

  ROSNode::request();

  sleep(1);

  std::vector<ConstrainedVariableId> scope;
  {
    scope.push_back(bothVar1.getId());
    scope.push_back(objx_var.getId());
    scope.push_back(objy_var.getId());
    scope.push_back(objz_var.getId());
    scope.push_back(rot_frame_1_1.getId());
    scope.push_back(rot_frame_1_2.getId());
    scope.push_back(rot_frame_1_3.getId());
    scope.push_back(rot_frame_2_1.getId());
    scope.push_back(rot_frame_2_2.getId());
    scope.push_back(rot_frame_2_3.getId());
    scope.push_back(rot_frame_3_1.getId());
    scope.push_back(rot_frame_3_2.getId());
    scope.push_back(rot_frame_3_3.getId());
    scope.push_back(trans_frame_1.getId());
    scope.push_back(trans_frame_2.getId());
    scope.push_back(trans_frame_3.getId());

    CalcGraspPositionConstraint comgrasp(LabelStr("CalcGraspPosition"),
					 LabelStr("Default"),
					 ENGINE, 
					 scope);
    ENGINE->propagate();

    assertTrue(bothVar1.getDerivedDomain().isSingleton());
    assertTrue(bothVar1.getDerivedDomain().getSingletonValue() == true);

    assertTrue(trans_frame_1.getDerivedDomain().isSingleton());
    assertTrue(trans_frame_2.getDerivedDomain().isSingleton());
    assertTrue(trans_frame_3.getDerivedDomain().isSingleton());

    std::cout << "Trans X " << trans_frame_1.getDerivedDomain().getSingletonValue() << std::endl;
    std::cout << "Trans Y " << trans_frame_2.getDerivedDomain().getSingletonValue() << std::endl;
    std::cout << "Trans Z " << trans_frame_3.getDerivedDomain().getSingletonValue() << std::endl; 

    scope.clear();

    scope.push_back(bothVar2.getId());
    scope.push_back(rot_frame_1_1.getId());
    scope.push_back(rot_frame_1_2.getId());
    scope.push_back(rot_frame_1_3.getId());
    scope.push_back(rot_frame_2_1.getId());
    scope.push_back(rot_frame_2_2.getId());
    scope.push_back(rot_frame_2_3.getId());
    scope.push_back(rot_frame_3_1.getId());
    scope.push_back(rot_frame_3_2.getId());
    scope.push_back(rot_frame_3_3.getId());
    scope.push_back(trans_frame_1.getId());
    scope.push_back(trans_frame_2.getId());
    scope.push_back(trans_frame_3.getId());
    scope.push_back(joint_1.getId());
    scope.push_back(joint_2.getId());
    scope.push_back(joint_3.getId());
    scope.push_back(joint_4.getId());
    scope.push_back(joint_5.getId());
    scope.push_back(joint_6.getId());
    scope.push_back(joint_7.getId());

    CalcArmInverseKinematicsConstraint comarminv(LabelStr("CalcArmInverseKinematics"),
						 LabelStr("Default"),
						 ENGINE, 
						 scope);
    ENGINE->propagate();

    //assertTrue(bothVar.getDerivedDomain().isSingleton());
    //assertTrue(bothVar.getDerivedDomain().getSingletonValue() == true);

    std::cout << joint_1.getDerivedDomain() << std::endl;

    assertTrue(joint_1.getDerivedDomain().isSingleton());
    assertTrue(joint_2.getDerivedDomain().isSingleton());
    assertTrue(joint_3.getDerivedDomain().isSingleton());
    assertTrue(joint_4.getDerivedDomain().isSingleton());
    assertTrue(joint_5.getDerivedDomain().isSingleton());
    assertTrue(joint_6.getDerivedDomain().isSingleton());
    assertTrue(joint_7.getDerivedDomain().isSingleton());

    std_msgs::PR2Arm armGoal;
    armGoal.turretAngle = joint_1.getDerivedDomain().getSingletonValue(); 
    armGoal.shoulderLiftAngle = joint_2.getDerivedDomain().getSingletonValue(); 
    armGoal.upperarmRollAngle = joint_3.getDerivedDomain().getSingletonValue();
    armGoal.elbowAngle = joint_4.getDerivedDomain().getSingletonValue();
    armGoal.forearmRollAngle = joint_5.getDerivedDomain().getSingletonValue();
    armGoal.wristPitchAngle = joint_6.getDerivedDomain().getSingletonValue();
    armGoal.wristRollAngle = joint_7.getDerivedDomain().getSingletonValue();
    armGoal.gripperForceCmd = 10.0; 
    armGoal.gripperGapCmd = 0.1;
    
  
    ROSNode::request()->publish("right_pr2arm_set_position",armGoal);
    
    sleep(1);

    //assertTrue(joint_1.getDerivedDomain().getSingletonValue() == des_joint_1);
    //assertTrue(joint_2.getDerivedDomain().getSingletonValue() == des_joint_2);
    //assertTrue(joint_3.getDerivedDomain().getSingletonValue() == des_joint_3);
    //assertTrue(joint_4.getDerivedDomain().getSingletonValue() == des_joint_4);
    //assertTrue(joint_5.getDerivedDomain().getSingletonValue() == des_joint_5);
    //assertTrue(joint_6.getDerivedDomain().getSingletonValue() == des_joint_6);
    //assertTrue(joint_7.getDerivedDomain().getSingletonValue() == des_joint_7);
  }
  scope.clear();
}
