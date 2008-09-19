#include "IntervalIntDomain.hh"

#include "TestSupport.hh"
#include "CalcArmInverseKinematicsConstraint.hh"


#include "libKDL/kdl_kinematics.h"

using namespace EUROPA;
using namespace TREX;

static const double JointMin = -M_PI/2;
static const double JointMax = M_PI/2;

int main(int argc, char** argv) {
  
  KDL::Rotation r = KDL::Rotation::RotZ(DTOR(0));
  KDL::Vector v(.568,.01,0.06);
  
  //double rot_1_1 = 0.889014;
  //double rot_1_2 = -0.450621;
  //double rot_1_3 = -0.0812059;
  //double rot_2_1 = 0.390995;
  //double rot_2_2 = 0.839411;
  //double rot_2_3 = -0.377507;
  //double rot_3_1 =  0.238278;
  //double rot_3_2 = 0.303858;
  //double rot_3_3 = 0.922439;
  //double trans_1 = 0.547702;
  //double trans_2 = 0.0825618;
  //double trans_3 = 0.54147;

  double rot_1_1 = r.data[0];
  double rot_1_2 = r.data[1];
  double rot_1_3 = r.data[2];
  double rot_2_1 = r.data[3];
  double rot_2_2 = r.data[4];
  double rot_2_3 = r.data[5];
  double rot_3_1 = r.data[6];
  double rot_3_2 = r.data[7];
  double rot_3_3 = r.data[8];
  double trans_1 = v.data[0];
  double trans_2 = v.data[1];
  double trans_3 = v.data[1];


  // Taken from pr2_kin_test
  // [[    0.889014,   -0.450621,  -0.0812059;
  //      0.390995,    0.839411,   -0.377507;
  //      0.238278,    0.303858,    0.922439]
  // [    0.547702,   0.0825618,     0.54147]]

  //values taken from those computed by pr2_kin_test
  //IK result:[ -0.00855573   -0.867175     1.67927    0.282756    -1.36737    0.610218    0.331123]

  double des_joint_1 = -0.00855573;
  double des_joint_2 = -0.867175;
  double des_joint_3 = 1.67927;
  double des_joint_4 = 0.282756;
  double des_joint_5 = -1.36737;
  double des_joint_6 = 0.610218;
  double des_joint_7 = 0.331123;

  BoolDomain bothDom;
  Variable<BoolDomain> bothVar(ENGINE, bothDom);

  Variable<IntervalDomain> rot_frame_1_1(ENGINE,IntervalDomain(rot_1_1));
  Variable<IntervalDomain> rot_frame_1_2(ENGINE,IntervalDomain(rot_1_2));
  Variable<IntervalDomain> rot_frame_1_3(ENGINE,IntervalDomain(rot_1_3));
  Variable<IntervalDomain> rot_frame_2_1(ENGINE,IntervalDomain(rot_2_1));
  Variable<IntervalDomain> rot_frame_2_2(ENGINE,IntervalDomain(rot_2_2));
  Variable<IntervalDomain> rot_frame_2_3(ENGINE,IntervalDomain(rot_2_3));
  Variable<IntervalDomain> rot_frame_3_1(ENGINE,IntervalDomain(rot_3_1));
  Variable<IntervalDomain> rot_frame_3_2(ENGINE,IntervalDomain(rot_3_2));
  Variable<IntervalDomain> rot_frame_3_3(ENGINE,IntervalDomain(rot_3_3));

  Variable<IntervalDomain> trans_frame_1(ENGINE,IntervalDomain(trans_1));
  Variable<IntervalDomain> trans_frame_2(ENGINE,IntervalDomain(trans_2));
  Variable<IntervalDomain> trans_frame_3(ENGINE,IntervalDomain(trans_3));

  Variable<IntervalDomain> joint_1(ENGINE,IntervalDomain(JointMin,JointMax));
  Variable<IntervalDomain> joint_2(ENGINE,IntervalDomain(JointMin,JointMax));
  Variable<IntervalDomain> joint_3(ENGINE,IntervalDomain(JointMin,JointMax));
  Variable<IntervalDomain> joint_4(ENGINE,IntervalDomain(JointMin,JointMax));
  Variable<IntervalDomain> joint_5(ENGINE,IntervalDomain(JointMin,JointMax));
  Variable<IntervalDomain> joint_6(ENGINE,IntervalDomain(JointMin,JointMax));
  Variable<IntervalDomain> joint_7(ENGINE,IntervalDomain(JointMin,JointMax));

  std::vector<ConstrainedVariableId> scope;
  {
    scope.push_back(bothVar.getId());
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

    assertTrue(bothVar.getDerivedDomain().isSingleton());
    assertTrue(bothVar.getDerivedDomain().getSingletonValue() == true);

    assertTrue(joint_1.getDerivedDomain().isSingleton());
    assertTrue(joint_2.getDerivedDomain().isSingleton());
    assertTrue(joint_3.getDerivedDomain().isSingleton());
    assertTrue(joint_4.getDerivedDomain().isSingleton());
    assertTrue(joint_5.getDerivedDomain().isSingleton());
    assertTrue(joint_6.getDerivedDomain().isSingleton());
    assertTrue(joint_7.getDerivedDomain().isSingleton());
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
