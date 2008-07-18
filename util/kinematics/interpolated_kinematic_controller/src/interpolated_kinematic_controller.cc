#include <ros/node.h>
#include <rosthread/mutex.h>

#include <std_msgs/PR2Arm.h>
#include <rosgazebo/EndEffectorState.h>

#include <libpr2API/pr2API.h>

#include <kdl/rotational_interpolation_sa.hpp>

using namespace std_msgs;
using namespace PR2;
using namespace KDL;

static const double step_size = .05;

class InterpolatedKinematicController : public ros::node {
public:
  
  InterpolatedKinematicController(void) : ros::node("easy_kinematic_controller") {
    advertise<rosgazebo::EndEffectorState>("cmd_leftarm_cartesian");
    advertise<rosgazebo::EndEffectorState>("cmd_rightarm_cartesian");
    subscribe("right_pr2arm_set_end_effector", _rightEndEffectorGoal, &InterpolatedKinematicController::setRightEndEffector);	
    subscribe("left_pr2arm_set_end_effector", _leftEndEffectorGoal, &InterpolatedKinematicController::setLeftEndEffector);
    subscribe("left_pr2arm_pos",  leftArmPosMsg,  &InterpolatedKinematicController::currentLeftArmPos);
    subscribe("right_pr2arm_pos", rightArmPosMsg, &InterpolatedKinematicController::currentRightArmPos);
  }
  
  void setRightEndEffector(void) {
    KDL::Frame f;
    for(int i = 0; i < 9; i++) {
      f.M.data[i] = _rightEndEffectorGoal.rot[i];
    }
    for(int i = 0; i < 3; i++) {
      f.p.data[i] = _rightEndEffectorGoal.trans[i];
    }
    RunControlLoop(true, f, rightArmPosMsg);
  }

  void setLeftEndEffector(void) {
    KDL::Frame f;
    for(int i = 0; i < 9; i++) {
      f.M.data[i] = _leftEndEffectorGoal.rot[i];
    }
    for(int i = 0; i < 3; i++) {
      f.p.data[i] = _leftEndEffectorGoal.trans[i];
    }
    RunControlLoop(false, f, leftArmPosMsg);
  }

  
  void currentLeftArmPos(void) {
    // don't need to do anything -- we already have the data
  }
  
  void currentRightArmPos(void) {
    // don't need to do anything -- we already have the data
  }

  void publishFrame(bool isRightArm, const Frame& f) {
    rosgazebo::EndEffectorState efs;
    efs.set_rot_size(9);
    efs.set_trans_size(3);
    for(int i = 0; i < 9; i++) {
      efs.rot[i] = f.M.data[i];
    }
    for(int i = 0; i < 3; i++) {
      efs.trans[i] = f.p.data[i];
    }
    if(isRightArm) {
      publish("cmd_rightarm_cartesian",efs);
    } else {
      publish("cmd_leftarm_cartesian",efs);
    }
  }

  void RunControlLoop(bool isRightArm, const Frame& r, const std_msgs::PR2Arm& arm) {

    PR2_kinematics pr2_kin;
    JntArray q = JntArray(pr2_kin.nJnts);
    
    q(0) = arm.turretAngle;
    q(1) = arm.shoulderLiftAngle;
    q(2) = arm.upperarmRollAngle;
    q(3) = arm.elbowAngle;
    q(4) = arm.forearmRollAngle;
    q(5) = arm.wristPitchAngle;
    q(6) = arm.wristRollAngle;

    Frame f;
    pr2_kin.FK(q,f);
    Vector start = f.p;
    Vector move = r.p-start;
    double dist = move.Norm();
    move = move/dist;
    
    RotationalInterpolation_SingleAxis rotInterpolater;
    rotInterpolater.SetStartEnd(f.M, r.M);
    double total_angle = rotInterpolater.Angle();
    //	printf("Angle: %f\n", rotInterpolater.Angle());

    Vector target;
    int nSteps = (int)(dist/step_size);
    double angle_step = total_angle/nSteps;
    for(int i=0;i<nSteps;i++) {
      f.p = start+(i+1)*move*step_size;
      f.M = rotInterpolater.Pos(angle_step*(i+1));
      publishFrame(isRightArm, f);
      usleep(100000);
    }
    f.p = r.p;
    f.M = r.M;
    publishFrame(isRightArm, f);
  }

private:

  rosgazebo::EndEffectorState _leftEndEffectorGoal;
  rosgazebo::EndEffectorState _rightEndEffectorGoal;
  std_msgs::PR2Arm leftArmPosMsg, rightArmPosMsg;

};

int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    InterpolatedKinematicController easy;

    easy.spin();
        
    easy.shutdown();
    
    return 0;    
}
