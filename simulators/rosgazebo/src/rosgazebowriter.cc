/*
 *  rosgazebo
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// gazebo
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <libpr2API/pr2API.h>
#include <libpr2HW/pr2HW.h>
#include <RosGazeboNode/ringbuffer.h>

#include <fstream>
#include <iostream>

// roscpp
#include <ros/node.h>
// roscpp - laser
#include <std_msgs/LaserScan.h>
// roscpp - laser image (point cloud)
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/Point3DFloat32.h>
#include <std_msgs/ChannelFloat32.h>
// roscpp - used for shutter message right now
#include <std_msgs/Empty.h>
// roscpp - base
#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/BaseVel.h>
// roscpp - arm
#include <std_msgs/PR2Arm.h>
//roscpp - trajectory message
#include <std_msgs/ArmTrajectory.h>
// roscpp - camera
#include <std_msgs/Image.h>

// for frame transforms
#include <rosTF/rosTF.h>

#include <time.h>
#include <signal.h>

#define SERVOTIME 2 //Sim time to wait for servoing to hanging position
// Our node
class GazeboNode : public ros::node
{
  private:
    // Messages that we'll send or receive
    std_msgs::BaseVel velMsg;
    std_msgs::LaserScan laserMsg;
    std_msgs::PointCloudFloat32 cloudMsg;
    std_msgs::PointCloudFloat32 full_cloudMsg;
    std_msgs::Empty shutterMsg;  // marks end of a cloud message
    std_msgs::RobotBase2DOdom odomMsg;

    // A mutex to lock access to fields that are used in message callbacks
    ros::thread::mutex lock;

    // for frame transforms, publish frame transforms
    rosTFServer tf;

    // time step calculation
    double lastTime, simTime;

    // smooth vx, vw commands
    double vxSmooth, vwSmooth;

    // used to generate Gaussian noise (for PCD)
    double GaussianKernel(double mu,double sigma);

    //Current desired velocity
    float currentVel[6];
	
     //Declare steam for writing to file
  ofstream datafile;
  public:	
    void PrintRightArmPosition(void);

    void PrintRightArm();

    //Commands arms to servo to hang downwards
    void LowerArms(void);

    //Command torques to be zero in right arm
    void DisableRightArm();

    //Command torques to be zero in left arm
    void DisableLeftArm();

    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    GazeboNode(int argc, char** argv, const char* fname);
    ~GazeboNode();

    // advertise / subscribe models
    int SubscribeModels();

    // Do one update of the simulator.  May pause if the next update time
    // has not yet arrived.
    void Update();

    //Sets desired cartesian velocities for the right arm
    void SetRightArmVelocities(float cartesianVels[]);
    //Sets desired cartesian velocities for the left arm
    void SetLeftArmVelocities(float cartesianVels[]);

    // Message callback for a std_msgs::BaseVel message, which set velocities.
    void cmdvelReceived();

    // Message callback for a std_msgs::PR2Arm message, which sets arm configuration.
    void cmd_leftarmconfigReceived();
    void cmd_rightarmconfigReceived();

    //Message callback for std_msgs::ArmTrajectory message, which defines a desired set point
    void cmd_trajectoryReceived();

    // laser range data
    float    ranges[GZ_LASER_MAX_RANGES];
    uint8_t  intensities[GZ_LASER_MAX_RANGES];

    // camera data
    std_msgs::Image img;
    
    // arm config messages
    std_msgs::PR2Arm leftarm;
    std_msgs::PR2Arm rightarm;

    //Trajectory messages
    std_msgs::ArmTrajectory trajectorymsg;

    // The main simulator object
    PR2::PR2Robot* myPR2;

    // for the point cloud data
    ringBuffer<std_msgs::Point3DFloat32> *cloud_pts;
    ringBuffer<float>                    *cloud_ch1;

    // keep count for full cloud
    int max_cloud_pts;

    // clean up on interrupt
    static void finalize(int);
};

void GazeboNode::cmd_trajectoryReceived(){
	printf("Trajectory received\n");
	for(int i = 0;i<6;i++){
		currentVel[i] = trajectorymsg.vel[i].data;
	}
}

//Command arms to be facing down
void GazeboNode::LowerArms(){

this->myPR2->SetArmControlMode(PR2::PR2_LEFT_ARM,PR2::PR2_JOINT_CONTROL);
this->myPR2->SetArmControlMode(PR2::PR2_RIGHT_ARM,PR2::PR2_JOINT_CONTROL);

this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_PAN           , 0,0);
this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_SHOULDER_PITCH, 90,0);
this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_SHOULDER_ROLL , 0,0);
this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_ELBOW_PITCH   , 0,0);
this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_ELBOW_ROLL    , 0,0);
this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_WRIST_PITCH   , 0,0);
this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_WRIST_ROLL    , 0,0);

this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_PAN           , 0,0);
this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_SHOULDER_PITCH, 90.0,0);
this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_SHOULDER_ROLL , 0,0);
this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_ELBOW_PITCH   , 0,0);
this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_ELBOW_ROLL    , 0,0);
this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_WRIST_PITCH   , 0,0);
this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_WRIST_ROLL    , 0,0);

}

//Print out right arm position
void GazeboNode::PrintRightArmPosition(){
	double position[7],cmd[7];
	//PR2_ERROR_CODE PR2Robot::GetArmJointSpeedCmd(PR2_MODEL_ID id, double speed[])

	//Write joint speed commands 
	this->myPR2->GetArmJointPositionCmd(PR2::PR2_RIGHT_ARM,position,cmd);
	
	cout<<"S1:"<<position[0]<<endl<<"S2:"<<position[1]<<endl<<"S3:"<<position[2]<<endl<<"S4:"<<position[3]<<endl<<"S5:"<<position[4]<<endl<<"S6:"<<position[5]<<endl<<"S7:"<<position[6]<<endl;	

	//Write actual joint positions
	this->myPR2->GetArmJointPositionActual(PR2::PR2_RIGHT_ARM,position,cmd);
	
	cout<<"AS1:"<<position[0]<<endl<<"AS2:"<<position[1]<<endl<<"AS3:"<<position[2]<<endl<<"AS4:"<<position[3]<<endl<<"AS5:"<<position[4]<<endl<<"AS6:"<<position[5]<<endl<<"AS7:"<<position[6]<<endl;	

}

//Sets desired cartesian velocities for the right arm
void GazeboNode::SetRightArmVelocities(float cartesianVels[]){

	KDL::ChainIkSolverVel_pinv* solver= this->myPR2->pr2_kin.ik_vel_solver;
	int numJnts = this->myPR2->pr2_kin.nJnts;

	//Define desired cartesian velocities
	KDL::Twist velocity;
	velocity(0) = cartesianVels[0]; //x y z
	velocity(1)= cartesianVels[1];
	velocity(2)= cartesianVels[2];
	velocity(3) = cartesianVels[3]; // yaw, pitch, roll
	velocity(4)= cartesianVels[4];
	velocity(5)= cartesianVels[5];

	double jointPosition[numJnts];
	double jointSpeed[numJnts];

	this->myPR2->GetArmJointPositionActual(PR2::PR2_RIGHT_ARM,jointPosition,jointSpeed);

	KDL::JntArray currentPosition = KDL::JntArray(numJnts);
	for (int ii = 0;ii<numJnts;ii++){
		currentPosition(ii) = jointPosition[ii];
	}
	KDL:: JntArray cmdVels = KDL::JntArray(numJnts);
	if(solver->KDL::ChainIkSolverVel_pinv::CartToJnt(currentPosition,velocity,cmdVels)>=0){
	
	//DEBUG
	//Uncomment here to command the resulting joint velocities

/*	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_PAN           , this->rightarm.turretAngle,       cmdVels(0));
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_SHOULDER_PITCH, this->rightarm.shoulderLiftAngle, cmdVels(1));
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_SHOULDER_ROLL , this->rightarm.upperarmRollAngle, cmdVels(2));
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_ELBOW_PITCH   , this->rightarm.elbowAngle,        cmdVels(3));
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_ELBOW_ROLL    , this->rightarm.forearmRollAngle,  cmdVels(4));
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_WRIST_PITCH   , this->rightarm.wristPitchAngle,   cmdVels(5));
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_WRIST_ROLL    , this->rightarm.wristRollAngle,    cmdVels(6));

	cout<<"J1:"<<cmdVels(0)<<endl<<"J2:"<<cmdVels(1)<<endl<<"J3:"<<cmdVels(2)<<endl<<"J4:"<<cmdVels(3)<<endl<<"J5:"<<cmdVels(4)		<<endl<<"J6:"<<cmdVels	(5)<<endl<<"J7:"<<cmdVels(6)<<endl;}

*/
	//Simpler code for commanding constant joint velocities
	//TUNE

	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_PAN           , 0,       0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_SHOULDER_PITCH, 0, 0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_SHOULDER_ROLL , 0, 0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_ELBOW_PITCH   ,0,        0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_ELBOW_ROLL    ,0,  0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_WRIST_PITCH   , 0,   0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_WRIST_ROLL    , 0,   0);

	double speed[7];
	//PR2_ERROR_CODE PR2Robot::GetArmJointSpeedCmd(PR2_MODEL_ID id, double speed[])

	//Write joint speed commands 
	this->myPR2->GetArmJointSpeedCmd(PR2::PR2_RIGHT_ARM,speed);
	
	cout<<"S1:"<<speed[0]<<endl<<"S2:"<<speed[1]<<endl<<"S3:"<<speed[2]<<endl<<"S4:"<<speed[3]<<endl<<"S5:"<<speed[4]<<endl<<"S6:"<<speed[5]<<endl<<"S7:"<<speed[6]<<endl;	

	//Write actual joint speeds
	//PR2_ERROR_CODE PR2Robot::GetArmJointSpeedCmd(PR2_MODEL_ID id, double speed[])
	this->myPR2->GetArmJointSpeedActual(PR2::PR2_RIGHT_ARM,speed);
	
	cout<<"AS1:"<<speed[0]<<endl<<"AS2:"<<speed[1]<<endl<<"AS3:"<<speed[2]<<endl<<"AS4:"<<speed[3]<<endl<<"AS5:"<<speed[4]<<endl<<"AS6:"<<speed[5]	<<endl<<"AS7:"<<speed[6]<<endl;	
	}
	//printf(" %f %f %f %f %f %f",cmdVels(0),cmdVels(1),cmdVels(2),cmdVels(3),cmdVels(4),cmdVels(5),cmdVels(6));
	else {cout<<"*****ERROR IN INVERSE VELOCITIES*****"<<endl;
	}
/*
	double t1,t2,t3,t4,t5,t6,t7;

	this->myPR2->hw.GetJointTorqueCmd(PR2::ARM_R_PAN, &t1);	
	this->myPR2->hw.GetJointTorqueCmd(PR2::ARM_R_SHOULDER_PITCH, &t2);	
	this->myPR2->hw.GetJointTorqueCmd(PR2::ARM_R_SHOULDER_ROLL, &t3);	
	this->myPR2->hw.GetJointTorqueCmd(PR2::ARM_R_ELBOW_PITCH, &t4);	
	this->myPR2->hw.GetJointTorqueCmd(PR2::ARM_R_ELBOW_ROLL, &t5);	
	this->myPR2->hw.GetJointTorqueCmd(PR2::ARM_R_WRIST_PITCH, &t6);	
	this->myPR2->hw.GetJointTorqueCmd(PR2::ARM_R_WRIST_ROLL, &t7);
	cout<<"T1"<<t1<<endl<<"T2"<<t2<<endl<<"T3"<<t3<<endl<<"T4"<<t4<<endl<<"T5"<<t5<<endl<<"T6"<<t6<<endl<<"T7"<<t7<<endl;

	//PR2_ERROR_CODE PR2Robot::GetArmJointTorqueActual(PR2_MODEL_ID id, double torque[])

	
	double torques[7];
	this->myPR2->GetArmJointTorqueActual(PR2::PR2_RIGHT_ARM,torques);
	for (int i = 0;i<7;i++){
	cout<<i<<":"<<torques[i]<<endl;

	}
*/
}

void
GazeboNode::cmd_rightarmconfigReceived()
{
  this->lock.lock();
	/*
  printf("turret angle: %.3f\n", this->rightarm.turretAngle);
  printf("shoulder pitch : %.3f\n", this->rightarm.shoulderLiftAngle);
  printf("shoulder roll: %.3f\n", this->rightarm.upperarmRollAngle);
  printf("elbow pitch: %.3f\n", this->rightarm.elbowAngle);
  printf("elbow roll: %.3f\n", this->rightarm.forearmRollAngle);
  printf("wrist pitch angle: %.3f\n", this->rightarm.wristPitchAngle);
  printf("wrist roll: %.3f\n", this->rightarm.wristRollAngle);
  printf("gripper gap: %.3f\n", this->rightarm.gripperGapCmd);
	
	double jointPosition[] = {this->rightarm.turretAngle,
														this->rightarm.shoulderLiftAngle,
														this->rightarm.upperarmRollAngle,
														this->rightarm.elbowAngle,
														this->rightarm.forearmRollAngle,
														this->rightarm.wristPitchAngle,
														this->rightarm.wristRollAngle,
														this->rightarm.gripperGapCmd};
	double jointSpeed[] = {0,0,0,0,0,0,0,0};

//	this->myPR2->SetArmJointPosition(PR2::PR2_LEFT_ARM, jointPosition, jointSpeed);
	*/
	//*
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_PAN           , this->rightarm.turretAngle,       0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_SHOULDER_PITCH, this->rightarm.shoulderLiftAngle, 0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_SHOULDER_ROLL , this->rightarm.upperarmRollAngle, 0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_ELBOW_PITCH   , this->rightarm.elbowAngle,        0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_ELBOW_ROLL    , this->rightarm.forearmRollAngle,  0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_WRIST_PITCH   , this->rightarm.wristPitchAngle,   0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_WRIST_ROLL    , this->rightarm.wristRollAngle,    0);
//Ignore gripper commands for now	
	//this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_GRIPPER       , this->rightarm.gripperGapCmd,     0);
	//this->myPR2->hw.CloseGripper(PR2::PR2_RIGHT_GRIPPER, this->rightarm.gripperGapCmd, this->rightarm.gripperForceCmd);
	//*/
	
  this->lock.unlock();
}


void
GazeboNode::cmd_leftarmconfigReceived()
{
  this->lock.lock();
	/*
	double jointPosition[] = {this->leftarm.turretAngle,
														this->leftarm.shoulderLiftAngle,
														this->leftarm.upperarmRollAngle,
														this->leftarm.elbowAngle,
														this->leftarm.forearmRollAngle,
														this->leftarm.wristPitchAngle,
														this->leftarm.wristRollAngle,
														this->leftarm.gripperGapCmd};
	double jointSpeed[] = {0,0,0,0,0,0,0,0};
	this->myPR2->SetArmJointPosition(PR2::PR2_LEFT_ARM, jointPosition, jointSpeed);
	*/

	//*
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_PAN           , this->leftarm.turretAngle,       0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_SHOULDER_PITCH, this->leftarm.shoulderLiftAngle, 0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_SHOULDER_ROLL , this->leftarm.upperarmRollAngle, 0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_ELBOW_PITCH   , this->leftarm.elbowAngle,        0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_ELBOW_ROLL    , this->leftarm.forearmRollAngle,  0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_WRIST_PITCH   , this->leftarm.wristPitchAngle,   0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_L_WRIST_ROLL    , this->leftarm.wristRollAngle,    0);
//Ignore gripper commands for now
//	this->myPR2->SetJointServoCmd(PR2::ARM_L_GRIPPER       , this->leftarm.gripperGapCmd,     0);
	//this->myPR2->hw.CloseGripper(PR2::PR2_LEFT_GRIPPER, this->leftarm.gripperGapCmd, this->leftarm.gripperForceCmd);
	//*/
  this->lock.unlock();
}

void
GazeboNode::cmdvelReceived()
{
  this->lock.lock();
  double dt;
  double w11, w21, w12, w22;
  double cmdSpeeds[7] = {0,0,0,0,0,0,0}; //DEBUG Used to check inverse kinematic solver

  // smooth out the commands by time decay
  // with w1,w2=1, this means equal weighting for new command every second
  this->myPR2->hw.GetSimTime(&(this->simTime));
  dt = simTime - lastTime;

  // smooth if dt is larger than zero
  if (dt > 0.0)
  {
    w11 =  0.0;
    w21 =  1.0;
    w12 =  0.0;
    w22 =  1.0;
    this->vxSmooth = (w11 * this->vxSmooth + w21*dt *this->velMsg.vx)/( w11 + w21*dt);
    this->vwSmooth = (w12 * this->vwSmooth + w22*dt *this->velMsg.vw)/( w12 + w22*dt);
  }

  // when running with the 2dnav stack, we need to refrain from moving when steering angles are off.
  // when operating with the keyboard, we need instantaneous setting of both velocity and angular velocity.

  // std::cout << "received cmd: vx " << this->velMsg.vx << " vw " <<  this->velMsg.vw
  //           << " vxsmoo " << this->vxSmooth << " vxsmoo " <<  this->vwSmooth
  //           << " | steer erros: " << this->myPR2->BaseSteeringAngleError() << " - " <<  M_PI/100.0
  //           << std::endl;

  // 2dnav: if steering angle is wrong, don't move or move slowly
  if (this->myPR2->BaseSteeringAngleError() > M_PI/100.0)
  {
    // set steering angle only
    this->myPR2->SetBaseSteeringAngle    (this->vxSmooth,0.0,this->vwSmooth);
  }
  else
  {
    // set base velocity
    this->myPR2->SetBaseCartesianSpeedCmd(this->vxSmooth, 0.0, this->vwSmooth);
  }

  // TODO: this is a hack, need to rewrite
  //       if we are trying to stop, send the command through
  if (this->velMsg.vx == 0.0)
  {
    // set base velocity
    this->myPR2->SetBaseCartesianSpeedCmd(this->vxSmooth, 0.0, this->vwSmooth);
  }

  this->lastTime = this->simTime;

  this->lock.unlock();
}

void GazeboNode::DisableRightArm(){
 	  this->myPR2->hw.SetJointControlMode(PR2::ARM_R_PAN             , PR2::DISABLED); 
          this->myPR2->hw.SetJointControlMode(PR2::ARM_R_SHOULDER_PITCH  , PR2::DISABLED);
          this->myPR2->hw.SetJointControlMode(PR2::ARM_R_SHOULDER_ROLL   , PR2::DISABLED); 
          this->myPR2->hw.SetJointControlMode(PR2::ARM_R_ELBOW_PITCH     , PR2::DISABLED);
          this->myPR2->hw.SetJointControlMode(PR2::ARM_R_ELBOW_ROLL      , PR2::DISABLED); 
          this->myPR2->hw.SetJointControlMode(PR2::ARM_R_WRIST_PITCH     , PR2::DISABLED);
          this->myPR2->hw.SetJointControlMode(PR2::ARM_R_WRIST_ROLL      , PR2::DISABLED); 

}

void GazeboNode::DisableLeftArm(){
 	  this->myPR2->hw.SetJointControlMode(PR2::ARM_L_PAN             , PR2::DISABLED); 
          this->myPR2->hw.SetJointControlMode(PR2::ARM_L_SHOULDER_PITCH  , PR2::DISABLED);
          this->myPR2->hw.SetJointControlMode(PR2::ARM_L_SHOULDER_ROLL   , PR2::DISABLED); 
          this->myPR2->hw.SetJointControlMode(PR2::ARM_L_ELBOW_PITCH     , PR2::DISABLED);
          this->myPR2->hw.SetJointControlMode(PR2::ARM_L_ELBOW_ROLL      , PR2::DISABLED); 
          this->myPR2->hw.SetJointControlMode(PR2::ARM_L_WRIST_PITCH     , PR2::DISABLED);
          this->myPR2->hw.SetJointControlMode(PR2::ARM_L_WRIST_ROLL      , PR2::DISABLED); 
}

void GazeboNode::PrintRightArm(){
	/*
	double speed[7],fake[7];
	//PR2_ERROR_CODE PR2Robot::GetArmJointSpeedCmd(PR2_MODEL_ID id, double speed[])

	//Write joint speed commands 
	this->myPR2->GetArmJointPositionCmd(PR2::PR2_RIGHT_ARM,speed,fake);
	
	cout<<"S1:"<<speed[0]<<endl<<"S2:"<<speed[1]<<endl<<"S3:"<<speed[2]<<endl<<"S4:"<<speed[3]<<endl<<"S5:"<<speed[4]<<endl<<"S6:"<<speed[5]<<endl<<"S7:"<<speed[6]<<endl;	

	//Write actual joint speeds
	//PR2_ERROR_CODE PR2Robot::GetArmJointSpeedCmd(PR2_MODEL_ID id, double speed[])
	this->myPR2->GetArmJointPositionActual(PR2::PR2_RIGHT_ARM,speed,fake);
	
	cout<<"AS1:"<<speed[0]<<endl<<"AS2:"<<speed[1]<<endl<<"AS3:"<<speed[2]<<endl<<"AS4:"<<speed[3]<<endl<<"AS5:"<<speed[4]<<endl<<"AS6:"<<speed[5]	<<endl<<"AS7:"<<speed[6]<<endl;	
*/
	
	double speed[7];
	//Write joint speed commands 
	this->myPR2->GetArmJointSpeedCmd(PR2::PR2_RIGHT_ARM,speed);
	
	cout<<"S1:"<<speed[0]<<endl<<"S2:"<<speed[1]<<endl<<"S3:"<<speed[2]<<endl<<"S4:"<<speed[3]<<endl<<"S5:"<<speed[4]<<endl<<"S6:"<<speed[5]<<endl<<"S7:"<<speed[6]<<endl;	

	//Write actual joint speeds
	//PR2_ERROR_CODE PR2Robot::GetArmJointSpeedCmd(PR2_MODEL_ID id, double speed[])
	this->myPR2->GetArmJointSpeedActual(PR2::PR2_RIGHT_ARM,speed);
	
	cout<<"AS1:"<<speed[0]<<endl<<"AS2:"<<speed[1]<<endl<<"AS3:"<<speed[2]<<endl<<"AS4:"<<speed[3]<<endl<<"AS5:"<<speed[4]<<endl<<"AS6:"<<speed[5]	<<endl<<"AS7:"<<speed[6]<<endl;	

	}

//CONSTRUCTOR
GazeboNode::GazeboNode(int argc, char** argv, const char* fname) :
        ros::node("rosgazebo"),tf(*this)
{
double currentTime;
  // initialize random seed
  srand(time(NULL));

  // Initialize robot object
  this->myPR2 = new PR2::PR2Robot();
  // Initialize connections
  this->myPR2->InitializeRobot();
 
//Make initial commanded velocities 0
 for (int i=0;i<6;i++){
	currentVel[i] = 0.0;
}

//Logging at higher level
/*
  //Get log file name
  char name[256];
  cout << "Enter filename: ";
  cin.getline (name,256);
  datafile.open (name);
*/

//Call to lower the arms to a hanging position

//Lower arms, wait for SERVOTIME seconds of sim time to elapse
/*
LowerArms();
currentTime = 0.0;
while(currentTime<SERVOTIME){
this->myPR2->hw.GetSimTime(&currentTime);
}
*/

//DEBUG
  //Set arms to speed control via torque
//this->myPR2->SetArmControlMode(PR2::PR2_LEFT_ARM,PR2::PR2_SPEED_TORQUE_CONTROL);
//this->myPR2->SetArmControlMode(PR2::PR2_RIGHT_ARM,PR2::PR2_SPEED_TORQUE_CONTROL);

//Set arms to position control via torque
//this->myPR2->SetArmControlMode(PR2::PR2_LEFT_ARM,PR2::PR2_CARTESIAN_TORQUE_CONTROL);
//this->myPR2->SetArmControlMode(PR2::PR2_RIGHT_ARM,PR2::PR2_CARTESIAN_TORQUE_CONTROL);

//TUNE
//Uncomment to restore speed control
//this->myPR2->SetArmControlMode(PR2::PR2_LEFT_ARM,PR2::PR2_CARTESIAN_CONTROL);
//this->myPR2->SetArmControlMode(PR2::PR2_RIGHT_ARM,PR2::PR2_CARTESIAN_CONTROL);

//Position control via PD controllers
//this->myPR2->SetArmControlMode(PR2::PR2_LEFT_ARM,PR2::PR2_JOINT_CONTROL);
//this->myPR2->SetArmControlMode(PR2::PR2_RIGHT_ARM,PR2::PR2_JOINT_CONTROL);

//DisableRightArm(); //Set all torques on right arm to zero.
//DisableLeftArm();  //Set all torques on left arm to zero

//Conrol a single joint
//this->myPR2->hw.SetJointControlMode(PR2::ARM_R_SHOULDER_PITCH, PR2::PD_TORQUE_CONTROL);

//Sanity check for joint type
PR2::PR2_JOINT_CONTROL_MODE mode;

//GetJointControlMode(PR2_JOINT_ID id, PR2_JOINT_CONTROL_MODE *mode);
this->myPR2->hw.GetJointControlMode(PR2::ARM_R_WRIST_ROLL,&mode);
cout<<"****************First Joint mode set to: "<<mode<<endl;
  //cin.getline (name,256);
/*
  this->myPR2->hw.SetJointControlMode(PR2::ARM_R_PAN             , PR2::SPEED_TORQUE_CONTROL); 
  this->myPR2->hw.SetJointControlMode(PR2::ARM_R_SHOULDER_PITCH  , PR2::SPEED_TORQUE_CONTROL);
  this->myPR2->hw.SetJointControlMode(PR2::ARM_R_SHOULDER_ROLL   , PR2::SPEED_TORQUE_CONTROL); 
  this->myPR2->hw.SetJointControlMode(PR2::ARM_R_ELBOW_PITCH     , PR2::SPEED_TORQUE_CONTROL);
  this->myPR2->hw.SetJointControlMode(PR2::ARM_R_ELBOW_ROLL      , PR2::SPEED_TORQUE_CONTROL); 
  this->myPR2->hw.SetJointControlMode(PR2::ARM_R_WRIST_PITCH     , PR2::SPEED_TORQUE_CONTROL);
  this->myPR2->hw.SetJointControlMode(PR2::ARM_R_WRIST_ROLL      , PR2::SPEED_TORQUE_CONTROL); 
*/


this->myPR2->hw.GetJointControlMode(PR2::ARM_R_SHOULDER_PITCH,&mode);
cout<<"*************Second Joint mode set to: "<<mode<<endl;
//cin.getline(name,256);

//Set all to 0
//Command servo
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_PAN           , 0,0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_SHOULDER_PITCH, 0, 0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_SHOULDER_ROLL , 0, 0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_ELBOW_PITCH   ,0,0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_ELBOW_ROLL    ,0,0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_WRIST_PITCH   , 0,0);
	this->myPR2->hw.SetJointServoCmd(PR2::ARM_R_WRIST_ROLL    , 0,0);

  // Set control mode for the arms
  // FIXME: right now this just sets default to pd control
  //this->myPR2->SetArmControlMode(PR2::PR2_RIGHT_ARM, PR2::PR2_JOINT_CONTROL);
  //this->myPR2->SetArmControlMode(PR2::PR2_LEFT_ARM, PR2::PR2_JOINT_CONTROL);
	//------------------------------------------------------------

  this->myPR2->hw.GetSimTime(&(this->lastTime));
  this->myPR2->hw.GetSimTime(&(this->simTime));


  // set torques for driving the robot wheels
  // this->myPR2->hw.SetJointTorque(PR2::CASTER_FL_DRIVE_L, 1000.0 );
  // this->myPR2->hw.SetJointTorque(PR2::CASTER_FR_DRIVE_L, 1000.0 );
  // this->myPR2->hw.SetJointTorque(PR2::CASTER_RL_DRIVE_L, 1000.0 );
  // this->myPR2->hw.SetJointTorque(PR2::CASTER_RR_DRIVE_L, 1000.0 );
  // this->myPR2->hw.SetJointTorque(PR2::CASTER_FL_DRIVE_R, 1000.0 );
  // this->myPR2->hw.SetJointTorque(PR2::CASTER_FR_DRIVE_R, 1000.0 );
  // this->myPR2->hw.SetJointTorque(PR2::CASTER_RL_DRIVE_R, 1000.0 );
  // this->myPR2->hw.SetJointTorque(PR2::CASTER_RR_DRIVE_R, 1000.0 );

}

void GazeboNode::finalize(int)
{
  fprintf(stderr,"Caught sig, clean-up and exit\n");
  sleep(1);
  exit(-1);
}


int
GazeboNode::SubscribeModels()
{
  //advertise<std_msgs::LaserScan>("laser");
  advertise<std_msgs::LaserScan>("scan");
  advertise<std_msgs::RobotBase2DOdom>("odom");
  advertise<std_msgs::Image>("image");
  advertise<std_msgs::PointCloudFloat32>("cloud");
  advertise<std_msgs::PointCloudFloat32>("full_cloud");
  advertise<std_msgs::Empty>("shutter");
  advertise<std_msgs::PR2Arm>("left_pr2arm_pos");
  advertise<std_msgs::PR2Arm>("right_pr2arm_pos");

  subscribe("cmd_vel", velMsg, &GazeboNode::cmdvelReceived);
  subscribe("cmd_leftarmconfig", leftarm, &GazeboNode::cmd_leftarmconfigReceived);
  subscribe("cmd_rightarmconfig", rightarm, &GazeboNode::cmd_rightarmconfigReceived);
  
  subscribe("arm_trajectory", trajectorymsg, &GazeboNode::cmd_trajectoryReceived);

  cout<<"Subscribed models"<<endl;
  return(0);
}

GazeboNode::~GazeboNode()
{
// 	datafile.close(); //Ensure we shut down the stream for writing to file
}

double
GazeboNode::GaussianKernel(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = 2.0*((double)rand()/(double)RAND_MAX-0.5); // normalized uniform random variable
  double V = 2.0*((double)rand()/(double)RAND_MAX-0.5); // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

void
GazeboNode::Update()
{
  this->lock.lock();

  float    angle_min;
  float    angle_max;
  float    angle_increment;
  float    range_max;
  uint32_t ranges_size;
  uint32_t ranges_alloc_size;
  uint32_t intensities_size;
  uint32_t intensities_alloc_size;
  std_msgs::Point3DFloat32 tmp_cloud_pt;


  /***************************************************************/
  /*                                                             */
  /*  Record data                                                */
  /*                                                             */
  /***************************************************************/

/*
	datafile<<this->simTime<<" ";

	double jointPosition[6];
	double jointSpeed[6];

	double myx;
	double myy;
	double myz;
	double myr;
	double myp;
	double myyaw;
	this->myPR2->hw.GetWristPoseGroundTruth(PR2::PR2_RIGHT_ARM, &myx,&myy,&myz,&myr,&myp,&myyaw);
	datafile<<myx<<" ";
	datafile<<myy<<" ";
	datafile<<myz<<" ";
	datafile<<endl<<" ";
*/
/*
	//perform forward kinematics for when we move out of simulation
	KDL::Frame f;
	if (this->myPR2->pr2_kin.FK(pr2_config,f)){
		//Get cartesian coordinates- last column of 4x4 frame
		datafile<<f(3,0)<<" "; //x
		datafile<<f(3,1)<<" "; //y
		datafile<<f(3,2)<<" "; //z
		datafile<<endl;
	}
*/	


  /***************************************************************/
  /*                                                             */
  /*  Update Commanded Velocities                                */
  /*                                                             */
  /***************************************************************/
//	SetRightArmVelocities(currentVel);
//PrintRightArm();
PrintRightArmPosition();


this->myPR2->hw.SetJointServoCmd(PR2::HEAD_LASER_PITCH , 0.0, 0.0); //Hold the laser still

  this->lock.unlock();
}



int 
main(int argc, char** argv)
{ 
  ros::init(argc,argv);

  GazeboNode gn(argc,argv,argv[1]);

  signal(SIGINT,  (&gn.finalize));
  signal(SIGQUIT, (&gn.finalize));
  signal(SIGTERM, (&gn.finalize));

  if (gn.SubscribeModels() != 0)
    exit(-1);

  while(1)
  {
    gn.Update();
    usleep(100000);
  }
  
  // have to call this explicitly for some reason.  probably interference
  // from signal handling in Stage / FLTK?


  ros::msg_destruct();

  exit(0);

}

/*
void GazeboNode::RecordData(PR2_JOINT_ID id, ofstream data){
	double jointPosition, jointSpeed;
	double 

}
*/
