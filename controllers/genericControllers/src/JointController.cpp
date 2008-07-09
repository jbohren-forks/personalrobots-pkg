#include <genericControllers/JointController.h>


//Simulator mode constructor
JointController::JointController(PR2::PR2Robot* robot, PR2::PR2_JOINT_ID jointID, CONTROLLER::CONTROLLER_CONTROL_MODE mode, double pGain, double iGain, double dGain, double iMax, double iMin)
{	//Instantiate PID class
	pidController.InitPid(pGain,iGain,dGain,iMax,iMin); //Constructor for pid controller

	myPR2 = robot; //Set the robot 
	joint = jointID; //Set joint id

	//Set commands to zero
	cmdTorque = 0;
	cmdPos = 0;
	cmdVel = 0;

	//Initialize the time
	GetTime(&lastTime);

	//Set the control mode
	controlMode = mode;

}
    
JointController::~JointController( )
{
}

//Returns the current time. Will eventually mode to use ROS::TIME
PR2::PR2_ERROR_CODE JointController::GetTime(double *time){
	return(myPR2->hw.GetSimTime(time));
}

//Set the controller control mode
void JointController::SetMode(CONTROLLER::CONTROLLER_CONTROL_MODE mode){
	controlMode = mode;
}

//Getter for control mode
CONTROLLER::CONTROLLER_CONTROL_MODE JointController::GetMode(){
	return controlMode;
}
//-
//Torque
//-

//Truncates, then sets the torque
//Accesses the robot model to read the maximum and minimum torques
//Sets flag to indicate torque saturation
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::SetTorque(double torque)
{	CONTROLLER::CONTROLLER_ERROR_CODE status = CONTROLLER::CONTROLLER_ALL_OK;
	double newTorque, maxPositiveTorque, maxNegativeTorque;

	if(GetMode() == CONTROLLER::CONTROLLER_TORQUE){ //Make sure we're in torque control mode
		
		//Read the max positive and max negative torque once
		maxPositiveTorque = GetMaxPosTorque();
		maxNegativeTorque = GetMaxNegTorque(); 

		if(torque>maxPositiveTorque){
			newTorque = maxPositiveTorque;
			status = CONTROLLER::CONTROLLER_TORQUE_LIMIT; //Hit the positive torque limit
		}
		else if (torque< maxNegativeTorque) {
			newTorque = maxNegativeTorque;
			status = CONTROLLER::CONTROLLER_TORQUE_LIMIT; //Hit the negative torque limit
		}
		else newTorque = torque;

		//Set torque command 
		myPR2->hw.SetJointTorque(joint,newTorque);
		return status;
	}
	else return CONTROLLER::CONTROLLER_MODE_ERROR;
}

//Return current torque command
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::getTorqueCmd(double *torque)
{
	*torque = cmdTorque;
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//Query motor for actual torque TODO
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::getTorqueAct(double *torque)
{
	if(myPR2->hw.GetJointTorqueActual(joint,torque) == PR2::PR2_ALL_OK)
	return CONTROLLER::CONTROLLER_ALL_OK;
	else return CONTROLLER::CONTROLLER_MODE_ERROR;
}

//-
//Position
//-

//Query mode, then set desired position 
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::setPos(double pos)
{
	if(GetMode() == CONTROLLER::CONTROLLER_POSITION){ //Make sure we're in position command mode
		cmdPos = pos;	
		return CONTROLLER::CONTROLLER_ALL_OK;
	}
	else return CONTROLLER::CONTROLLER_MODE_ERROR;
}

//Return the current position command
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::getPosCmd(double *pos)
{
	*pos = cmdPos;
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//Query the joint for the actual position
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::getPosAct(double *pos)
{
	double fake; //Query velocity at the same time, but we don't care

	if(myPR2->hw.GetJointPositionActual(joint,pos,&fake)==PR2::PR2_ALL_OK) return CONTROLLER::CONTROLLER_ALL_OK;
	else return CONTROLLER::CONTROLLER_JOINT_ERROR;
			/*	SliderJoint *sjoint;
	HingeJoint *hjoint;
	switch(myJoint->GetType()){
		case Joint::SLIDER:
			sjoint = dynamic_cast<SliderJoint*>( tmpJoint );
			*pos = sjoint->GetPosition();
			return CONTROLLER::CONTROLLER_ALL_OK;
			break;
		case Joint::HINGE:
			hjoint = dynamic_cast<HingeJoint*>(this->joints[count]);
			*pos = hjoint->GetAngle();
			return CONTROLLER::CONTROLLER_ALL_OK;
			break;
		case Joint::HINGE2: //Not implemented yet. Return a joint error
			return CONTROLLER::CONTROLLER_JOINT_ERROR;
			break;
		case Joint::BALL:
			return CONTROLLER::CONTROLLER_JOINT_ERROR;
			break;
		case Joint::UNIVERSAL:
			return CONTROLLER::CONTROLLER_JOINT_ERROR;
			break;
		
		return CONTROLLER::CONTROLLER_ALL_OK;
		*/
}

//
//-
//Velocity
//-
//Check mode, then set the commanded velocity
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::setVel(double vel)
{
	if(GetMode() == CONTROLLER::CONTROLLER_VELOCITY){ //Make sure we're in velocity command mode
		cmdVel = vel;	
		return CONTROLLER::CONTROLLER_ALL_OK;
	}
	else return CONTROLLER::CONTROLLER_MODE_ERROR;
}

//Return the internally stored commanded velocity
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::getVelCmd(double *vel)
{
	*vel = cmdVel;
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//Query our joint for velocity
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::getVelAct(double *vel)
{
	double fake; //Query position at the same time, but we don't care

	if(myPR2->hw.GetJointPositionActual(joint,&fake,vel)==PR2::PR2_ALL_OK) return CONTROLLER::CONTROLLER_ALL_OK;
	else return CONTROLLER::CONTROLLER_JOINT_ERROR;
/*
	SliderJoint *sjoint;
	HingeJoint *hjoint;
	switch(myJoint->GetType()){
		case Joint::SLIDER:
			sjoint = dynamic_cast<SliderJoint*>( tmpJoint );
			*pos = sjoint->GetPositionRate();
			return CONTROLLER::CONTROLLER_ALL_OK;
			break;
		case Joint::HINGE:
			hjoint = dynamic_cast<HingeJoint*>(this->joints[count]);
			*pos = hjoint->GetAngleRate();
			return CONTROLLER::CONTROLLER_ALL_OK;
			break;
		case Joint::HINGE2: //Not implemented yet. Return a joint error
			return CONTROLLER::CONTROLLER_JOINT_ERROR;
			break;
		case Joint::BALL:
			return CONTROLLER::CONTROLLER_JOINT_ERROR;
			break;
		case Joint::UNIVERSAL:
			return CONTROLLER::CONTROLLER_JOINT_ERROR;
			break;
		
		return CONTROLLER::CONTROLLER_ALL_OK;
		*/
}

//-
//Params
//-
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::setParam(string label,double value)
{
 	
  return CONTROLLER::CONTROLLER_ALL_OK;
}

CONTROLLER::CONTROLLER_ERROR_CODE
JointController::setParam(string label,string value)
{
  return CONTROLLER::CONTROLLER_ALL_OK;
}

 void JointController::Update(void){
	double error, currentTorqueCmd, time, cmd, act;
	GetTime(&time);
	CONTROLLER_CONTROL_MODE type = GetMode();
	if(type==CONTROLLER::CONTROLLER_TORQUE){
		currentTorqueCmd = cmdTorque; //In torque mode, we pass along the commanded torque
	}
	else if (type==CONTROLLER::CONTROLLER_POSITION){
		getPosCmd(&cmd);
		getPosAct(&act);
		//Read position, get error
		error = CONTROLLER::Controller::ModNPi2Pi(cmd-act); 
	
		//Update the controller
		currentTorqueCmd = pidController.UpdatePid(error,time); //Close the loop around position
	}
	else if (type==CONTROLLER::CONTROLLER_VELOCITY){
		getVelCmd(&cmd);
		getVelCmd(&act);
		//Read velocity, get error
		error = cmd - act; 
	
		//Update the controller
		currentTorqueCmd = pidController.UpdatePid(error,time); //Close the loop around velocity

	}
	else currentTorqueCmd = 0; //On error, set torque to zero
	//Issue the torque command
	SetTorque(currentTorqueCmd); 	

}

//The following functions are stubs until we access the robot object properly
//TODO

//Returns max positive torque
double GetMaxPosTorque(void){
	return 100;
}

//Returns max negative torque
double GetMaxNegTorque(void){
	return 100;
}

