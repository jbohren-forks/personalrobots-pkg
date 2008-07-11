#include <genericControllers/JointController.h>

//Todo: 
//1. Get and set params via server
//2. Integrate Joint and robot objects
//3. Integrate Motor controller time
//4. Subscribe to setpoints
/*
void JointController::SubscribeToInput(string name){
		 // Subscribe to the input messages
		  subscribe(name, inputBus, &JointController::InputCallback())
}
	
void JointController::InputCallback(){
	if(controlMode == CONTROLLER::CONTROLLER_TORQUE){ //Issue torque command
		SetTorqueCmd(inputBus.TorqueCommand);
	}
	else if (controlMode == CONTROLLER::CONTROLLER_POSITION){ //Issue position command
		SetPosCmd(inputBus.PositionCommand);
	}
	else if (controlMode == CONTROLLER:CONTROLLER_VELOCITY){ //Issue velocity command

		SetVelCmd(inputBus.VelocityCommand);
	}
}

*/

JointController::JointController(double PGain, double IGain, double DGain, double IMax, double IMin, CONTROLLER::CONTROLLER_CONTROL_MODE mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort){
	//Instantiate PID class
	pidController.InitPid(PGain,IGain,DGain,IMax,IMin); //Constructor for pid controller	

	//Set commands to zero
	cmdTorque = 0;
	cmdPos = 0;
	cmdVel = 0;
	
	lastTime = time;

	this->PGain = PGain;
	this->IGain = IGain;
	this->DGain = DGain;
	this->IMax = IMax;
	this->IMin = IMin;

	this->maxPositiveTorque = maxPositiveTorque;
	this->maxNegativeTorque = maxNegativeTorque;
	this->maxEffort = maxEffort;

	EnableController();

	controlMode = mode;
}

//JointController::JointController(Robot* robot, string name)
JointController::JointController(string name)
{	
	//Record namespace and robot pointer
	jointName = name; 
//	myRobot = robot;

	//Access the param server to fill in gains
	//TODO: Put in 
	/*
	PGain = GetParam("PGain");
	IGain = GetParam("IGain");
	DGain = GetParam("DGain");
	IMax = GetParam("IMax");
	IMin = GetParam("IMin");
	controlMode = GetParam("ControlMode");
	*/
	
	//Instantiate PID class
	pidController.InitPid(PGain,IGain,DGain,IMax,IMin); //Constructor for pid controller	

	//Set commands to zero
	cmdTorque = 0;
	cmdPos = 0;
	cmdVel = 0;

	//Initialize the time
	GetTime(&lastTime);

}
     
JointController::JointController( )
{
	
}


JointController::~JointController( )
{
	
}

//Returns the current time. Will eventually mode to use motor board controller time TODO
PR2::PR2_ERROR_CODE JointController::GetTime(double *time){
	return PR2::PR2_ALL_OK;
//	return(myPR2->hw.GetSimTime(time));
}

//Set the controller control mode
void JointController::SetMode(CONTROLLER::CONTROLLER_CONTROL_MODE mode){
	controlMode = mode;
}

//Getter for control mode
CONTROLLER::CONTROLLER_CONTROL_MODE JointController::GetMode(){
	return controlMode;
}

//Allow controller to function
void JointController::EnableController(){
	enabled = true;
}

//Disable functioning. Set joint torque to zero.
void JointController::DisableController(){
	enabled = false;
	//thisJoint->commandedEffort = 0; //Immediately set commanded Effort to 0
}

//-
//Torque
//-

//Truncates (if needed), then sets the torque
double JointController::SetTorque(double torque)
{
	double newTorque;
	CONTROLLER::CONTROLLER_ERROR_CODE status = CONTROLLER::CONTROLLER_ALL_OK;
		
	//Read the max positive and max negative torque once
	//maxPositiveTorque = thisJoint->effortLimit;
	//maxNegativeTorque = -1*thisJoint->effortLimit; 

	if(torque>maxPositiveTorque){
		newTorque = maxPositiveTorque;
		SaturationFlag = true;
	}
	else if (torque< maxNegativeTorque) {
		newTorque = maxNegativeTorque;
		SaturationFlag = true;
	}
	else {
		newTorque = torque;
		SaturationFlag = false;
	}

	
	//Set torque command 
//	thisJoint->commandedEffort = newTorque; 
	
	return newTorque;
}

CONTROLLER::CONTROLLER_ERROR_CODE JointController::SetTorqueCmd(double torque){
//	maxEffort = thisJoint->effortLimit;
	
	if(GetMode() != CONTROLLER::CONTROLLER_TORQUE)  //Make sure we're in torque command mode
	return CONTROLLER::CONTROLLER_MODE_ERROR;
	
	cmdTorque = torque;	
	
	if(cmdTorque > maxEffort){ //Truncate to positive limit
		cmdTorque = maxEffort;
		return CONTROLLER::CONTROLLER_TORQUE_LIMIT;
	}
	else if (cmdPos < -1*maxEffort){ //Truncate to negative limit
		cmdTorque = -1*maxEffort;
		return CONTROLLER::CONTROLLER_TORQUE_LIMIT;
	}
	
	return CONTROLLER::CONTROLLER_ALL_OK;

}

//Return current torque command
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::GetTorqueCmd(double *torque)
{
	*torque = cmdTorque;
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//Query motor for actual torque 
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::GetTorqueAct(double *torque)
{
//	*torque = thisJoint->appliedEffort; //Read torque from joint
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//-
//Position
//-

//Query mode, then set desired position 
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::SetPosCmd(double pos)
{
	if(GetMode() != CONTROLLER::CONTROLLER_POSITION)  //Make sure we're in position command mode
	return CONTROLLER::CONTROLLER_MODE_ERROR;
	
	cmdPos = pos;	
//	if(cmdPos > thisJoint->jointLimitMax){ //Truncate to positive limit
//		cmdPos = thisJoint->jointLimitMax;
//		return CONTROLLER::CONTROLLER_JOINT_LIMIT;
//	}
//	else if (cmdPos < thisJoint->jointLimitMin){ //Truncate to negative limit
//		cmdPos = thisJoint->jointLimitMin;
//		return CONTROLLER::CONTROLLER_JOINT_LIMIT;
//	}
	return CONTROLLER::CONTROLLER_ALL_OK;
	
}

//Return the current position command
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::GetPosCmd(double *pos)
{
	*pos = cmdPos;
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//Query the joint for the actual position
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::GetPosAct(double *pos)
{
//	*pos = thisJoint->position;
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//
//-
//Velocity
//-
//Check mode, then set the commanded velocity
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::SetVelCmd(double vel)
{
	if(GetMode() == CONTROLLER::CONTROLLER_VELOCITY){ //Make sure we're in velocity command mode
		cmdVel = vel;	
		return CONTROLLER::CONTROLLER_ALL_OK;
	}
	else return CONTROLLER::CONTROLLER_MODE_ERROR;
}

//Return the internally stored commanded velocity
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::GetVelCmd(double *vel)
{
	*vel = cmdVel;
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//Query our joint for velocity
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::GetVelAct(double *vel)
{
//	*vel = thisJoint->velocity;
	return CONTROLLER::CONTROLLER_ALL_OK;
}

bool JointController::CheckForSaturation(void){ 
	return SaturationFlag;
}

//Call each tick. Based on controller mode, will close the correct loop then call SetTorque to issue command
 void JointController::Update(void){
	double error, currentTorqueCmd, time, cmd, act;
	GetTime(&time);
	CONTROLLER_CONTROL_MODE type = GetMode();
	if(type==CONTROLLER::CONTROLLER_TORQUE){
		currentTorqueCmd = cmdTorque; //In torque mode, we pass along the commanded torque
	}
	else if (type==CONTROLLER::CONTROLLER_POSITION){
		GetPosCmd(&cmd);
		GetPosAct(&act);
		//Read position, get error
		error = CONTROLLER::Controller::ModNPi2Pi(cmd-act); 
	
		//Update the controller
		currentTorqueCmd = pidController.UpdatePid(error,time); //Close the loop around position
	}
	else if (type==CONTROLLER::CONTROLLER_VELOCITY){
		GetVelCmd(&cmd);
		GetVelCmd(&act);
		//Read velocity, get error
		error = cmd - act; 
	
		//Update the controller
		currentTorqueCmd = pidController.UpdatePid(error,time); //Close the loop around velocity

	}
	else currentTorqueCmd = 0; //On error, set torque to zero
	//Issue the torque command
	if(enabled) SetTorque(currentTorqueCmd); 	
	else SetTorque(0); //Send a zero command if disabled

}

double JointController::SimUpdate(double position, double velocity, double time){
	double error, currentTorqueCmd, cmd, act;
	CONTROLLER_CONTROL_MODE type = GetMode();
	if(type==CONTROLLER::CONTROLLER_TORQUE){
		currentTorqueCmd = cmdTorque; //In torque mode, we pass along the commanded torque
	}
	else if (type==CONTROLLER::CONTROLLER_POSITION){
		GetPosCmd(&cmd);
		//Read position, get error
		error = CONTROLLER::Controller::ModNPi2Pi(cmd-position); 
	
		//Update the controller
		currentTorqueCmd = pidController.UpdatePid(error,time); //Close the loop around position
	}
	else if (type==CONTROLLER::CONTROLLER_VELOCITY){
		GetVelCmd(&cmd);
		//Read velocity, get error
		error = cmd - velocity; 
	
		//Update the controller
		currentTorqueCmd = pidController.UpdatePid(error,time); //Close the loop around velocity

	}
	else currentTorqueCmd = 0; //On error, set torque to zero
	//Issue the torque command
	if(enabled) return SetTorque(currentTorqueCmd); 	
	else return 0; //Send a zero command if disabled


}

//-
// Interact with param server
//-

//TODO: Case statement to effect changes when parameters are set here
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::SetParam(string label,double value)
{ 	
  return CONTROLLER::CONTROLLER_ALL_OK;
}

/*
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::SetParam(string label,string value)
{
  return CONTROLLER::CONTROLLER_ALL_OK;
}
*/

CONTROLLER::CONTROLLER_ERROR_CODE JointController::GetParam(string label, double* value){
}

/*
CONTROLLER::CONTROLLER_ERROR_CODE JointController::GetParam(string label, string value){
}
*/
