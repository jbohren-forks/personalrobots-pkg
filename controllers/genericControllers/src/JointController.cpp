#include <genericControllers/JointController.h>

//What happens if we're set in velocity mode and we set position>? and vice versa?

JointController::JointController(double pGain, double iGain, double dGain, double iMax, double iMin)
{	//Instantiate PID class
	pidController = 
	//Read in gains for controller
	//Get motor object in here 
	//Set dParamFMax = 0 if we're in the simulator mode

	//Member variables
	//Robot * robot;
	//Motor * motor;
}
    
JointController::~JointController( )
{
}

//-
//Torque
//-

//Truncates, then sets the torque
//Accesses the robot model to read the maximum and minimum torques
//Sets flag to indicate torque saturation
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::setTorque(double torque)
{	CONTROLLER::CONTROLLER_ERROR_CODE status = CONTROLLER::CONTROLLER_ALL_OK;
	double newTorque, maxPositiveTorque, maxNegativeTorque;

	if(GetMode == CONTROLLER::CONTROLLER_TORQUE){ //Make sure we're in torque control mode
		
	//Read the max positive and max negative torque once
	maxPositiveTorque = robot->actuator[id]->MaxPositiveTorque;
	maxNegativeTorque = robot->actuator[id]->MaxNegativeTorque;

	if(torque>maxPositiveTorque){
		newTorque = maxPositiveTorque;
		status = CONTROLLER::CONTROLLER_TORQUE_LIMIT; //Hit the positive torque limit
	}
	else if (torque< maxNegativeTorque) {
		newTorque = maxNegativeTorque
		status = CONTROLLER::CONTROLLER_TORQUE_LIMIT; //Hit the negative torque limit
	}
	else newTorque = torque;

	//Actually set the torque value
	motor->SetTorque(newTorque);
		
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
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//-
//Position
//-

//Give a position setpoint
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::setPos(double pos)
{
	
	return CONTROLLER::CONTROLLER_ALL_OK;
}

CONTROLLER::CONTROLLER_ERROR_CODE
JointController::getPosCmd(double *pos)
{
	*pos = cmdPos;
	return CONTROLLER::CONTROLLER_ALL_OK;
}
//TODO
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::getPosAct(double *pos)
{
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//TODO
//-
//Velocity
//-
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::setVel(double vel)
{
	return CONTROLLER::CONTROLLER_ALL_OK;
}

CONTROLLER::CONTROLLER_ERROR_CODE
JointController::getVelCmd(double *vel)
{
	*vel = cmdVel;
	return CONTROLLER::CONTROLLER_ALL_OK;
}

CONTROLLER::CONTROLLER_ERROR_CODE
JointController::getVelAct(double *vel)
{
  return CONTROLLER::CONTROLLER_ALL_OK;
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

virtual void Update(void){
	double error, torqueCmd;
	//Read position, get error
	positionError = Controller.ModNPi2Pi(pos-motor->GetAngle()); 
	
	//Update the controller
	pidController->UpdatePid(positionError,robot->GetTime());

	//Issue the torque command
	SetTorque(torqueCmd); 	

}

