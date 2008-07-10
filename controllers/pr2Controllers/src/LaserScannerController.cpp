#include <pr2Controllers/LaserScannerController.h>


using namespace CONTROLLER;

LaserScannerController::LaserScannerController()
{
	profileX = NULL;
	profileT = NULL; 

	lowerControl.GetTime(&lastCycleStart); //Mark this time as beginning of cycle
}
    
LaserScannerController::~LaserScannerController( )
{
	if(profileX!=NULL) delete[] profileX;
	if(profileT!=NULL) delete[] profileT;
}

/*
LaserScannerController::LaserScannerController(){
	//Pass in joint* when we get it
	lowerControl(joint); //Call constructor on joint controller
	
	//Set gains
	
	// Set control mode to position control
	
	//Enable controller
}
*/
void
LaserScannerController::Update( )
{
	double currentTime;
	lowerControl.GetTime(&currentTime);
	//Check for automatic scan mode
	if(automaticProfile){
		if(currentTime-lastCycleStart > profileT[profileIndex]){ //Check to see whether it's time for a new setpoint
			lowerControl.SetPosCmd(profileX[profileIndex]); 
			
			//Advance time index. Reset if necessary
			if(profileIndex == profileLength-1){
				profileIndex = 0; //Restart profile
				lastCycleStart = currentTime;
			}
			else profileIndex++;
		}
		//No new setpoint necessary
	}
	else if (controlMode == CONTROLLER::CONTROLLER_POSITION)
	{
		lowerControl.SetPosCmd(cmdPos);
	}
	else if (controlMode == CONTROLLER::CONTROLLER_VELOCITY){
		lowerControl.SetVelCmd(cmdVel);
	}
	else if (controlMode == CONTROLLER::CONTROLLER_TORQUE){
		lowerControl.SetTorqueCmd(cmdTorque);
	}
	
	lowerControl.Update();
}

PR2::PR2_ERROR_CODE
LaserScannerController::setProfile(double *&t, double *&x, int numElements)
{
	profileX = x;
	profileT = t;
	profileLength = numElements; //Set length of profile
	profileIndex = 0; //Start at beginning of profile
  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
LaserScannerController::SetPosition(double pitch)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
LaserScannerController::setParam(string label,double value)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
LaserScannerController::setParam(string label,string value)
{

  return PR2::PR2_ALL_OK;
}

void LaserScannerController::SetSawtoothProfile(double period, double amplitude, double dt, double offset){

	unsigned int elements = (unsigned int) period/dt;
//	setParam('profile' , 'sawtooth');
	GenerateSawtooth(profileX,profileT,period,amplitude,dt,offset,elements);
	profileLength = elements;
}

void LaserScannerController::GenerateSawtooth(double *&x, double *&t, double period, double amplitude, double dt, double offset, unsigned int numElements){
	double lastPeakTime = 0.0; //Track the time of the last peak
	double timeDifference= 0.0;
	double currentTime = 0.0;
	double rawValue = 0.0;

	//Set up arrays
	x = new double[numElements];
	t = new double[numElements];

	for (unsigned int i = 0;i<numElements;i++){
		currentTime = i*dt;
		t[i] = currentTime;
		timeDifference = currentTime-lastPeakTime;
		rawValue = amplitude - (timeDifference * amplitude)/period;
		if(rawValue<0){
			lastPeakTime = currentTime;
			rawValue = amplitude;
		}
		 x[i] =  rawValue + offset; //Assume floor is 0
	}
}

void LaserScannerController::SetSinewaveProfile(double period, double amplitude, double dt, double offset){
//	setParam('profile', 'sinewave');
	unsigned int elements = (unsigned int) period/dt;
	GenerateSinewave(profileX,profileT,period,amplitude,dt,offset,elements);
	profileLength = elements;
}


void LaserScannerController::GenerateSinewave(double *&x, double *&t, double period, double amplitude,double dt, double offset, unsigned int numElements){

	double currentTime = 0.0;
	//Set up arrays
	x = new double[numElements];
	t = new double[numElements];

	for(unsigned int i = 0;i<numElements;i++){
		currentTime = i*dt;
		t[i] = currentTime;
		x[i] = amplitude* sin(2*M_PI*currentTime/period) + offset;
	}

}


void LaserScannerController::SetSquarewaveProfile(double period, double amplitude, double dt, double offset){
//	setParam('profile', 'squarewave');
	unsigned int elements = (unsigned int) period/dt;
	GenerateSquarewave(profileX,profileT,period,amplitude,dt,offset,elements);
	profileLength = elements;
}


void LaserScannerController::GenerateSquarewave(double *&x, double *&t, double period, double amplitude, double dt, double offset, unsigned int numElements){

	double currentTime = 0.0;
	double lastSwitchTime = 0.0;
	//Set up arrays
	x = new double[numElements];
	t = new double[numElements];

	for(unsigned int i = 0;i<numElements;i++){
		currentTime = i*dt;
		t[i] = currentTime;
		if((currentTime-lastSwitchTime)>period){
			lastSwitchTime = currentTime;
			x[i] = offset;
		}
		else if((currentTime-lastSwitchTime) < (period/2))x[i] = offset; //Lower part of square wave
		else x[i] = amplitude + offset;
	}
}


//-
//Torque
//-


CONTROLLER::CONTROLLER_ERROR_CODE LaserScannerController::SetTorqueCmd(double torque){
	lowerControl.SetTorqueCmd(torque);
}

//Return current torque command
CONTROLLER::CONTROLLER_ERROR_CODE
LaserScannerController::GetTorqueCmd(double *torque)
{
	if(GetMode() != CONTROLLER::CONTROLLER_TORQUE)  //Make sure we're in torque command mode
	return CONTROLLER::CONTROLLER_MODE_ERROR;
	

	lowerControl.GetTorqueCmd(torque);
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//Query motor for actual torque 
CONTROLLER::CONTROLLER_ERROR_CODE
 LaserScannerController::GetTorqueAct(double *torque)
{
	lowerControl.GetTorqueAct(torque);
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//-
//Position
//-

//Query mode, then set desired position 
CONTROLLER::CONTROLLER_ERROR_CODE
 LaserScannerController::SetPosCmd(double pos)
{
	if(GetMode() != CONTROLLER::CONTROLLER_POSITION)  //Make sure we're in position command mode
	return CONTROLLER::CONTROLLER_MODE_ERROR;
	
	lowerControl.SetPosCmd(pos);

	return CONTROLLER::CONTROLLER_ALL_OK;
	
}

//Return the current position command
CONTROLLER::CONTROLLER_ERROR_CODE
LaserScannerController::GetPosCmd(double *pos)
{
	lowerControl.GetPosCmd(pos);
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//Query the joint for the actual position
CONTROLLER::CONTROLLER_ERROR_CODE
LaserScannerController::GetPosAct(double *pos)
{
	lowerControl.GetPosAct(pos);
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//
//-
//Velocity
//-
//Check mode, then set the commanded velocity
CONTROLLER::CONTROLLER_ERROR_CODE
LaserScannerController::SetVelCmd(double vel)
{
	if(GetMode() != CONTROLLER::CONTROLLER_VELOCITY)  //Make sure we're in velocity command mode
	return CONTROLLER::CONTROLLER_MODE_ERROR;
	
	lowerControl.SetVelCmd(vel);

	return CONTROLLER::CONTROLLER_MODE_ERROR;
}

//Return the internally stored commanded velocity
CONTROLLER::CONTROLLER_ERROR_CODE
LaserScannerController::GetVelCmd(double *vel)
{
	lowerControl.GetVelCmd(vel);
	return CONTROLLER::CONTROLLER_ALL_OK;
}

//Query our joint for velocity
CONTROLLER::CONTROLLER_ERROR_CODE
LaserScannerController::GetVelAct(double *vel)
{
	lowerControl.GetVelAct(vel);
	return CONTROLLER::CONTROLLER_ALL_OK;
}

