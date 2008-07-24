#include <pr2Controllers/LaserScannerController.h>
#define DEMO

using namespace controller;

 //---------------------------------------------------------------------------------//
 //CONSTRUCTION/DESTRUCTION CALLS
 //---------------------------------------------------------------------------------//


LaserScannerController::LaserScannerController()
{
  controlMode = CONTROLLER_DISABLED;

	profileX = NULL;
	profileT = NULL; 
  counter = 0;
}
    
LaserScannerController::~LaserScannerController( )
{
	if(profileX!=NULL) delete[] profileX;
	if(profileT!=NULL) delete[] profileT;

}

/*
LaserScannerController::LaserScannerController(Joint* joint, std::string name);
	//Pass in joint* when we get it
	lowerControl.init(joint,name);
  this->name = name;
	//Set gains
	
	// Set control mode 
	lowerControl.setMode(CONTROLLER_POSITION);
	
	//Enable controller
	//lowerControl.enableController();
	
	
}
*/
 
void LaserScannerController::Init(double PGain, double IGain, double DGain, double IMax, double IMin, controllerControlMode mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort, mechanism::Joint *joint){
  
  //If we're in automatic mode, want to pass down position control mode to lower controller
  controllerControlMode newMode;

  controlMode = mode;
  if(mode==CONTROLLER_AUTOMATIC) {
    newMode = CONTROLLER_POSITION;
  }
  else newMode = mode;

  lowerControl.init(PGain,IGain,DGain,IMax,IMin,mode,time,maxEffort,-maxEffort,joint);

  EnableController();
}

 //---------------------------------------------------------------------------------//
 //AUTOMATIC PROFILE FUNCTIONS
 //---------------------------------------------------------------------------------//
void LaserScannerController::SetSawtoothProfile(double period, double amplitude, double dt, double offset){

	unsigned int elements = (unsigned int) (period/dt);
	setParam("profile" , "sawtooth");
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
	setParam("profile", "sinewave");
	unsigned int elements = (unsigned int) (period/dt);
	GenerateSinewave(profileX,profileT,period,amplitude,dt,offset,elements);
	profileLength = elements;
	profileIndex = 0; //Start at beginning of profile
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
	setParam("profile", "squarewave");
	unsigned int elements = (unsigned int) (period/dt);
	GenerateSquarewave(profileX,profileT,period,amplitude,dt,offset,elements);
	profileLength = elements;
	profileIndex = 0; //Start at beginning of profile
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
 
controllerErrorCode LaserScannerController::setProfile(double *&t, double *&x, int numElements)
{
	profileX = x;
	profileT = t;
	profileLength = numElements; //Set length of profile
	profileIndex = 0; //Start at beginning of profile
  return CONTROLLER_ALL_OK;
}
 
//---------------------------------------------------------------------------------//
//TIME CALLS
//
//---------------------------------------------------------------------------------//
void LaserScannerController::GetTime(double* time){
        lowerControl.getTime(time);
      }

//---------------------------------------------------------------------------------//
//MODE/ENABLE CALLS
//---------------------------------------------------------------------------------//

//Set the controller control mode
controllerControlMode LaserScannerController::SetMode(controllerControlMode mode){
  //Record top level of control mode
  controlMode = mode;

  //Want the lower level controller to be set to position control if we're providing an automatic profile
  if(mode==CONTROLLER_AUTOMATIC) {
    lowerControl.setMode(CONTROLLER_POSITION);
  }
  else{
    lowerControl.setMode(mode);
  }
  
  return CONTROLLER_MODE_SET;
}


controllerControlMode LaserScannerController::GetMode(void){
	return controlMode;
}


//Allow controller to function
controllerControlMode LaserScannerController::EnableController(){
  enabled = true;
  return lowerControl.enableController();
  
}

//Disable functioning. Set joint torque to zero.
controllerControlMode LaserScannerController::DisableController(){
  enabled = false;
  return lowerControl.disableController();   
}

//Check for saturation of last command
bool LaserScannerController::CheckForSaturation(void){ 
  return lowerControl.checkForSaturation();
}



//---------------------------------------------------------------------------------//
//TORQUE CALLS
//---------------------------------------------------------------------------------//

controllerErrorCode LaserScannerController::SetTorqueCmd(double torque){
	if(controlMode != CONTROLLER_TORQUE)  //Make sure we're in torque command mode
	return CONTROLLER_MODE_ERROR;
	
  return lowerControl.setTorqueCmd(torque);
  }

//Return current torque command
controllerErrorCode
LaserScannerController::GetTorqueCmd(double *torque)
{
  *torque = cmdTorque;
  return CONTROLLER_ALL_OK; 
}

//Query motor for actual torque 
controllerErrorCode
LaserScannerController::GetTorqueAct(double *torque)
{
	return lowerControl.getTorqueAct(torque);
}

//---------------------------------------------------------------------------------//
//POSITION CALLS
//---------------------------------------------------------------------------------//

//Query mode, then set desired position 
controllerErrorCode LaserScannerController::SetPosCmd(double pos)
{
	if(controlMode != CONTROLLER_POSITION)  //Make sure we're in position command mode
	return CONTROLLER_MODE_ERROR;
	
  return lowerControl.setPosCmd(pos);

	
}

//Return the current position command
controllerErrorCode LaserScannerController::GetPosCmd(double *pos)
{
  *pos = cmdPos;
	return CONTROLLER_ALL_OK;
}

//Query the joint for the actual position
controllerErrorCode LaserScannerController::GetPosAct(double *pos)
{
  return	lowerControl.getPosAct(pos);
}

 
//---------------------------------------------------------------------------------//
//VELOCITY CALLS
//---------------------------------------------------------------------------------//
//Check mode, then set the commanded velocity
controllerErrorCode LaserScannerController::SetVelCmd(double vel)
{
	if( controlMode != CONTROLLER_VELOCITY)  //Make sure we're in velocity command mode
	return CONTROLLER_MODE_ERROR;

  return  lowerControl.setVelCmd(vel);
}

//Return the internally stored commanded velocity
controllerErrorCode LaserScannerController::GetVelCmd(double *vel)
{
  return lowerControl.getVelCmd(vel);
}

//Query our joint for velocity
controllerErrorCode LaserScannerController::GetVelAct(double *vel)
{
//	lowerControl.getVelAct(vel);
	return CONTROLLER_ALL_OK;
}
      
//---------------------------------------------------------------------------------//
//UPDATE CALLS
//---------------------------------------------------------------------------------//

void LaserScannerController::Update( )
{
  if(!enabled) return; //Check for enabled
 /* 	double currentTime;
	lowerControl.getTime(&currentTime);
	//Check for automatic scan mode
	if(controlMode == CONTROLLER_AUTOMATIC){
		if(currentTime-lastCycleStart > profileT[profileIndex]){ //Check to see whether it's time for a new setpoint
			lowerControl.setPosCmd(profileX[profileIndex]); 
			
			//Advance time index. Reset if necessary
			if(profileIndex == profileLength-1){
				profileIndex = 0; //Restart profile
				lastCycleStart = currentTime;
			}
			else profileIndex++;
		}
		//No new setpoint necessary
	}
	*/
    if(controlMode == CONTROLLER_AUTOMATIC){
			lowerControl.setPosCmd(profileX[profileIndex]);
      //Every x calls, advance profile Index
      if(counter==1000){ //Hardcoded for now...
       counter = 0;			
		  	//Advance time index. Reset if necessary
		  	if(profileIndex == profileLength-1){
			  	profileIndex = 0; //Restart profile
			  } else profileIndex++;
      }
      else counter++;
		}
		//No new setpoint necessary
	
 
   
  /*
  double torque;
  //Print torque
  lowerControl.getTorqueCmd(&torque);
  std::cout<<"Torque:"<<torque<<std::endl;
 //Instruct lower level controller to update
 */

	lowerControl.update();

}

//---------------------------------------------------------------------------------//
//PARAM SERVER CALLS
//---------------------------------------------------------------------------------//

controllerErrorCode
LaserScannerController::setParam(std::string label,double value)
{

  return CONTROLLER_ALL_OK;
}
controllerErrorCode 
LaserScannerController::setParam(std::string label,std::string value)
{

  return CONTROLLER_ALL_OK;
}


