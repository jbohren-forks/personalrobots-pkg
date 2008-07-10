#include <pr2Controllers/LaserScannerController.h>


using namespace CONTROLLER;

LaserScannerController::LaserScannerController()
{
}
    
LaserScannerController::~LaserScannerController( )
{
}

void
LaserScannerController::Update( )
{

}

PR2::PR2_ERROR_CODE
LaserScannerController::setProfile(double *t, double *x)
{

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


