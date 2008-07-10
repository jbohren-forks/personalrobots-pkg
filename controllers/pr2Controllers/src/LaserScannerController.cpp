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


