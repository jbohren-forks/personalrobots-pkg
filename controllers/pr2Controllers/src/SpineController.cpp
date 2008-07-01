#include <pr2Controllers/SpineController.h>

SpineController::SpineController()
{

}
    
SpineController::~SpineController( )
{

}

PR2::PR2_ERROR_CODE
SpineController::setPos(double z)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
SpineController::setForce(double fz)
{

  return PR2::PR2_ALL_OK;
}


PR2::PR2_ERROR_CODE
SpineController::setParam(string label,double value)
{

  return PR2::PR2_ALL_OK;
}
