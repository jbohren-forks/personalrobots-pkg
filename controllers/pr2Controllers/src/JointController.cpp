#include <pr2Controllers/JointController.h>

JointController::JointController()
{
}
    
JointController::~JointController( )
{
}

PR2::PR2_ERROR_CODE
JointController::setPos(double pos)
{
  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
JointController::setTorq(double torq)
{
  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
JointController::setVel(double vel)
{
  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
JointController::setParam(string label,double value)
{
  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
JointController::setParam(string label,string value)
{
  return PR2::PR2_ALL_OK;
}



