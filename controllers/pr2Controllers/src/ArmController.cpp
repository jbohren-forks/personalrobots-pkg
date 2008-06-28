
#include <controller/ArmController.h>


ArmController::ArmController()
{
}
    
ArmController::~ArmController( )
{
}

PR2::PR2_ERROR_CODE
ArmController::setHandCartesianPosition(double x, double y, double z, double roll, double pitch, double yaw)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setHandOrientation(double roll, double pitch, double yaw)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setHandParam(string label, double value)
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::setHandParam(string label, string value)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setArmJointPosition(int numJoints, double angles[],double speed[])
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setArmJointTorque(int numJoints, double torque[])
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setArmJointSpeed(int numJoints, double speed[])
{

  return PR2::PR2_ALL_OK;
}


