#include <pr2Controllers/ArmController.h>

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
ArmController::getHandCartesianPositionCmd(double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getHandCartesianPositionAct(double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setHandOrientation(double roll, double pitch, double yaw)
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getHandOrientationCmd(double *roll, double *pitch, double *yaw)
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getHandOrientationAct(double *roll, double *pitch, double *yaw)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setHandParam(string label, double value)
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getHandParam(string label, double *value)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setHandParam(string label, string value)
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getHandParam(string label, string *value)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setArmJointPosition(int numJoints, double angles[],double speed[])
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getArmJointPositionCmd(int *numJoints, double *angles[],double *speed[])
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getArmJointPositionAct(int *numJoints, double *angles[],double *speed[])
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setArmJointTorque(int numJoints, double torque[])
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getArmJointTorqueCmd(int *numJoints, double *torque[])
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getArmJointTorqueAct(int *numJoints, double *torque[])
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setArmJointSpeed(int numJoints, double speed[])
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getArmJointSpeedCmd(int *numJoints, double *speed[])
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getArmJointSpeedAct(int *numJoints, double *speed[])
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setArmJointMaxTorque(int numJoints, double maxTorque[])
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getArmJointMaxTorqueCmd(int *numJoints, double *maxTorque[])
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getArmJointMaxTorqueAct(int *numJoints, double *maxTorque[])
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
ArmController::setArmCamGazePoint(double x, double y, double z)
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getArmCamGazePointCmd(double *x, double *y, double *z)
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getArmCamGazePointAct(double *x, double *y, double *z)
{

  return PR2::PR2_ALL_OK;
}

