#include <pr2Controllers/BaseController.h>


BaseController::BaseController()
{

}
    
BaseController::~BaseController( )
{

}

PR2::PR2_ERROR_CODE
BaseController::setCourse(double v , double yaw)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
BaseController::setCourseXY(double vx, double vy)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
BaseController::setTarget(double x,double y, double yaw, double vx, double vy, double yawDot)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
BaseController::setTraj(int num_pts, double x[],double y[], double yaw[], double vx[], double vy[], double yawDot[])
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
BaseController::setHeading(double yaw)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
BaseController::setForce(double fx, double fy)
{

  return PR2::PR2_ALL_OK;
}


PR2::PR2_ERROR_CODE
BaseController::setParam(string label,double value)
{

  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
BaseController::setParam(string label,string value)
{

  return PR2::PR2_ALL_OK;
}
