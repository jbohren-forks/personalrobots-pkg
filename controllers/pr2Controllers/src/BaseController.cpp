#include <controller/BaseController.h>


BaseController::BaseController()
{

}
    
BaseController::~BaseController( )
{

}

PR2::PR2_ERROR_CODE
BaseController::setCourse(double v , double yaw)
{

}

PR2::PR2_ERROR_CODE
BaseController::setCourseXY(double vx, double vy)
{

}

PR2::PR2_ERROR_CODE
BaseController::setTarget(double x,double y, double yaw, double vx, double vy, double yawDot)
{

}

PR2::PR2_ERROR_CODE
BaseController::setTraj(int num_pts, double x[],double y[], double yaw[], double vx[], double vy[], double yawDot[])
{

}

PR2::PR2_ERROR_CODE
BaseController::setHeading(double yaw)
{

}

PR2::PR2_ERROR_CODE
BaseController::setForce(double fx, double fy)
{

}


PR2::PR2_ERROR_CODE
BaseController::setParam(string label,double value)
{


}
