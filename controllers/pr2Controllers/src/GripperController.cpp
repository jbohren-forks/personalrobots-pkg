#include <controller/GripperController.h>

GripperController::GripperController()
{
	return PR2::PR2_ALL_OK;
}
    
GripperController::~GripperController( )
{
	return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
GripperController::setGap(double distance)
{
	return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
GripperController::setForce(double force)
{
	return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
GripperController::close()
{
	return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
GripperController::open()
{
	return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
GripperController::setParam(string label,double value);
{
	return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE
GripperController::setParam(string label,string value);
{
	return PR2::PR2_ALL_OK;
}




