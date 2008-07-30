#include "object.h"

namespace grasp_module {

object_msg Object::getMsg() const
{
	object_msg om;
       
	om.centroid.x = mCentroid.x; om.centroid.y = mCentroid.y; om.centroid.z = mCentroid.z;

	om.axis1.x = mA1.x; om.axis1.y = mA1.y; om.axis1.z = mA1.z;
	om.axis2.x = mA2.x; om.axis2.y = mA2.y; om.axis2.z = mA2.z;
	om.axis3.x = mA3.x; om.axis3.y = mA3.y; om.axis3.z = mA3.z;

	return om;
}

void Object::setFromMsg(const object_msg &om)
{
	mCentroid.x = om.centroid.x; mCentroid.y = om.centroid.y; mCentroid.z = om.centroid.z;

	mA1.x = om.axis1.x; mA1.y = om.axis1.y; mA1.z = om.axis1.z;
	mA2.x = om.axis2.x; mA2.y = om.axis2.y; mA2.z = om.axis2.z;
	mA3.x = om.axis3.x; mA3.y = om.axis3.y; mA3.z = om.axis3.z;
}

} //namespace grasp_module
