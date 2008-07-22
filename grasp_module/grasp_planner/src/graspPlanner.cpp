#include "graspPlanner.h"

#include <libTF/libTF.h>

#include "object.h"
#include "graspPoint.h"

#include <algorithm> //for sort

namespace grasp_module{

/*!  Given an object, this function will return a vector of the
  best grasp points it can compute for it. It is the responsability of
  the caller to free the returned memory.

  The grasp points returned are as follows: The origin of the gripper
  is placed at the centroid of the object. The "up" direction of the
  gripper is aligned with the dominant axis (meaning the jaws are on
  either side of the dominant axis). This is guaranteed for all grasps
  returned.

  A number of grasps are then obtained by rotating the gripper around
  the dominant axis of the object. The starting point is the grasp
  where the approach direction is along the second dominant axis of
  the object - this is considered the most desirable grasp. Starting
  from this position, more grasps are obtained by rotating around the
  dominant axis.

  \param obj - target object

  \param resolution - number of grasps returned. If \a resolution = 1
  the function only returns the most desirable grasp, with the
  approach direction along the second dominant axis of the object. If
  \a resolution > 1 it will sample along the "equator" of the object
  and return exactly \resolution grasps, spaced equally apart around
  the object.

  All the grasps have a quality value between 0 and 1 set depending on
  angular distance between approach direction and second dominant axis
  of the object. Higher values mean better grasps, and the most
  desirable grasp with quality = 1 is again considered to be the one
  along the second dominant axis.

  Returned vector is sorted in ascending quality of the grasps.
*/

std::vector<GraspPoint*> *GraspPlanner::getGraspPoints(const Object *obj, int resolution) 
{
	std::vector<GraspPoint*> *grasps = new std::vector<GraspPoint*>;
	if (resolution < 1) resolution = 1;

	libTF::TFPoint c = obj->getCentroid();
	libTF::TFVector a1,a2,a3;
	//axes should already be orthonormal
	obj->getAxes(a1,a2,a3);

	libTF::TFVector app,up;

	//the only fixed constraint is that we want the jaws to be on either side
	//of the dominant axis. This should never change
	up = a1;

	//for starters let's approach along the second axis. This should be the most
	//desirable grasp, with the jaws along the least dominant axis
	app  = a2;

	//this is the base grasp that all others are computed relative to
	GraspPoint baseGrasp(c,app,up);

	NEWMAT::Matrix B;
	baseGrasp.getTran(&B);

	//std::cerr << "B:" << std::endl << B;

	libTF::TransformReference baseTran;
	baseTran.setWithMatrix( GraspPoint::graspFrame, GraspPoint::baseFrame, B, 0);

	float angleStep = 2.0 * M_PI / resolution;
	//get other grasp points by rotating the base grasp around its up (or z) direction
	for (int i=0; i<resolution; i++) {
		
		//NEWMAT::Matrix M0 = baseTran.getMatrix(GraspPoint::graspFrame, GraspPoint::baseFrame, 0);
		//std::cerr << "M0:" << std::endl << M0 << std::endl;

		baseTran.setWithEulers( GraspPoint::newFrame, GraspPoint::graspFrame, 
					0, 0, 0,
					i * angleStep, 0, 0,
					0 );
		
		//NEWMAT::Matrix M1 = baseTran.getMatrix(GraspPoint::baseFrame, GraspPoint::graspFrame, 0);
		//std::cerr << "M1:" << std::endl << M1 << std::endl;

		//NEWMAT::Matrix M2 = baseTran.getMatrix(GraspPoint::graspFrame, GraspPoint::newFrame, 0);
		//std::cerr << "M2:" << std::endl << M2 << std::endl;

		NEWMAT::Matrix M = baseTran.getMatrix(GraspPoint::baseFrame, GraspPoint::newFrame, 0);
		//std::cerr << "M:" << std::endl << M << std::endl;

		GraspPoint *newGrasp = new GraspPoint(&M);
		newGrasp->setQuality( fabs( cos(i * angleStep) ) );
		grasps->push_back(newGrasp);
	}
	
	std::sort(grasps->begin(), grasps->end(), sortGraspPredicate);
	return grasps;
}


} //namespace grasp_module
