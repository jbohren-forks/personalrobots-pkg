#ifndef _graspplanner_h_
#define _graspplanner_h_

#include <vector>

namespace grasp_module {

class GraspPoint;
class Object;

/*!
  This class computes GraspPoints from Objects.
 */
class GraspPlanner {
 private:
 public:
	//! Constructor does absolutely nothing for now.
	GraspPlanner(){}

	//! Returns a list of grasp points for a given object
	std::vector<GraspPoint*> *getGraspPoints(const Object *obj, int resolution = 8);
};

} //namespace grasp_module
#endif

