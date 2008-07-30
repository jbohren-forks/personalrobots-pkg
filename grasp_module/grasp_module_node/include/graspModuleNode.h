#ifndef _graspmodulenode_h_
#define _graspmodulenode_h_

#include <ros/node.h>

#include "grasp_module/object_detector_srv.h"

/**
   @mainpage 

   @b This is a thin ROS node that wraps around the services offered
   by the other components of the grasp_module (which for now are the
   object_detector and grasp_planner). See the Wiki page of the Zeroth
   order Grasp Module for details on the operations of these
   components.

   Conceptually, the object_detector and grasp_planner could each have
   their own wrapper nodes, but for now to keep things simple this
   node will offer the services of both.
 **/

namespace grasp_module {

/*!  This class provides the services of the object_detector and
  grasp_planner in ROS service format.
*/
class GraspModuleNode : public ros::node
{
 private:

 public:
	GraspModuleNode();
	~GraspModuleNode();

	bool objDetect(object_detector_srv::request &req,
		       object_detector_srv::response &res);
}; 

} //namespace grasp_module
#endif
