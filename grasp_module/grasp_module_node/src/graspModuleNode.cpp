#include "graspModuleNode.h"

#include <stdio.h>
#include <vector>

#include "smartScan.h"
#include "object.h"
#include "objectDetector.h"
#include "graspPoint.h"
#include "graspPlanner.h"

namespace grasp_module {

GraspModuleNode::GraspModuleNode() : ros::node("grasp_module_node")
{
	advertise_service("object_detector_srv",&GraspModuleNode::objDetect);
	advertise_service("grasp_planner_srv",&GraspModuleNode::graspPlan);
	fprintf(stderr,"GraspModuleNode created and subscribed.\n");
}

GraspModuleNode::~GraspModuleNode()
{
	fprintf(stderr,"GraspModuleNode destructed\n");
}

/*! This function is responsible for providing the object_detector_srv
    service. It takes in a request, which consists of a cloud and
    parameters, creates an ObjectDetector and calls it, then takes the
    response and populates the reply which is a vector of object_msg.
 */
bool GraspModuleNode::objDetect(object_detector_srv::request &req,
				object_detector_srv::response &res)
{
	SmartScan *scan = new SmartScan();
	scan->setFromRosCloud(req.cloud);

	ObjectDetector detector;
	detector.setParamComponents(req.connectedThreshold, req.minComponentSize);
	detector.setParamGrazing(req.grazingThreshold, req.grazingRadius, req.grazingNbrs);
	detector.setParamPlane(req.planeRemovalThreshold, req.planeFitThreshold);

	std::vector<Object*> *objects = detector.getObjects(scan);

	std::vector<object_msg> objmsg;

	for(int i=0; i<(int)objects->size(); i++) {
		objmsg.push_back( (*objects)[i]->getMsg() );
		delete (*objects)[i];
	}
	res.set_objects_vec(objmsg);

	delete objects;
	delete scan;

	return true;
}

bool GraspModuleNode::graspPlan(grasp_planner_srv::request &req,
				grasp_planner_srv::response &res)
{

	GraspPlanner planner;

	Object objIn;
	objIn.setFromMsg(req.obj);

	std::vector<GraspPoint*> *grasps = planner.getGraspPoints(&objIn, req.resolution);

	std::vector<graspPoint_msg> graspmsg;

	for (int i=0; i<(int)grasps->size(); i++) {
		graspmsg.push_back( (*grasps)[i]->getMsg() );
		delete (*grasps)[i];
	}
	res.set_graspPoints_vec(graspmsg);

	delete grasps;

	return true;
}


} //namespace grasp_module
