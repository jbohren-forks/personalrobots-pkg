#include "graspModuleNode.h"

#include <stdio.h>
#include <vector>

#include "smartScan.h"
#include "object.h"
#include "objectDetector.h"

namespace grasp_module {

GraspModuleNode::GraspModuleNode() : ros::node("grasp_module_node")
{
	advertise_service("object_detector_srv",&GraspModuleNode::objDetect);
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

	grasp_module::ObjectDetector detector;
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

} //namespace grasp_module
