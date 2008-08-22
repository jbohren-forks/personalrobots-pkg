/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* author: Matei Ciocarlie */

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
