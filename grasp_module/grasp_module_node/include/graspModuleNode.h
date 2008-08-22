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

#ifndef _graspmodulenode_h_
#define _graspmodulenode_h_

#include <ros/node.h>

#include "grasp_module/object_detector_srv.h"
#include "grasp_module/grasp_planner_srv.h"

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
	bool graspPlan(grasp_planner_srv::request &req,
		       grasp_planner_srv::response &res);
}; 

} //namespace grasp_module
#endif
