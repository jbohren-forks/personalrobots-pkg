/*
 * Copyright (c) 2008, Maxim Likhachev
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
 *     * Neither the name of the University of Pennsylvania nor the names of its
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
#ifndef __PLANPATH_ROBARM_H_
#define __PLANPATH_ROBARM_H_

#include <iostream>
#include "../../src/headers.h"
#include <pr2_mechanism_controllers/JointTraj.h>
#include <pr2_mechanism_controllers/JointTrajPoint.h>
#include <ros/node.h>
#include <std_msgs/Point.h>
#include <robarm3d/PlanPathSrv.h>

namespace plan_path_node
{
#define VERBOSE 1
#define MAX_RUNTIME 60.0

  class PlanPathNode : public ros::node
  {
    
    public:

    PlanPathNode(std::string node_name);

    ~PlanPathNode();
    
    int planrobarmROS(const pr2_mechanism_controllers::JointTrajPoint &start, const std_msgs::Point &goal, pr2_mechanism_controllers::JointTraj &armpath);

    std::string filename_;

    bool planPath(robarm3d::PlanPathSrv::request &req, robarm3d::PlanPathSrv::response &resp);

  };
}

#endif
