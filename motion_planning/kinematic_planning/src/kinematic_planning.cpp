/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/**

@mainpage

@htmlinclude manifest.html

@b KinematicPlanning is a node capable of planning kinematic paths for
a set of robot models.

**/

#include "ros/node.h"
#include "std_msgs/PointCloudFloat32.h"
#include "std_msgs/KinematicPath.h"
#include "std_srvs/KinematicPlan.h"

using namespace std_msgs;
using namespace std_srvs;

class KinematicPlanning : public ros::node
{
public:

    KinematicPlanning(void) : ros::node("KinematicPlanning")
    {
	advertise_service<KinematicMotionPlan>("plan_kinematic_path", &KinematicPlanning::plan);
	
	subscribe("world_3d_map", cloud, &KinematicPlanning::pointCloudCallback);
	
    }
    
    void pointCloudCallback(void)
    {
	printf("received %d points\n", cloud.pts_size);
    }
    
    bool plan(KinematicMotionPlan::request &req, KinematicMotionPlan::response &res)
    {
	
    }
    
private:
    
    PointCloudFloat32 cloud;
    
};


int main(int argc, char **argv)
{  
    ros::init(argc, argv);

    KinematicPlanning planner;
    planner.spin();
    planner.shutdown();
    
    return 0;    
}
