/*********************************************************************
*
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
*
* Author: Sachin Chitta
*********************************************************************/

#include <manipulation_msgs/JointTrajPoint.h>
#include <manipulation_msgs/JointTraj.h>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

#include <angles/angles.h>
#include <pr2_ik/pr2_ik_solver.h>

//Pose command for the ik node
#include <manipulation_srvs/IKService.h>
#include <manipulation_srvs/IKQuery.h>

#include <manipulation_msgs/IKRequest.h>

namespace pr2_ik
{
  class PR2IKNode
  {
    public:

    PR2IKNode();

    ~PR2IKNode();

    bool init();

    private:

    ros::NodeHandle node_handle_;

    ros::ServiceServer ik_service_;

    ros::ServiceServer ik_query_;

    PR2IKSolver pr2_ik_solver_;

    tf::TransformListener tf_;

    std::string control_topic_name_;

    KDL::Frame pose_desired_;

    std::string root_name_;

    int dimension_;

    bool free_angle_constraint_;

    bool ikService(manipulation_srvs::IKService::Request &request, manipulation_srvs::IKService::Response &response);

    bool ikQuery(manipulation_srvs::IKQuery::Request &request, manipulation_srvs::IKQuery::Response &response);

    int getJointIndex(const std::string &name);

    bool checkJointNames(const manipulation_srvs::IKService::Request &request);

    std::string ik_service_name_;

    std::string ik_query_name_;
  };
}
