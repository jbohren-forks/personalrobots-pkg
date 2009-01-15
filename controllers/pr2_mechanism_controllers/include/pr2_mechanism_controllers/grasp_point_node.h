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

#pragma once

#include <libKinematics/kinematics.h>
#include <libKinematics/pr2_ik.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <LinearMath/btTransform.h>

#include <pr2_msgs/GraspPoint.h>
#include <pr2_mechanism_controllers/GraspPointSrv.h>

#include <ros/node.h>

#include <urdf/URDF.h>


namespace grasp_point_node 
{

  #define NUM_JOINTS 7

  class GraspPointNode : public ros::node
  {
    /**
     * @brief Generate a arm trajectory to get to a grasp point
     * access
     */
    public:  

    GraspPointNode(std::string node_name);

    ~GraspPointNode();
  
    void init();

    private:

    double increment_;

    tf::TransformListener tf_; /**< Used to do transforms */

    kinematics::arm7DOF *arm_kinematics_;

    double grasp_standoff_distance_;

    double init_solution_theta3_;

    std::string robot_description_model_;

    std::string service_prefix_;

    std::string arm_name_;

    std::string root_link_name_;

    robot_desc::URDF urdf_model_;

    robot_desc::URDF::Link* findNextLinkInGroup(robot_desc::URDF::Link *link_current, robot_desc::URDF::Group* group);

    int initializeKinematicModel();

    bool computeIKSolution(const tf::Pose &pose, std::vector<double> &soln);

    tf::Transform calculateIntermediatePoint(tf::Transform grasp_point);  //Assumption is that the grasp point is located along the X axis of the grasp point transform

    bool processGraspPointService(pr2_mechanism_controllers::GraspPointSrv::request &req, pr2_mechanism_controllers::GraspPointSrv::response &resp);

    bool chooseSoln(const std::vector<std::vector<double> > &ik_solns, std::vector<double> &solution);

  };
}
