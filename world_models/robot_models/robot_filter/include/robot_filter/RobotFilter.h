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

/** \author Ioan Sucan */


#include <collision_space/util.h>
#include <robot_model/knode.h>


#ifndef ROBOT_FILTER_H
#define ROBOT_FILTER_H

namespace robot_filter {

  /**
     @b RobotFilter A class that filters laser hits on the robot.
     
     <hr>
     
     @section topic ROS topics
     
     Subscribes to (name/type):
     - None, but those used by robot_model::NodeRobotModel.
     
     Publishes to (name/type):
     - None
     
     <hr>
     
     @section services ROS services
     
     Uses (name/type):
     - None
     
     Provides (name/type):
     - None
     
     <hr>
     
  **/
  class RobotFilter {
  private:
    struct RobotPart {
      collision_space::bodies::Shape        *body;
      planning_models::KinematicModel::Link *link;	
    };
    std::vector<RobotPart>                   m_selfSeeParts;
    double                                   m_bodyPartScale;
    ros::node                               *m_node;
    robot_model::NodeRobotModel             *m_model;
    bool                                     m_verbose;


    void addSelfSeeBodies(void);
  public:
    void loadRobotDescription();
    void waitForState();
    std_msgs::PointCloud* filter(const std_msgs::PointCloud &cloud);

    RobotFilter(ros::node* node, std::string robot_model_name, bool verbose, double bodyPartScale);
    ~RobotFilter();
  };
  
}

#endif

