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
#include <libKinematics/kinematics.h>
#include <libKinematics/pr2_ik.h>
#include <ros/node.h>
#include <urdf/URDF.h>

using namespace kinematics;

namespace kinematics
{

  #define NUM_JOINTS 7

  class LibKinematicsNode : public ros::Node
  {
    /**
     * @brief Initialize the kinematics library 
     */
    public:  

    LibKinematicsNode(std::string node_name,std::string arm_name);

    ~LibKinematicsNode();

    bool initializeKinematicModel();

    kinematics::arm7DOF *arm_kinematics_;

    bool init();

    double init_solution_theta3_;

    private:

    std::string robot_description_model_;

    std::string service_prefix_;

    std::string arm_name_;

    std::string root_link_name_;

    robot_desc::URDF urdf_model_;

    robot_desc::URDF::Link* findNextLinkInGroup(robot_desc::URDF::Link *link_current, robot_desc::URDF::Group* group);

    double increment_;

  };
}
