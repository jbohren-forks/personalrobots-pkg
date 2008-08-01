//Software License Agreement (BSD License)
//
//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel_io.hpp>
#include <kdl/utilities/utility.h>

#include <stdio.h>
#include <iostream>
#include <math.h>

#include <urdf/URDF.h>
#include <libTF/Pose3D.h>
#include <robot_kinematics/serial_chain.h>

namespace robot_kinematics
{
  class RobotKinematics
  {
    public:

    RobotKinematics();

    ~RobotKinematics();

    void loadXML(std::string filename);

    SerialChain* getSerialChain(std::string name) const;

    private:

    SerialChain *chains_;

    void createChain(robot_desc::URDF::Group* group);

    void cross(const double p1[], const double p2[], double p3[]);

    void getKDLJointInXMLFrame(robot_desc::URDF::Link *link, KDL::Frame &frame);

    double getAngleBetweenVectors(double p1[],double p2[]);

    KDL::Frame *robot_kdl_frame_;

    KDL::Frame convertNewmatMatrixToKDL(NEWMAT::Matrix m);

    KDL::Frame getKDLNextJointFrame(robot_desc::URDF::Link *link, robot_desc::URDF::Link *link_plus_one);

    std::vector<robot_desc::URDF::Group*> groups_;

    NEWMAT::Matrix getKDLJointInXMLFrame(robot_desc::URDF::Link *link);

    robot_desc::URDF::Link* findNextLinkInGroup(robot_desc::URDF::Link *link_current, robot_desc::URDF::Group* group);
    
    int num_chains_;

    int chain_counter_;

    protected:

    std::map<std::string, SerialChain*> serial_chain_map_;

  };
}
#endif


