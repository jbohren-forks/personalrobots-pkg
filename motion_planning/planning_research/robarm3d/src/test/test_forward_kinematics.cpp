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


#include <robot_kinematics/robot_kinematics.h>
#include <unistd.h>
#include <ros/node.h> // roscpp

using namespace KDL;
using namespace std;
using namespace robot_kinematics;

int main( int argc, char** argv )
{

//   char *c_filename = getenv("ROS_PACKAGE_PATH");
//   std::stringstream filename;
//   filename << c_filename << "/robot_descriptions/wg_robot_description/pr2/pr2.xml" ;
//   RobotKinematics pr2_kin;
//   pr2_kin.loadXML(filename.str());
//   SerialChain *left_arm = pr2_kin.getSerialChain("left_arm");


  //Initialize ROS
  ros::init(argc, argv);

  ros::Node test_forward_kin("test_forward_kin");

  std::string pr2Content;
  test_forward_kin.get_param("robotdesc/pr2",pr2Content);

  RobotKinematics pr2_kin;
  pr2_kin.loadString(pr2Content.c_str());  // parse the big pr2.xml string from ros
  SerialChain *left_arm = pr2_kin.getSerialChain("left_arm");
  assert(left_arm);
  
  JntArray pr2_config = JntArray(left_arm->num_joints_);

  pr2_config(0) = 0;
  pr2_config(1) = 0.0;
  pr2_config(2)= 0;
  pr2_config(3)= 0;
  pr2_config(4) = 0.0;
  pr2_config(5) = 0.0;
  pr2_config(6) = 0;
  cout << "Config of the arm:" << pr2_config << endl;

  Frame f;
  if (left_arm->computeFK(pr2_config,f))
    cout<<"End effector transformation:"<<f<<endl;

  ros::fini();
}
