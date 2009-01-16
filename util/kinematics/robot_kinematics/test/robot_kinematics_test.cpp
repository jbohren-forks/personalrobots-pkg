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
#include <sys/time.h>
#include <unistd.h>
#include <ros/node.h> // roscpp

using namespace KDL;
using namespace std;
using namespace robot_kinematics;

int main( int argc, char** argv )
{
/*
  char *c_filename = getenv("ROS_PACKAGE_PATH");
  std::stringstream filename;
  filename << c_filename << "/robot_descriptions/wg_robot_description/pr2/pr2.xml" ;
  RobotKinematics pr2_kin;
  pr2_kin.loadXML(filename.str());
  SerialChain *left_arm = pr2_kin.getSerialChain("leftArm");
*/
  struct timeval t0, t1;

  //Initialize ROS
  ros::init(argc, argv);

  ros::Node test_kin("test_kin");

  std::string pr2Content;
  test_kin.get_param("robotdesc/pr2",pr2Content);

  RobotKinematics pr2_kin;
  pr2_kin.loadString(pr2Content.c_str());  // parse the big pr2.xml string from ros
  SerialChain *left_arm = pr2_kin.getSerialChain("left_arm");

  SerialChain *right_arm = pr2_kin.getSerialChain("right_arm");

  assert(left_arm);
  assert(right_arm);
  
  JntArray pr2_config = JntArray(left_arm->num_joints_);
  JntArray pr2_config_r = JntArray(right_arm->num_joints_);


  pr2_config(0) = 0.0, pr2_config(1) = -0, pr2_config(2)=0.0, pr2_config(3)=0.0;
  pr2_config(4) = 0.0, pr2_config(5) = deg2rad*0, pr2_config(6) = deg2rad*0.0;
  cout << "Config of the arm:" << pr2_config << endl;

  pr2_config_r(0) = 0.0, pr2_config_r(1) = -0, pr2_config_r(2)=0.0, pr2_config_r(3)=0.25;
  pr2_config_r(4) = 0.3, pr2_config_r(5) = deg2rad*0, pr2_config_r(6) = deg2rad*0.0;
  cout << "Config of the arm:" << pr2_config_r << endl;

  Frame f;
  if (left_arm->computeFK(pr2_config,f))
    cout<<"End effector transformation:"<<f<<endl;

  if (right_arm->computeFK(pr2_config_r,f))
    cout<<"End effector transformation Right:"<<f<<endl;

  pr2_config(0) = 0.1, pr2_config(1) = -1, pr2_config(2)=0.3, pr2_config(3)=0.3;
  pr2_config(4) = 0.2, pr2_config(5) = 0.5, pr2_config(6) = 0.0;
  cout<<"Config of the arm:"<<pr2_config<<endl;

  if (left_arm->computeFK(pr2_config,f))
    cout<<"End effector transformation:"<<f<<endl;
  else
    cout<<"Could not compute Fwd Kin."<<endl;

  for (int i=0; i<7; i++)
  {
  if (left_arm->computeFK(pr2_config,f,i))
     cout<<"Link transformation: "<< i << endl << f<<endl;
  else
     cout<<"Could not compute Fwd Kin.for link: "<< i << endl;
  }
  (*left_arm->q_IK_guess)(0) = 0.1, (*left_arm->q_IK_guess)(1) = 0.0, (*left_arm->q_IK_guess)(2) = 0.0, (*left_arm->q_IK_guess)(3) = 0.0;
  (*left_arm->q_IK_guess)(4) = 0.0, (*left_arm->q_IK_guess)(5) = 0.0, (*left_arm->q_IK_guess)(6) = 0.0;

  gettimeofday(&t0,NULL);
  if (left_arm->computeIK(f) == true)
  {
    gettimeofday(&t1, NULL);
    cout<<"IK result:"<< *left_arm->q_IK_result<<endl;
    double time_taken = (t1.tv_sec*1000000+t1.tv_usec - (t0.tv_sec*1000000+t0.tv_usec))/1000.;
    printf("Time taken: %f ms\n", time_taken);
  }
  else
    cout<<"Could not compute Inv Kin."<<endl;

  Frame f_ik;   //------ checking that IK returned a valid soln -----
  if (left_arm->computeFK(*(left_arm->q_IK_result),f_ik))
    cout<<"End effector after IK:"<<f_ik<<endl;
  else
    cout<<"Could not compute Fwd Kin. (After IK)"<<endl;

  ros::fini();
}
