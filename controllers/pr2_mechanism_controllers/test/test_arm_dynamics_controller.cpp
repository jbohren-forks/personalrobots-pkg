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

#include <libTF/libTF.h>
#include <ros/node.h>
#include <pr2_mechanism_controllers/SetJointCmd.h>
#include <std_msgs/TransformWithRateStamped.h>
#include <std_msgs/BaseVel.h>
#include <std_msgs/RobotBase2DOdom.h>

static int done = 0;

void finalize(int donecare)
{
  done = 1;
}

/* class test_run_arm
{
   public:

      test_run_arm(){}; 

      ~test_run_arm(){};

      robot_msgs::MechanismState state;

      void jointMsgReceived()
      {

      };
};
*/


int main( int argc, char** argv )
{
  /*********** Initialize ROS  ****************/
  ros::init(argc,argv);
  ros::node *node = new ros::node("test_arm_dynamics_controller"); 

  signal(SIGINT,  finalize);
  signal(SIGQUIT, finalize);
  signal(SIGTERM, finalize);

  /*********** Start moving the arm ************/
  pr2_mechanism_controllers::SetJointCmd::request  req;
  pr2_mechanism_controllers::SetJointCmd::response resp;

  int num_joints = 7;

  req.set_positions_size(num_joints);
  req.set_velocity_size(num_joints);
  req.set_acc_size(num_joints);

  resp.set_positions_size(num_joints);
  resp.set_velocity_size(num_joints);
  resp.set_acc_size(num_joints);

  for(int i=0; i < num_joints; i++)
  {
     req.positions[i] = 0.0;
     req.velocity[i]  = 0.0;
     req.acc[i]       = 0.0;
  }

  sleep(1);

//  node->subscribe("mechanism_state",tra.state,&test_run_arm::jointMsgReceived,&tra,10);
//  if(ros::service::call("arm_dynamics_controller/set_command_array",req,resp))
//  {
//  }

}
