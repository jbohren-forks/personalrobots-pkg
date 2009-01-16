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


#include <ros/node.h>
#include <ros/time.h>
#include <std_msgs/PoseWithRatesStamped.h>
#include <std_msgs/BaseVel.h>
#include "control_toolbox/base_position_pid.h"
#include "tf/transform_datatypes.h"

using namespace std ;

namespace fake_localization
{

class GroundTruthController : public ros::Node
{
public:
  std_msgs::PoseWithRatesStamped  m_ground_truth_ ;                     //!< Message on which we receive ground truth info
  control_toolbox::BasePositionPid base_position_pid_ ;                 //!< Does the PID math for controlling the robot
  tf::Vector3 xyt_target_ ;                                             //!< The ground truth pose we want to acheive
  bool first_time_ ;
  ros::Time prev_time_ ;
  std_msgs::Point cmd_ ;
  
  GroundTruthController() : ros::Node("ground_truth_controller")
  {
    xyt_target_.setX(0) ;
    xyt_target_.setY(0) ;
    xyt_target_.setZ(0) ;
  }
  
  ~GroundTruthController() {  }

  bool initController()
  {
    printf("initController()\n") ;
    
    string xml_config ;
    param("~xml_config", xml_config, string("")) ;
    subscribe("base_pose_ground_truth", m_ground_truth_, &GroundTruthController::updateControl, 1) ;
    subscribe("~set_cmd", cmd_, &GroundTruthController::setCommandCallback, this, 10) ;

    advertise<std_msgs::BaseVel>("cmd_vel", 1) ;
    
    // Initialize the BasePositionPid util via xml
    TiXmlDocument xml ;
    xml.Parse(xml_config.c_str()) ;
    TiXmlElement *config = xml.RootElement() ;
    if (config == NULL)
    {
      ROS_WARN("Error opening XML file") ;
      return false ;
    }
    bool result = base_position_pid_.initXml(config) ;
    if (!result)
    {
      ROS_WARN("Error loading BasePositionPid xml") ;
      return false ;      
    }
    
    first_time_ = true ;

    return true ;
  }
  
  /**
   * Grabs the current ground truth message and computes the next command, and published the command
   */
  void updateControl()
  {
    //printf("Got ground truth\n") ;
    
    if (first_time_)
    {
      prev_time_ = m_ground_truth_.header.stamp ;
      first_time_ = false ;
      return ;
    }

    ros::Time cur_time = m_ground_truth_.header.stamp ;
    ros::Duration time_elapsed = cur_time - prev_time_ ;
    if (time_elapsed.toSec() < .05)
      return ;
    
    tf::Transform ground_truth_pose ;
    
    tf::PoseMsgToTF(m_ground_truth_.pos, ground_truth_pose) ;

    //! \todo Compute yaw angle in a more stable way
    double yaw,pitch,roll ;
    ground_truth_pose.getBasis().getEulerZYX(yaw, pitch, roll) ;
    
    tf::Vector3 xyt_current(ground_truth_pose.getOrigin().x(), ground_truth_pose.getOrigin().y(), yaw) ;

    
    // Determine next velocity to command
    tf::Vector3 vel_cmd ;
    vel_cmd = base_position_pid_.updateControl(xyt_target_, xyt_current, time_elapsed.toSec()) ;

    std_msgs::BaseVel base_vel ;
    base_vel.vx = vel_cmd.x() ;
    base_vel.vy = vel_cmd.y() ;
    base_vel.vw = vel_cmd.z() ;
    
    publish("cmd_vel", base_vel) ;
    
    prev_time_ = cur_time ;
  }
  
  void setCommandCallback()
  {
    setCommand(cmd_.x, cmd_.y, cmd_.z) ;
  }

  void setCommand(double x, double y, double w)
  {
    ROS_INFO("BasePositionControllerNode:: Odom Frame Position Command: %f %f %f\n", x, y, w) ;
    
    //! \todo Mutex this data type
    xyt_target_.setX(x) ;
    xyt_target_.setY(y) ;
    xyt_target_.setZ(w) ;
  }

} ;

}

using namespace fake_localization ;

int main(int argc, char** argv)
{
  ros::init(argc, argv);

  GroundTruthController controller ;
  controller.initController() ;

  controller.spin() ;

  ros::fini() ;

  return 0 ;
  
  
  
}
