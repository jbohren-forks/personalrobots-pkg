/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_calibration_actions/robot_pixels_capture.h>
#include <pr2_calibration_actions/CaptureRobotPixelsAction.h>

using namespace pr2_calibration_actions;

class CaptureRobotPixelsActionServer
{
public:
  CaptureRobotPixelsActionServer(ros::NodeHandle nh, std::string action_name) :
    as_(nh, action_name)
  {
    as_.registerPreemptCallback( boost::bind(&CaptureRobotPixelsActionServer::preemptCallback, this));
    as_.registerGoalCallback( boost::bind(&CaptureRobotPixelsActionServer::goalCallback, this));
  }

  void preemptCallback()
  {
    boost::mutex::scoped_lock lock(capture_mutex_);
    ROS_FATAL_COND(!capture_, "Trying to preempt a null capture_");
    capture_->shutdown();
    as_.setPreempted();
  }

  void goalCallback()
  {
    ROS_INFO("Got a Goal. Going to start listening to data");
    boost::mutex::scoped_lock lock(capture_mutex_);
    if ( as_.isActive() )
    {
      ROS_FATAL_COND(!capture_, "Trying to shutdown a null capture_");
      capture_->shutdown();
      as_.setPreempted();
    }
    capture_.reset();

    cur_goal_ = as_.acceptNewGoal();

    capture_.reset(new RobotPixelsCapture(cur_goal_->config,
                                          boost::bind(&CaptureRobotPixelsActionServer::completionCallback, this, _1),
                                          boost::bind(&CaptureRobotPixelsActionServer::sendFeedback, this, _1)));
  }

  void completionCallback(const RobotPixelsResult& result)
  {
    capture_->shutdown();
    CaptureRobotPixelsResult full_result;
    full_result.result = result;
    as_.setSucceeded(full_result);
  }

  void sendFeedback(const RobotPixelsFeedback& feedback)
  {
    CaptureRobotPixelsFeedback full_feedback;
    full_feedback.feedback = feedback;
    //as_.publishFeedback(full_feedback);
  }


private:
  actionlib::SimpleActionServer<pr2_calibration_actions::CaptureRobotPixelsAction> as_;

  boost::mutex capture_mutex_;
  boost::shared_ptr<RobotPixelsCapture> capture_;
  CaptureRobotPixelsGoalConstPtr cur_goal_;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "capture_robot_pixels_action_node");

  ros::NodeHandle nh;

  CaptureRobotPixelsActionServer act(nh, "capture_robot_pixels");

  ros::spin();
}
