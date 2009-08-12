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

//! \author Vijay Pradeep

#include "camera_offsetter/generic_offsetter.h"
#include "stereo_msgs/RawStereo.h"

namespace camera_offsetter
{

class StereoOffsetter : public GenericOffsetter
{
public:
  StereoOffsetter()
  {
    if (!nh_.getParam("~cam_name", cam_name_))
      ROS_FATAL("Couldn't find param [~cam_name]");

    raw_stereo_sub_ = nh_.subscribe(cam_name_ + "/raw_stereo", 1, &StereoOffsetter::rawStereoCb, this);

    raw_stereo_pub_ = nh_.advertise<stereo_msgs::RawStereo>(cam_name_ + "_offset/raw_stereo", 1);
  }

  void rawStereoCb(const stereo_msgs::RawStereoConstPtr& msg)
  {
    stereo_msgs::RawStereo next_raw_stereo = *msg;
    next_raw_stereo.header.frame_id = msg->header.frame_id + "_offset";
    raw_stereo_pub_.publish(next_raw_stereo);

    publishTransform(msg->header.stamp, next_raw_stereo.header.frame_id, msg->header.frame_id);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber raw_stereo_sub_;
  ros::Publisher  raw_stereo_pub_;

  std::string cam_name_;
} ;

}

using namespace camera_offsetter;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_offsetter");

  StereoOffsetter offsetter;

  ros::spin();
}
