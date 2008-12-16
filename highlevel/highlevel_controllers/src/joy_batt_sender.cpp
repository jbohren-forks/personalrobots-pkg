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

#include "ros/node.h"
#include "ros/publisher.h"
#include "robot_msgs/BatteryState.h"
#include "joy/Joy.h"

class JoyBattSender : public ros::node
{
  public:
    JoyBattSender() : ros::node("joy_batt_msg")
    {
      param<int>("~stop_button", stop_button_, 7);
      param<int>("~go_button", go_button_, 5);
      param<int>("~deadman_button", deadman_button_, 4);
      robot_msgs::BatteryState bs;
      advertise("bogus_battery_state", bs, &JoyBattSender::sendHeartbeat, 2);
      subscribe("joy", joy_msg_, &JoyBattSender::handleJoyMsg, 2);
    }

    void handleJoyMsg()
    {
      ROS_INFO("%d", joy_msg_.buttons.size());
      if(joy_msg_.buttons[stop_button_] && (joy_msg_.buttons[deadman_button_]))
      {
        // Fake battery message that says we're empty
        robot_msgs::BatteryState s;
        s.energy_remaining = 0.0;
        s.energy_capacity = 1000.0;
        s.power_consumption = -800.0;

        publish("bogus_battery_state", s);

        ROS_INFO("Published bogus battery message");
      }
      else if(joy_msg_.buttons[go_button_] && (joy_msg_.buttons[deadman_button_]))
      {
        // Fake battery message that says we're empty
        robot_msgs::BatteryState s;
        s.energy_remaining = 1000.0;
        s.energy_capacity = 1000.0;
        s.power_consumption = -800.0;

        publish("bogus_battery_state", s);

        ROS_INFO("Published bogus battery message");
      }
    }

    void sendHeartbeat(const ros::PublisherPtr& pub)
    {
        robot_msgs::BatteryState s;
        s.energy_remaining = 1000.0;
        s.energy_capacity = 1000.0;
        s.power_consumption = -800.0;

        publish("bogus_battery_state", s);

        ROS_INFO("Published bogus battery message");
    }

  private:
    joy::Joy joy_msg_;
    int stop_button_;
    int go_button_;
    int deadman_button_;

};

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  JoyBattSender jbs;

  jbs.spin();

  ros::fini();

  return 0;
}
