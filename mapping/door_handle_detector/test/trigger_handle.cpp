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
 *   * Neither the name of Willow Garage nor the names of its
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
 *
 * $Id$
 *
 *********************************************************************/

#include <ros/node.h>
#include <robot_msgs/Door.h>
#include <door_handle_detector/DoorDetector.h>
#include <door_handle_detector/DoorsDetector.h>

using namespace ros;
using namespace std;

class TriggerHandleDetection
{
  private:
    robot_msgs::Door my_door_;
    door_handle_detector::DoorsDetector::Request req_doorsdetect;
    door_handle_detector::DoorsDetector::Response res_doorsdetect;
    door_handle_detector::DoorDetector::Request req_handledetect;
    door_handle_detector::DoorDetector::Response res_handledetect;
    ros::Node& node_;

  public:
    TriggerHandleDetection (ros::Node& anode) : node_ (anode)
    {
      // initialize my door
      double tmp;
      int tmp2;
      node_.param ("~/door_frame_p1_x", tmp, 1.5);
      my_door_.frame_p1.x = tmp;
      node_.param ("~/door_frame_p1_y", tmp, -0.5);
      my_door_.frame_p1.y = tmp;
      node_.param ("~/door_frame_p2_x", tmp, 1.5);
      my_door_.frame_p2.x = tmp;
      node_.param ("~/door_frame_p2_y", tmp, 0.5);
      my_door_.frame_p2.y = tmp;
      node_.param ("~/door_hinge", tmp2, -1);
      my_door_.hinge = tmp2;
      node_.param ("~/door_rot_dir", tmp2, -1);
      my_door_.rot_dir = tmp2;
      my_door_.header.frame_id = "base_footprint";

      req_doorsdetect.door = my_door_;
      ros::service::call ("doors_detector", req_doorsdetect, res_doorsdetect);
      
      if (res_doorsdetect.doors.size () < 1)
        return;
        
      ROS_INFO ("%d candidate doors returned. Selecting the best one with:\nP1 = [%g, %g, %g]. P2 = [%g, %g, %g]. Width = %g. Height = %g. Weight = %g.", 
                res_doorsdetect.doors.size (),
                res_doorsdetect.doors[0].door_p1.x, res_doorsdetect.doors[0].door_p1.y, res_doorsdetect.doors[0].door_p1.z,
                res_doorsdetect.doors[0].door_p2.x, res_doorsdetect.doors[0].door_p2.y, res_doorsdetect.doors[0].door_p2.z,
                sqrt ( (res_doorsdetect.doors[0].door_p1.x - res_doorsdetect.doors[0].door_p2.x) * 
                       (res_doorsdetect.doors[0].door_p1.x - res_doorsdetect.doors[0].door_p2.x) +
                       (res_doorsdetect.doors[0].door_p1.y - res_doorsdetect.doors[0].door_p2.y) * 
                       (res_doorsdetect.doors[0].door_p1.y - res_doorsdetect.doors[0].door_p2.y) ),
                res_doorsdetect.doors[0].height, res_doorsdetect.doors[0].weight);
                                                                                             
      // A list of doors was detected. Start searching for a handle in the door with the largest weight.
      my_door_ = res_doorsdetect.doors[0];
      my_door_.header.frame_id = "base_footprint";
      req_handledetect.door = my_door_;
      ros::service::call ("handle_detector", req_handledetect, res_handledetect);
    }
};                              // class

// -----------------------------------
//              MAIN
// -----------------------------------

int
  main (int argc, char **argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("detect_handle_test");
  TriggerHandleDetection executive (ros_node);

  return (0);
}
