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
 *********************************************************************/

#include <ros/node.h>
#include <robot_msgs/Door.h>
#include <checkerboard_detector/ObjectDetection.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

using namespace checkerboard_detector;
using namespace tf;

class DoorDetectionTestNode : public ros::Node
{
  public:
    std::string listen_topic_, publish_topic_;
    checkerboard_detector::ObjectDetection checkerboard_msg_;
    double door_width_;
    double door_checkerboard_x_offset_, door_checkerboard_z_offset_, checkerboard_handle_z_offset_, checkerboard_handle_x_offset_;

    DoorDetectionTestNode(std::string node_name):ros::Node(node_name)
    {
      this->param<std::string>("door_detection_test_node/listen_topic",listen_topic_,"/checkerdetector/ObjectDetection");
      this->param<double>("door_detection_test_node/door_width",door_width_,0.9);
      this->param<double>("door_detection_test_node/door_checkerboard_x_offset",door_checkerboard_x_offset_,0.9);
      this->param<double>("door_detection_test_node/door_checkerboard_z_offset",door_checkerboard_z_offset_,0.9);

      this->param<double>("door_detection_test_node/checkerboard_handle_z_offset",checkerboard_handle_z_offset_,0.9);
      this->param<double>("door_detection_test_node/checkerboard_handle_x_offset",checkerboard_handle_x_offset_,0.9);

      subscribe(listen_topic_, checkerboard_msg_,  &DoorDetectionTestNode::doorCallback,1);
      advertise<robot_msgs::Door>("door_location",1);
    }

    ~DoorDetectionTestNode()
    {
      unadvertise(publish_topic_);
      unsubscribe(listen_topic_);
    }

    void doorCallback()
    {
      robot_msgs::Door door_msg;

      door_msg.header.stamp    = checkerboard_msg_.header.stamp;
      door_msg.header.frame_id = checkerboard_msg_.header.frame_id;

      tf::Pose door_pose;
      tf::Vector3 frame_p1, frame_p2, door_p1, door_p2, handle;

      if(checkerboard_msg_.get_objects_size() > 0)
      {
        tf::PoseMsgToTF(checkerboard_msg_.objects[0].pose,door_pose);
        door_p1 = door_pose*Vector3(-door_checkerboard_x_offset_,door_checkerboard_z_offset_,0.0);
        door_p2 = door_pose*Vector3(door_width_ - door_checkerboard_x_offset_,door_checkerboard_z_offset_,0.0);
        frame_p1 = door_p1;
        frame_p2 = door_p2;
        handle = door_pose*Vector3(checkerboard_handle_x_offset_,checkerboard_handle_z_offset_,0.0);

        PointTFToMsg(door_p1,door_msg.door_p1);
        PointTFToMsg(door_p2,door_msg.door_p2);

        PointTFToMsg(frame_p1,door_msg.frame_p1);
        PointTFToMsg(frame_p2,door_msg.frame_p2);

        PointTFToMsg(handle,door_msg.handle);
        publish(publish_topic_,door_msg);
      }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  DoorDetectionTestNode node("test_door_detection");

  try {
    node.spin();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }
  
  return(0);
}
