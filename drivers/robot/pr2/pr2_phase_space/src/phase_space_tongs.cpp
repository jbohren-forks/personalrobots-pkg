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

/****
 * This node is to be used with the insturmented teleoperation tongs. Once
 * a rigid body for both sides of the tongs is extracted, this node determines
 * the angle between both sides, and the average pose of the tongs.
 */

#include <string>

#include "ros/node.h"

// Messages
#include "std_msgs/Float64.h"
#include "robot_msgs/PoseStamped.h"
#include "mocap_msgs/MocapSnapshot.h"
#include "mocap_msgs/MocapMarker.h"
#include "mocap_msgs/MocapBody.h"

#include "tf/transform_datatypes.h"

using namespace std ;

namespace pr2_phase_space
{

class PhaseSpaceTongs
{
public :

  ros::Node* node_ ;
  int left_id_ ;
  int right_id_ ;
  mocap_msgs::MocapSnapshot snapshot_ ;
  string frame_id_ ;
  unsigned int prev_seq_num ;

  PhaseSpaceTongs(ros::Node& node) : node_(&node)
  {
    prev_seq_num = 0 ;
    node_->advertise<robot_msgs::PoseStamped>("tong_pose", 1) ;
    node_->advertise<std_msgs::Float64>("tong_spacing",1) ;
    node_->subscribe("phase_space_snapshot", snapshot_, &PhaseSpaceTongs::snapshotCallback, this, 1) ;

    node_->param("~left_id", left_id_, 0) ;
    node_->param("~right_id", right_id_, 1) ;

    node_->param("~frame_id", frame_id_, string("phase_space") ) ;
  }

  ~PhaseSpaceTongs()
  {
    node_->unsubscribe("phase_space_snapshot") ;
    node_->unadvertise("tong_pose") ;
    node_->unadvertise("tong_angle") ;
  }

  void snapshotCallback()
  {
    if (prev_seq_num+1 != snapshot_.header.seq)
      printf("%u: Dropped %i Snapshot Packets\n", snapshot_.header.seq,
	       snapshot_.header.seq - prev_seq_num) ;
    //    else
    //      printf("%u: \n", snapshot_.header.seq) ;
    prev_seq_num = snapshot_.header.seq ;

    tf::Pose left ;
    tf::Pose right ;

    bool left_found  = false ;
    bool right_found = false ;

    for (unsigned int i=0; i<snapshot_.get_bodies_size(); i++)
    {
      if (snapshot_.bodies[i].id == left_id_)
      {
        left_found = true ;
        tf::TransformMsgToTF(snapshot_.bodies[i].pose, left) ;
      }
      if (snapshot_.bodies[i].id == right_id_)
      {
        right_found = true ;
        tf::TransformMsgToTF(snapshot_.bodies[i].pose, right) ;
      }
    }

    if (left_found && right_found)
    {
      //tf::Quaternion rotation_diff = right.getRotation() * left.getRotation().inverse() ;
      //double opening_angle = rotation_diff.getAngle() ;

      tf::Vector3 left_ref(.220, 0.0, 0.0) ;
      tf::Vector3 right_ref(.220, 0.0, 0.0) ;

      tf::Vector3 spacing_vec = left*left_ref - right*right_ref ;

      std_msgs::Float64 tong_spacing ;
      tong_spacing.data = spacing_vec.length() ;

      // For now, just use the left tong as the pose

      tf::Stamped<tf::Pose> tf_pose(left, snapshot_.header.stamp, frame_id_) ;
      robot_msgs::PoseStamped pose_msg ;
      tf::PoseStampedTFToMsg(tf_pose, pose_msg) ;

      node_->publish("tong_spacing", tong_spacing) ;

      pose_msg.header.stamp = ros::Time::now() - ros::Duration(.5) ;
      node_->publish("tong_pose", pose_msg) ;
    }
  }
} ;

}

using namespace pr2_phase_space ;


int main(int argc, char** argv)
{
  ros::init(argc, argv);

  ros::Node node("phase_space_tongs") ;

  PhaseSpaceTongs tongs(node) ;

  node.spin() ;

  return 0 ;
}

