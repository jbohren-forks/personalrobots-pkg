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
 * This node takes the PhaseSpaceSnapshot packet and repackages a marker as a point_stamped message
 */

#include <string>

#include "ros/node.h"

// Messages
#include "phase_space/PhaseSpaceSnapshot.h"
#include "phase_space/PhaseSpaceMarker.h"
#include "phase_space/PhaseSpaceBody.h"

#include "std_msgs/PoseStamped.h"

#include "tf/transform_datatypes.h"

using namespace std ;

namespace pr2_phase_space
{

/**
 * \brief Listens for a PhaseSpaceSnapshot message and then publishes it as a PoseStamped message, based on a
 * series a parameters.
 * @section topic ROS topics
 * Subscribes to (name [type]):
 * - @b "phase_space_snapshot" [phase_space/PhaseSpaceSnapshot] : The current state of the phasespace system,
 *               which is normally published by phase_space::PhaseSpaceNode
 *
 * Publishes to (name [type]):
 * - @b "cmd" [std_msgs/PoseStamped] : The commanded pose, with an associated timestamp and frame_id. You will
 *               probably have to remap this topic name in order to feed this command into a controller.
 * @section parameters ROS parameters
 * - @b ~body_id : The PhaseSpace ID associated with the rigid body we want to track
 * - @b ~num_to_skip : The number of PhaseSpace Snapshot frames we want to skip before republishing. If this
 *                       is 0, then always publish when we get phasespace data. If this is 29, then we publish
 *                       once and then skip the next 29 frames.
 * - @b ~frame_id : The TF frame name that should be attached to the published message
 * - @b ~offset_* : This set of params defines a transform to be applied before publishing the data
 *   - @b ~offset_trans_x, @b ~offset_trans_y, @b ~offset_trans_z : Defines the translation vector of the transform
 *   - @b ~offset_rot_axis_x, @b ~offset_rot_axis_y, @b ~offset_rot_axis_z : Defines the rotation axis of the transform
 *   - @b ~offset_rot_angle : Defines the rotation angle of the transform
 */
class PhaseSpacePoseStamped : public ros::node
{
public :
  PhaseSpacePoseStamped() : ros::node("phase_space_point_stamped")
  {
    subscribe("phase_space_snapshot", snapshot_, &PhaseSpacePoseStamped::snapshotCallback, 10) ;
    advertise<std_msgs::PoseStamped>("cmd", 1) ;

    param("~body_id", body_id_, 1) ;
    param("~num_to_skip", num_to_skip_, 0) ;

    param("~frame_id", frame_id_, string("base_link") ) ;

    param("~scale_trans_x", scale_trans_[0], 1.0) ;
    param("~scale_trans_y", scale_trans_[1], 1.0) ;
    param("~scale_trans_z", scale_trans_[2], 1.0) ;

    param("~offset_trans_x", offset_trans_[0], 0.0) ;
    param("~offset_trans_y", offset_trans_[1], 0.0) ;
    param("~offset_trans_z", offset_trans_[2], 0.0) ;

    param("~offset_rot_axis_x", offset_rot_axis_[0], 0.0) ;
    param("~offset_rot_axis_y", offset_rot_axis_[1], 0.0) ;
    param("~offset_rot_axis_z", offset_rot_axis_[2], 1.0) ;

    param("~offset_rot_angle", offset_rot_angle_, 0.0) ;

    // Build the transform to apply each run

    tf::Vector3 axis(offset_rot_axis_[0], offset_rot_axis_[1], offset_rot_axis_[2]) ;
    if (axis.length() < .000001)
      ROS_ERROR("WARNING: ROT AXIS is too small (%f, %f, %f)", offset_rot_axis_[0], offset_rot_axis_[1], offset_rot_axis_[2]) ;

    tf::Quaternion rot(axis, offset_rot_angle_) ;
    tf::Vector3 trans(offset_trans_[0], offset_trans_[1], offset_trans_[2]) ;

    transform_.setRotation(rot) ;
    transform_.setOrigin(trans) ;

    publish_count_ = 0 ;
  }

  ~PhaseSpacePoseStamped()
  {
    unsubscribe("cmd") ;
  }

  void snapshotCallback()
  {
    if (snapshot_.frameNum % (num_to_skip_+1) != 0)
      return ;

    for (unsigned int i=0; i<snapshot_.get_bodies_size(); i++)
    {
      if (snapshot_.bodies[i].id == body_id_)                   // Did we find our body?
      {
        const phase_space::PhaseSpaceBody& cur_body = snapshot_.bodies[i] ;

        // Define our starting frame
        tf::Quaternion rot_phasespace ;
        tf::QuaternionMsgToTF(cur_body.pose.rotation, rot_phasespace) ;
        tf::Point trans_phasespace(cur_body.pose.translation.x*scale_trans_[0],
                                   cur_body.pose.translation.y*scale_trans_[1],
                                   cur_body.pose.translation.z*scale_trans_[2]) ;
        tf::Transform pose_phasespace(rot_phasespace, trans_phasespace) ;

        // Transform our starting frame by our fixed transform
        tf::Transform pose_result(transform_.inverse()*pose_phasespace ) ;

        std_msgs::PoseStamped pose_msg ;

        pose_msg.header.stamp = ros::Time() ;
        pose_msg.header.frame_id =  frame_id_ ;

        tf::PointTFToMsg(pose_result.getOrigin(), pose_msg.pose.position) ;
        tf::QuaternionTFToMsg(pose_result.getRotation(), pose_msg.pose.orientation) ;

        publish("cmd", pose_msg) ;

        return ;
      }
      else
        printf("\n") ;
    }
    return ;
  }

private :
  phase_space::PhaseSpaceSnapshot snapshot_ ;

  tf::Transform transform_ ;

  double scale_trans_[3] ;
  double offset_trans_[3] ;
  double offset_rot_axis_[3] ;
  double offset_rot_angle_ ;
  string frame_id_ ;
  int publish_count_ ;
  int body_id_ ;
  int num_to_skip_ ;
} ;

}


using namespace pr2_phase_space ;


int main(int argc, char** argv)
{
  ros::init(argc, argv);

  PhaseSpacePoseStamped phase_space_pose_stamped ;

  phase_space_pose_stamped.spin() ;

  ros::fini() ;

  return 0 ;
}
