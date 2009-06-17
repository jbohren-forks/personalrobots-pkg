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


#include "ros/ros.h"
#include "tf/tfMessage.h"
#include "tf/transform_datatypes.h"
#include "mechanism_msgs/MechanismState.h"
#include "laser_scan/LaserScan.h"
#include "image_msgs/RawStereo.h"

class LaserHeadKinematics
{
public:
  LaserHeadKinematics(ros::NodeHandle& nh)
  {

    buildKinematicModel() ;

    // TF Updates
    tf_pub_ = nh.advertise<tf::tfMessage>("tf_message", 100) ;
    ms_sub_ = nh.subscribe("mechanism_state", 100,
                        &LaserHeadKinematics::updateTf,
                        this) ;

    // Tilt Scan Updates
    laser_pub_ = nh.advertise<laser_scan::LaserScan>("tilt_scan_cal", 100) ;
    laser_sub_ = nh.subscribe("tilt_scan", 100,
                              &LaserHeadKinematics::updateTiltScan, this) ;

    // Raw Stereo Updates
    stereo_pub_ = nh.advertise<image_msgs::RawStereo>("/stereo/raw_stereo_cal", 100) ;
    stereo_sub_ = nh.subscribe("/stereo/raw_stereo", 100,
                               &LaserHeadKinematics::updateStereo, this) ;
  }

  void updateStereo(const image_msgs::RawStereoConstPtr& msg_in)
  {
    image_msgs::RawStereo msg_out ;
    msg_out = *msg_in ;
    msg_out.header.frame_id = "stereo_optical_frame_cal" ;
    msg_out.right_info.P[3] = PR_03_ ;
    stereo_pub_.publish(msg_out) ;
  }

  void updateTiltScan(const laser_scan::LaserScanConstPtr& msg_in)
  {
    laser_scan::LaserScan msg_out ;
    msg_out = *msg_in ;
    msg_out.header.frame_id = "laser_tilt_link_cal" ;
    laser_pub_.publish(msg_out) ;
  }

  btTransform buildTransform(double p[])
  {
    btVector3 trans   (p[0], p[1], p[2]) ;
    btVector3 rot_axis(p[3], p[4], p[5]) ;
    double angle = rot_axis.length() ;
    if (angle < .000001)
    {
      rot_axis = btVector3(1,0,0) ;
      angle = 0.0 ;
    }
    return btTransform(btQuaternion(rot_axis, angle), trans) ;
  }

  btTransform rotYJoint(double ang)
  {
    return btTransform(btQuaternion(btVector3(0, 1, 0), ang),
                       btVector3(0,0,0)) ;
  }

  btTransform rotZJoint(double ang)
  {
    return btTransform(btQuaternion(btVector3(0, 0, 1), ang),
                       btVector3(0,0,0)) ;
  }

  void buildKinematicModel()
  {

    double laser_initial[]= { 0.10741,  0.00848,  0.20616,
                              0.01125, -0.66536,  0.02150};
    double laser_final[]  = { 0.00000,  0.00000,  0.03858,
                             -0.00268,  0.67298,  0.00901};
    double head_initial[] = { 0.00000,  0.00000,  0.38150,
                              0.00000,  0.00000,  0.00000};
    double after_pan[]    = { 0.05800,  0.00000,  0.00000,
                             -1.57080, -0.00000, -0.00000};
    double after_tilt[]   = { 0.02843, -0.07069,  0.01307,
                              0.00072,  1.57238, -0.00040};
    PR_03_ = -63.30398;

    laser_initial_T_ = buildTransform(laser_initial) ;
    laser_final_T_   = buildTransform(laser_final) ;
    head_initial_T_  = buildTransform(head_initial) ;
    after_pan_T_      = buildTransform(after_pan) ;
    after_tilt_T_     = buildTransform(after_tilt) ;
  }

  double getJointPos(const mechanism_msgs::MechanismStateConstPtr& mech_state,
                     const std::string& name)
  {
    for(unsigned int i=0; i<mech_state->joint_states.size(); i++)
    {
      if (name == mech_state->joint_states[i].name)
        return mech_state->joint_states[i].position ;
    }

    assert(false) ;
    return -1 ;
  }


  void updateTf(const mechanism_msgs::MechanismStateConstPtr& mech_state)
  {
    //printf("In callback\n") ;

    btTransform pan_joint_T   = rotZJoint(getJointPos(mech_state, "head_pan_joint")) ;
    btTransform tilt_joint_T  = rotZJoint(getJointPos(mech_state, "head_tilt_joint")) ;
    btTransform laser_joint_T = rotYJoint(getJointPos(mech_state, "laser_tilt_mount_joint")) ;

    tf::tfMessage msg ;
    msg.transforms.resize(5) ;

    for (unsigned int i=0; i<msg.transforms.size(); i++)
      msg.transforms[i].header.stamp = mech_state->header.stamp ;

    // Laser Chain
    msg.transforms[0].parent_id = "torso_lift_link" ;
    msg.transforms[0].header.frame_id = "laser_tilt_mount_link_cal" ;
    tf::TransformTFToMsg(laser_initial_T_*laser_joint_T, msg.transforms[0].transform) ;

    msg.transforms[1].parent_id = "laser_tilt_mount_link_cal" ;
    msg.transforms[1].header.frame_id = "laser_tilt_link_cal" ;
    tf::TransformTFToMsg(laser_final_T_, msg.transforms[1].transform) ;

    // Head Chain
    msg.transforms[2].parent_id = "torso_lift_link" ;
    msg.transforms[2].header.frame_id = "head_pan_link_cal" ;
    tf::TransformTFToMsg(head_initial_T_*pan_joint_T, msg.transforms[2].transform) ;

    msg.transforms[3].parent_id = "head_pan_link_cal" ;
    msg.transforms[3].header.frame_id = "head_tilt_link_cal" ;
    //tf::TransformTFToMsg(after_pan_T_*tilt_joint_T, msg.transforms[3].transform) ;
    tf::TransformTFToMsg(after_pan_T_*tilt_joint_T, msg.transforms[3].transform) ;

    msg.transforms[4].parent_id = "head_tilt_link_cal" ;
    msg.transforms[4].header.frame_id = "stereo_optical_frame_cal" ;
    tf::TransformTFToMsg(after_tilt_T_, msg.transforms[4].transform) ;

    tf_pub_.publish(msg) ;
    //printf("done!\n") ;
  }


private:
  ros::Subscriber ms_sub_ ;
  ros::Publisher  tf_pub_ ;

  ros::Subscriber laser_sub_ ;
  ros::Publisher  laser_pub_ ;

  ros::Subscriber stereo_sub_ ;
  ros::Publisher  stereo_pub_ ;

  double PR_03_ ;                       // Stores elem P[0][3] in the projection matrix for the right cam

  btTransform laser_initial_T_ ;
  btTransform laser_final_T_ ;
  btTransform head_initial_T_ ;
  btTransform after_pan_T_ ;
  btTransform after_tilt_T_ ;

} ;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_pub") ;

  ros::NodeHandle nh ;

  LaserHeadKinematics lh_kinematics(nh) ;

  ros::spin() ;

  return 0 ;
}

