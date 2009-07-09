/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Vijay Pradeep

#include <string>

#include "ros/node.h"
#include "robot_msgs/PointCloud.h"

#include "mechanism_model/robot.h"
#include "mechanism_model/chain.h"
#include "tinyxml/tinyxml.h"
#include "hardware_interface/hardware_interface.h"

#include "kdl/chainfksolverpos_recursive.hpp"

using namespace std ;

/**
 * Given a set of tilt-angles, ray-angles, and ranges, project these into a point cloud,
 * which can be displayed in rviz
 */
class PR2TiltLaserProjector
{
public:
  PR2TiltLaserProjector(ros::Node* node) : node_(node), hw_(0)
  {
    // Init the robot
    ROS_DEBUG("Getting robot description from param server...") ;
    string robot_desc ;
    bool success ;
    success = node_->getParam("robotdesc/pr2", robot_desc) ;
    if (!success)
    {
      ROS_ERROR("ERROR: Could not access robot_desc/pr2 from param server\n") ;
    }
    else
      ROS_DEBUG("Success!\n") ;

    ROS_DEBUG("Parsing robotdesc/pr2...") ;
    fflush(stdout) ;
    TiXmlDocument doc ;
    doc.Parse(robot_desc.c_str()) ;

    TiXmlElement *root = doc.FirstChildElement("robot");
    if (!root)
    {
      ROS_ERROR("Error finding 'robot' tag in xml\n");
    }

    ROS_DEBUG("Initializing Robot...") ;
    fflush(stdout) ;
    robot_.hw_ = &hw_ ;
    robot_.initXml(root) ;
    ROS_DEBUG("Success!\n") ;

    // Add an extra joint & link after the laser_tilt_link


    mechanism::Joint* pointing_joint_ = new mechanism::Joint() ;
    mechanism::Link* pointing_link_   = new mechanism::Link() ;
    pointing_joint_->name_ = "laser_tilt_pointing_joint" ;
    pointing_joint_->axis_ = tf::Vector3(0,0,1) ;
    pointing_joint_->type_ = mechanism::JOINT_CONTINUOUS ;

    pointing_link_->name_        = "laser_tilt_pointing_frame" ;
    pointing_link_->parent_name_ = "laser_tilt_link" ;
    pointing_link_->joint_name_  = "laser_tilt_pointing_joint" ;
    pointing_link_->origin_xyz_  = tf::Vector3(0,0,0) ;
    pointing_link_->origin_rpy_  = tf::Vector3(0,0,0) ;

    // Extract the pertinent chain
    robot_.links_.push_back(pointing_link_) ;
    robot_.joints_.push_back(pointing_joint_) ;

    mechanism::Chain mech_chain ;
    success = mech_chain.init(&robot_, "torso_lift_link", "laser_tilt_pointing_frame") ;
    if (success)
      ROS_INFO("Extract chain") ;
    else
      ROS_ERROR("Error extracting chain") ;

    // Convert the chain to KDL
    mech_chain.toKDL(chain_);

    // Add the 'range' joint
    chain_.addSegment(KDL::Segment("RangeSegment", KDL::Joint("RangeJoint", KDL::Joint::TransX))) ;

    if (chain_.getNrOfJoints() != 3)
      ROS_ERROR("Num joints doesn't seem right") ;
    else
      printf("Num joints correct\n") ;
    node_->advertise<robot_msgs::PointCloud>("dense_tilt_scan/projected_corners", 1) ;
    node_->subscribe("dense_tilt_scan/measured_corners", in_, &PR2TiltLaserProjector::callback, this, 1) ;
  }

  void callback()
  {
    unsigned int C = in_.pts.size() ;
    robot_msgs::PointCloud cloud_out ;
    cloud_out.pts.resize(C) ;

    KDL::ChainFkSolverPos_recursive solver(chain_) ;
    KDL::Frame loc ;
    KDL::JntArray joint_vals(3) ;


    for (unsigned int i=0; i<C; i++)
    {
      joint_vals(0) = in_.pts[i].x ;
      joint_vals(1) = in_.pts[i].y ;
      joint_vals(2) = in_.pts[i].z ;
      solver.JntToCart(joint_vals, loc) ;
      cloud_out.pts[i].x = loc.p.x() ;
      cloud_out.pts[i].y = loc.p.y() ;
      cloud_out.pts[i].z = loc.p.z() ;
    }
    cloud_out.header.stamp = in_.header.stamp ;
    cloud_out.header.frame_id = "torso_lift_link" ;

    node_->publish("dense_tilt_scan/projected_corners", cloud_out) ;
  }


private:
  ros::Node* node_ ;
  mechanism::Robot robot_ ;
  mechanism::RobotState* robot_state_ ;
  HardwareInterface hw_ ;

  KDL::Chain chain_ ;

  robot_msgs::PointCloud in_ ;


};




int main(int argc, char** argv)
{
  ros::init(argc, argv);

  ros::Node node("pr2_tilt_laser_projector");

  PR2TiltLaserProjector projector(&node);

  node.spin();
}


