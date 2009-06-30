/*
 * Copyright (C) 2008, Jason Wolfe and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/exceptions.h"
#include "tf/transform_listener.h"

#include "tf/FrameGraph.h"

#include "robot_msgs/PointStamped.h"
#include "robot_msgs/PointCloud.h"
#include "robot_msgs/PoseStamped.h"
#include "robot_msgs/QuaternionStamped.h"
#include "robot_msgs/Vector3Stamped.h"

#include "tf_node/TransformPoint.h"
#include "tf_node/TransformPointCloud.h"
#include "tf_node/TransformPose.h"
#include "tf_node/TransformQuaternion.h"
#include "tf_node/TransformVector.h"


using namespace tf_node;

tf::TransformListener *tl;

bool getFramesCallback(tf::FrameGraph::Request &req, 
		       tf::FrameGraph::Response &res) {
  return tl->getFrames(req, res);
}

bool transformPointCallback(TransformPoint::Request &req, 
			    TransformPoint::Response &res) {
  try {
    if (req.fixed_frame.length() == 0) {
      tl->transformPoint(req.target_frame, req.pin, res.pout);
    } else {
      tl->transformPoint(req.target_frame, req.target_time, req.pin, req.fixed_frame, res.pout);
    }
  } catch (tf::TransformException &e) {
      ROS_ERROR(e.what());
      return false;
  }
  return true;
}

bool transformPointCloudCallback(TransformPointCloud::Request &req, 
			         TransformPointCloud::Response &res) {
  try {
    if (req.fixed_frame.length() == 0) {
      tl->transformPointCloud(req.target_frame, req.pcin, res.pcout);
    } else {
      tl->transformPointCloud(req.target_frame, req.target_time, req.pcin, req.fixed_frame, res.pcout);
    }
  } catch (tf::TransformException &e) {
      ROS_ERROR(e.what());
      return false;
  }
  return true;
}

bool transformPoseCallback(TransformPose::Request &req, 
			    TransformPose::Response &res) {
  try {
    if (req.fixed_frame.length() == 0) {
      tl->transformPose(req.target_frame, req.pin, res.pout);
    } else {
      tl->transformPose(req.target_frame, req.target_time, req.pin, req.fixed_frame, res.pout);
    }
  } catch (tf::TransformException &e) {
      ROS_ERROR(e.what());
      return false;
  }
  return true;
}

bool transformQuaternionCallback(TransformQuaternion::Request &req, 
			    TransformQuaternion::Response &res) {
  try {
    if (req.fixed_frame.length() == 0) {
      tl->transformQuaternion(req.target_frame, req.qin, res.qout);
    } else {
       tl->transformQuaternion(req.target_frame, req.target_time, req.qin, req.fixed_frame, res.qout);
    }
  } catch (tf::TransformException &e) {
      ROS_ERROR(e.what());
      return false;
  }
  return true;
}

bool transformVectorCallback(TransformVector::Request &req, 
			    TransformVector::Response &res) {
  try {
    if (req.fixed_frame.length() == 0) {
      tl->transformVector(req.target_frame, req.vin, res.vout);
    } else {
      tl->transformVector(req.target_frame, req.target_time, req.vin, req.fixed_frame, res.vout);
    }
  } catch (tf::TransformException &e) {
      ROS_ERROR(e.what());
      return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_node");

  ros::NodeHandle n;

  // Use default arguments
  tl = new tf::TransformListener(); //*(n.getNode()));

  ros::ServiceServer f  = n.advertiseService("get_tf_frames",         &getFramesCallback);
  ros::ServiceServer p  = n.advertiseService("transform_point",       &transformPointCallback);
  ros::ServiceServer pc = n.advertiseService("transform_point_cloud", &transformPointCloudCallback);
  ros::ServiceServer ps = n.advertiseService("transform_pose",        &transformPoseCallback);
  ros::ServiceServer q  = n.advertiseService("transform_quaternion",  &transformQuaternionCallback);
  ros::ServiceServer v  = n.advertiseService("transform_vector",      &transformVectorCallback);


  ros::spin();

  return 0;
}
