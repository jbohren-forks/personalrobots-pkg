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

/** \author Tully Foote */

#ifndef TF_TRANSFORMLISTENER_H
#define TF_TRANSFORMLISTENER_H

#include "robot_msgs/PointCloud.h"
#include "std_msgs/Empty.h"
#include "tf/tfMessage.h"
#include "tf/tf.h"
#include "ros/node.h"

#include "tf/FrameGraph.h" //frame graph service

// This extern avoids an undefined symbol error on OS X.  In Linux, it
// generates multiply defined symobls.  There must be some non-ifdef middle
// ground.
#if defined(__APPLE__)
extern const double tf::Transformer::DEFAULT_CACHE_TIME;
#endif

namespace tf{

/** \brief This class inherits from Transformer and automatically subscribes to ROS transform messages */
class TransformListener : public Transformer { //subscribes to message and automatically stores incoming data

private:
  ros::Node& node_;

public:
  /**@brief Constructor for transform listener
   * \param rosnode A reference to an instance of a ros::Node for communication
   * \param interpolating Whether to interpolate or return the closest
   * \param max_cache_time How long to store transform information */
  TransformListener(ros::Node & rosnode,
                    bool interpolating = true,
                    ros::Duration max_cache_time = ros::Duration(DEFAULT_CACHE_TIME)):
    Transformer(interpolating,
                max_cache_time),
    node_(rosnode)
  {
    //  printf("Constructed rosTF\n");
    node_.subscribe("/tf_message", msg_in_, &TransformListener::subscription_callback, this,100); ///\todo magic number

    node_.subscribe("/reset_time", empty_, &TransformListener::reset_callback, this,100); ///\todo magic number

    node_.advertiseService("~tf_frames", &TransformListener::getFrames, this);
    node_.param(std::string("~tf_prefix"), tf_prefix_, std::string(""));
  };

  ~TransformListener()
  {
    node_.unsubscribe("/tf_message", &TransformListener::subscription_callback, this);
    node_.unsubscribe("/reset_time", &TransformListener::reset_callback, this);
  };

  /* Methods from transformer unhiding them here */
  using Transformer::transformQuaternion;
  using Transformer::transformVector;
  using Transformer::transformPoint;
  using Transformer::transformPose;


  /** \brief Transform a Stamped Quaternion Message into the target frame */
  void transformQuaternion(const std::string& target_frame, const robot_msgs::QuaternionStamped& stamped_in, robot_msgs::QuaternionStamped& stamped_out);
  /** \brief Transform a Stamped Vector Message into the target frame */
  void transformVector(const std::string& target_frame, const robot_msgs::Vector3Stamped& stamped_in, robot_msgs::Vector3Stamped& stamped_out);
  /** \brief Transform a Stamped Point Message into the target frame */
  void transformPoint(const std::string& target_frame, const robot_msgs::PointStamped& stamped_in, robot_msgs::PointStamped& stamped_out);
  /** \brief Transform a Stamped Pose Message into the target frame */
  void transformPose(const std::string& target_frame, const robot_msgs::PoseStamped& stamped_in, robot_msgs::PoseStamped& stamped_out);


  /** \brief Transform a Stamped Quaternion Message into the target frame*/
  void transformQuaternion(const std::string& target_frame, const ros::Time& target_time,
                           const robot_msgs::QuaternionStamped& qin,
                           const std::string& fixed_frame, robot_msgs::QuaternionStamped& qout);
  /** \brief Transform a Stamped Vector Message into the target frame and time */
  void transformVector(const std::string& target_frame, const ros::Time& target_time,
                       const robot_msgs::Vector3Stamped& vin,
                           const std::string& fixed_frame, robot_msgs::Vector3Stamped& vout);
  /** \brief Transform a Stamped Point Message into the target frame and time  */
  void transformPoint(const std::string& target_frame, const ros::Time& target_time,
                           const robot_msgs::PointStamped& pin,
                           const std::string& fixed_frame, robot_msgs::PointStamped& pout);
  /** \brief Transform a Stamped Pose Message into the target frame and time  */
  void transformPose(const std::string& target_frame, const ros::Time& target_time,
                     const robot_msgs::PoseStamped& pin,
                     const std::string& fixed_frame, robot_msgs::PoseStamped& pout);


  /** \brief Transform a robot_msgs::PointCloud natively */
    void transformPointCloud(const std::string& target_frame, const robot_msgs::PointCloud& pcin, robot_msgs::PointCloud& pcout);

  /** @brief Transform a robot_msgs::PointCloud in space and time */
  void transformPointCloud(const std::string& target_frame, const ros::Time& target_time,
                           const robot_msgs::PointCloud& pcin,
                           const std::string& fixed_frame, robot_msgs::PointCloud& pcout);



    ///\todo move to high precision laser projector class  void projectAndTransformLaserScan(const robot_msgs::LaserScan& scan_in, robot_msgs::PointCloud& pcout);

  bool getFrames(tf::FrameGraph::Request& req, tf::FrameGraph::Response& res)
  {
    res.dot_graph = allFramesAsDot();
    return true;
  }

private:
  /// memory space for callback
  tfMessage msg_in_;
  ///\todo Switch to robot_msgs::Transform
  /// Callback function for ros message subscriptoin
  void subscription_callback();

  /** @brief a helper function to be used for both transfrom pointCloud methods */
  void transformPointCloud(const std::string & target_frame, const Transform& transform, const ros::Time& target_time, const robot_msgs::PointCloud& pcin, robot_msgs::PointCloud& pcout);

  /// clear the cached data
  void reset_callback(){clear();};
  std_msgs::Empty empty_;

};
}

#endif //TF_TRANSFORMLISTENER_H
