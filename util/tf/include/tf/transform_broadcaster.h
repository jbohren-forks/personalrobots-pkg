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

#ifndef TF_TRANSFORMBROADCASTER_H
#define TF_TRANSFORMBROADCASTER_H

#include "ros/node.h"
#include "tf/tf.h"
#include "tf/tfMessage.h"
///\todo only for backwards compatabilty, remove!
#include "rosTF/TransformArray.h"

namespace tf
{

/** \brief This class provides an easy way to publish coordinate frame transform information.  
 * It will handle all the messaging and stuffing of messages.  And the function prototypes lay out all the 
 * necessary data needed for each message.  */

class TransformBroadcaster{
public:
  /** \brief Constructor (needs a ros::node reference) */
  TransformBroadcaster(ros::node& anode):
    node_(anode)
  {
    node_.advertise<tfMessage>("/tf_message", 100);
  };
  /** \brief Send a Stamped<Transform> with parent parent_id 
   * The stamped data structure includes frame_id, and time already.  */
  void sendTransform(const Stamped<Transform> & transform, const std::string& parent_id)
  {
    tfMessage message;
    message.header.stamp = ros::Time(transform.stamp_);
    message.header.frame_id = transform.frame_id_;
    message.parent = parent_id;
    TransformTFToMsg(transform.data_, message.transform);
    ///\todo removed for non collision with backwards compatability    node_.publish("/tf_message", message);

    ///\todo only for backwards compatabilty, remove!
    rosTF::TransformArray tfArray;
    tfArray.set_quaternions_size(1);

    tfArray.quaternions[0].header.frame_id = transform.frame_id_;
    tfArray.quaternions[0].parent = parent_id;
    Quaternion q = transform.data_.getRotation();
    tfArray.quaternions[0].xt = transform.data_.getOrigin().x();
    tfArray.quaternions[0].yt = transform.data_.getOrigin().y();
    tfArray.quaternions[0].zt = transform.data_.getOrigin().z();
    tfArray.quaternions[0].xr = q.x();
    tfArray.quaternions[0].yr = q.y();
    tfArray.quaternions[0].zr = q.z();
    tfArray.quaternions[0].w = q.w();
    tfArray.quaternions[0].header.stamp = ros::Time(transform.stamp_);

    node_.publish("TransformArray", tfArray);

  } 
  
  /** \brief Send a Transform, stamped with time, frame_id and parent_id */
  void sendTransform(const Transform & transform, const ros::Time& time, const std::string& frame_id, const std::string& parent_id)
  {
    tfMessage message;
    message.header.stamp = time;
    message.header.frame_id = frame_id;
    message.parent = parent_id;
    TransformTFToMsg(transform, message.transform);
    ///\todo removed for non collision with backwards compatability    node_.publish("/tf_message", message);

    ///\todo only for backwards compatabilty, remove!
    rosTF::TransformArray tfArray;
    tfArray.set_quaternions_size(1);

    tfArray.quaternions[0].header.frame_id = frame_id;
    tfArray.quaternions[0].parent = parent_id;
    Quaternion q = transform.getRotation();
    tfArray.quaternions[0].xt = transform.getOrigin().x();
    tfArray.quaternions[0].yt = transform.getOrigin().y();
    tfArray.quaternions[0].zt = transform.getOrigin().z();
    tfArray.quaternions[0].xr = q.x();
    tfArray.quaternions[0].yr = q.y();
    tfArray.quaternions[0].zr = q.z();
    tfArray.quaternions[0].w = q.w();
    tfArray.quaternions[0].header.stamp = ros::Time(time);

    node_.publish("TransformArray", tfArray);
  }
  
private:
  /// Internal reference to ros::node
  ros::node & node_;

};

}

#endif //TF_TRANSFORMBROADCASTER_H
