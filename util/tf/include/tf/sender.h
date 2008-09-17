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

#ifndef TF_TRANSFORMSENDER_H
#define TF_TRANSFORMSENDER_H

#include "ros/node.h"
#include "tf/tf.h"
#include "tf/tfMessage.h"

namespace tf
{

class TransformSender{
public:
  TransformSender(ros::node& anode):
    node_(anode)
  {
    node_.advertise<tfMessage>("/tfMessage", 100);
  };
  
  void sendTransform(const Stamped<btTransform> & transform, const std::string& parent_id)
  {
    tfMessage message;
    message.header.stamp = ros::Time(transform.stamp_);
    message.header.frame_id = transform.frame_id_;
    message.parent = parent_id;
    TransformBtToMsg(transform.data_, message.transform);
    node_.publish("/tfMessage", message);
  } 
  
  void sendTransform(const btTransform & transform, const uint64_t & time, const std::string& frame_id, const std::string& parent_id)
  {
    tfMessage message;
    message.header.stamp = ros::Time(time);
    message.header.frame_id = frame_id;
    message.parent = parent_id;
    TransformBtToMsg(transform, message.transform);
    node_.publish("/tfMessage", message);
  }
  
private:
  ros::node & node_;

};

}

#endif //TF_TRANSFORMSENDER_H
