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

#include <node_sequencer/sequencer_client.h>

namespace node_sequencer
{

void waitFor (const string& name, const string& ns)
{
  impl::Waiter(name, ns).wait();
}

void notify (const string& name, const string& ns)
{
  impl::Notifier(name, ns).notify();
}



namespace impl
{


Waiter::Waiter (const string& name, const string& ns) : name_(name), done_(false)
{
  sub_=nh_.subscribe(ns+"/can_start", 1000, &Waiter::waiterCallback, this);
}

void Waiter::waiterCallback (const String::ConstPtr& msg)
{
  if (msg->data == name_)
    done_=true;
}

void Waiter::wait()
{
  ros::Duration d(.1);
  while (!done_ && nh_.ok())
  {
    ros::spinOnce();
    d.sleep();
  }
}


Notifier::Notifier (const string& name, const string& ns) : connected_(false)
{
  pub_ = nh_.advertise<String>(ns + "/completed", 1000, bind(&Notifier::connectionCallback, this, _1));
  msg_.data = name;
}

void Notifier::notify ()
{
  ros::Duration d(.1);
  while (!connected_) {
    ros::spinOnce();
    d.sleep();
  }
  pub_.publish(msg_);
}

void Notifier::connectionCallback (const ros::SingleSubscriberPublisher& pub)
{
  connected_=true;
}

  
} // namespace impl
} // namespace node_sequencer
