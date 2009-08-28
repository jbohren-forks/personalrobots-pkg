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

#ifndef CALIBRATION_MESSAGE_FILTERS_STATIONARY_FILTER_H_
#define CALIBRATION_MESSAGE_FILTERS_STATIONARY_FILTER_H_

#include "message_filters/simple_filter.h"

#include "calibration_message_filters/stationary_elem.h"
#include "calibration_message_filters/joint_tolerance.h"
#include "pr2_mechanism_msgs/MechanismState.h"

namespace calibration_message_filters
{

/**
 * Given a 'SandwichElem' that is tagged with MechanismState, this filter determines
 * if the element was stationary during the interval.  The callback is called only
 * if the elem was in fact stationary.
 */
template <class M>
class StationaryFilter
{
public:
  typedef boost::shared_ptr< SandwichElem<M, pr2_mechanism_msgs::JointStates> > SandConstPtr;   // Shared pointer to the sandwich elem
  typedef boost::function<void(const StationaryElem<M>&)> Callback;
  typedef boost::signal<Callback> Signal;

  StationaryFilter()
  {

  }

  template<class F>
  StationaryFilter(F& f, std::vector<JointTolerance> joint_tol)
  {
    connectInput(f);
    setJointTolerances(joint_tol);
  }

  ~StationaryFilter()
  {
    incoming_connection_.disconnect();
  }

  void connectInput(F& f)
  {
    incoming_connection_ = f.connect(boost::bind(&StationaryFilter<M>::callback, this));
  }

  void registerCallback(const Callback& callback)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    return Connection(boost::bind(&StationaryFilter<M>::disconnect, this), signal_.connect(callback));
  }

  void setJointTolerances(std::vector<JointTolerance> joint_tol)
  {
    joint_tol_ = joint_tol;
  }

  void callback(const SandConstPtr& msg)
  {


  }

private:
  void disconnect(const Connection& c)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    c.getBoostConnection().disconnect();
  }

  std::vector<JointTolerance> joint_tol_;
  boost::mutex data_mutex_;

  Connection incoming_connection_;
  Signal signal_;
  boost::mutex signal_mutex_;
};

}

#endif // CALIBRATION_MESSAGE_FILTERS_STATIONARY_FILTER_H_
