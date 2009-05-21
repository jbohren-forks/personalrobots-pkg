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

/*! \mainpage
 *  \htmlinclude manifest.html
 */

#ifndef MESSAGE_FILTERS_SYNC_HELPER_H_
#define MESSAGE_FILTERS_SYNC_HELPER_H_

#include <boost/thread.hpp>
#include <vector>

namespace message_filters
{

template <class M>
class SyncHelper
{
public:

  /**
   * Used to link ROS messages to a filter system
   * \param topic The topic that we want to subscribe to
   * \param queue_size Defines the queue size in the subscribe call
   * \param nh Node handle
   */
  SyncHelper(const std::string& topic, uint32_t queue_size, ros::NodeHandle& nh)
  {
    sub_ = nh.subscribe(topic, queue_size, &SyncHelper<M>::msgHandler, this) ;
  }

  /**
   *  Ros subscribe callback. Receives messages, and then pushes them to all the
   *  output callbacks
   */
  void msgHandler(const boost::shared_ptr<M const>& msg)
  {
    printf("%u - Called SyncHelper Callback!\n", msg->header.seq) ;

    // Sequentially call each registered call
    for (unsigned int i=0; i<output_callbacks_.size(); i++)
      output_callbacks_[i](msg) ;
  }

  /**
   * Called by another filter that wants the output of this filter
   * \param callback The function that is called when data is available
   */
  void addOutputCallback(const boost::function<void(const boost::shared_ptr<M const>&)>& callback)
  {
    output_callbacks_.push_back(callback) ;
  }

private:
  ros::Subscriber sub_ ;

  //! Vector of callbacks that need to be called, whenever data is ready.
  std::vector<boost::function<void(const boost::shared_ptr<M const>&)> > output_callbacks_ ;

} ;

}



#endif  // MESSAGE_FILTERS_SYNC_HELPER_H_
