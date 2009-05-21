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

#ifndef MESSAGE_FILTERS_CONSUMER_H_
#define MESSAGE_FILTERS_CONSUMER_H_

#include <boost/thread.hpp>
#include "image_msgs/CamInfo.h"

namespace message_filters
{

// A really easy 'filter' that simply consumes data
class Consumer
{
public:

  template<class T>

  /**
   * Links Consumer's input to some provider's output.
   * \param provider The filter from which we want to receive data
   */
  void subscribe(T& provider)
  {
    printf("Called Subscribe\n") ;
    provider.addOutputCallback(boost::bind(&Consumer::processData, this, _1)) ;
  }

  /**
   * Method in which we will want to process the incoming data.  Often, this will
   * involve pushing data onto queues, and pushing it along the pipeline into
   * another filter
   */
  void processData(const image_msgs::CamInfoConstPtr& msg)
  {
    printf("%u - Called Consumer Callback!\n", (*msg).header.seq) ;
  }

private:


} ;

}



#endif  // MESSAGE_FILTERS_CONSUMER_H_
