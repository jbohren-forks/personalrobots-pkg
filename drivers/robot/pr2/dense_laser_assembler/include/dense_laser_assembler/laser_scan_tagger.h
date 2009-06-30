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

#ifndef DENSE_LASER_ASSEMBLER_LASER_SCAN_TAGGER_H_
#define DENSE_LASER_ASSEMBLER_LASER_SCAN_TAGGER_H_

#include <deque>
#include "laser_scan/LaserScan.h"
#include "message_filters/msg_cache.h"
#include "dense_laser_assembler/joint_extractor.h"
#include "boost/shared_ptr.hpp"

namespace dense_laser_assembler
{

template <class T>
class LaserScanTagger
{
public:

  struct TaggedLaserScan
  {
    roslib::Header header ;
    laser_scan::LaserScanConstPtr scan ;
    boost::shared_ptr<const T> before ;
    boost::shared_ptr<const T> after ;
  } ;

  typedef boost::shared_ptr<const TaggedLaserScan> TaggedLaserScanConstPtr ;


  LaserScanTagger(message_filters::MsgCache<T>& tag_cache, unsigned int max_queue_size)
  {
    tag_cache_ = &tag_cache ;
    max_queue_size_ = max_queue_size ;
  }

  LaserScanTagger()
  {
    tag_cache_ = NULL ;
    max_queue_size_ = 1 ;
  }

  void setQueueSize(unsigned int max_queue_size)
  {
    max_queue_size_ = max_queue_size ;
  }

  void setTagCache(message_filters::MsgCache<T>& tag_cache)
  {
    tag_cache_ = &tag_cache ;
  }

  /**
   * Adds the laser scan onto the queue of scans that need to be matched with the stamped data
   */
  void processLaserScan(const laser_scan::LaserScanConstPtr& msg)
  {

    queue_lock_.lock() ;

    //! \todo need to decide on overflow logic
    if (queue_.size() < max_queue_size_)
      queue_.push_back(msg) ;
    else
      ROS_WARN("Queue full, not pushing new data onto queue until queue is serviced") ;

    queue_lock_.unlock() ;

    update() ;
  }

  /**
   * Triggers the module to try to service the queue
   */
  void update()
  {
    if (!tag_cache_)
    {
      ROS_WARN("Have a NULL pointer to TagCache. Skipping update") ;
      return ;
    }

    queue_lock_.lock() ;

    bool did_something = true ;

    while (did_something && queue_.size() > 0)
    {
      did_something = false ;   // Haven't done anything yet

      const laser_scan::LaserScanConstPtr& elem = queue_.front() ;

      ros::Time scan_start = elem->header.stamp ;
      ros::Time scan_end   = elem->header.stamp + ros::Duration().fromSec(elem->time_increment*elem->ranges.size()) ;

      boost::shared_ptr<const T> tag_before = tag_cache_->getElemBeforeTime(scan_start) ;
      boost::shared_ptr<const T> tag_after  = tag_cache_->getElemAfterTime(scan_end) ;

      if (!tag_before)          // Can't get old enough tag. Give up
      {
        queue_.pop_front() ;
        did_something = true ;
        ROS_WARN("Popped elem off back of queue") ;
      }
      else if (!tag_after)      // Don't have new enough tag. Keep scan on queue until we get it
      {
        did_something = false ;
      }
      else                      // Both tags are fine. Use the data, and push out
      {
        did_something = true ;
        boost::shared_ptr<TaggedLaserScan> tagged_scan(new TaggedLaserScan) ;
        tagged_scan->header = elem->header ;
        tagged_scan->scan   = elem ;
        tagged_scan->before = tag_before ;
        tagged_scan->after  = tag_after ;

        for (unsigned int i=0; i<output_callbacks_.size(); i++)
          output_callbacks_[i](tagged_scan) ;

        queue_.pop_front() ;
      }

    }

    queue_lock_.unlock() ;
  }

  void addOutputCallback(const boost::function<void(const TaggedLaserScanConstPtr&)>& callback)
  {
    output_callbacks_.push_back(callback) ;
  }


private:
  std::deque<laser_scan::LaserScanConstPtr> queue_ ;    //!< Incoming queue of laser scans
  boost::mutex queue_lock_ ;                            //!< Mutex for queue
  unsigned int max_queue_size_ ;                        //!< Max # of laser scans to queue up for processing

  message_filters::MsgCache<T>* tag_cache_ ;            //!< Cache of the tags that we need to merge with laser data

  std::vector<boost::function<void(const TaggedLaserScanConstPtr&)> > output_callbacks_ ;

} ;

}



#endif /* DENSE_LASER_ASSEMBLER_LASER_SCAN_TAGGER_H_ */
