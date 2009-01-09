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

#include "ros/node.h"
#include "std_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/message_notifier.h"
#include "laser_scan_annotator/LaserScanAnnotated.h"
#include "boost/thread.hpp"

using namespace std ;
using namespace std_msgs ;

namespace laser_scan_annotator
{

class LaserScanAnnotatorNode : public ros::node
{
public:

  LaserScanAnnotatorNode() : ros::node("laser_scan_annotator_node"), tf_(*this)
  {
    advertise<LaserScanAnnotated>("scan_annotated", 100) ;

    // ***** Set fixed_frame *****
    param("~fixed_frame", fixed_frame_, std::string("ERROR_NO_NAME")) ;
    ROS_INFO("Fixed Frame: %s", fixed_frame_.c_str()) ;
    if (fixed_frame_ == "ERROR_NO_NAME")
      ROS_ERROR("Need to set parameter fixed_frame") ;

    // ***** Start Listening to Data *****
    scan_notifier_ = new tf::MessageNotifier<LaserScan>(&tf_, this, boost::bind(&LaserScanAnnotatorNode::callback, this, _1), "scan_in", fixed_frame_, 100) ;
  }

  ~LaserScanAnnotatorNode()
  {
    delete scan_notifier_ ;
  }

  void callback(const boost::shared_ptr<LaserScan>& scan_ptr)
  {
    LaserScan scan_in = *scan_ptr ;
    LaserScanAnnotated scan_annotated ;

    const unsigned int N = scan_in.get_ranges_size() ;

    scan_annotated.scan = scan_in ;

    scan_annotated.set_poses_size(N) ;
    for (unsigned int i=0; i<N; i++)
    {
      ros::Time ray_time(scan_in.header.stamp.to_ull() + (uint64_t) (i*scan_in.time_increment * 1000000000)) ;

      tf::Stamped<tf::Transform> transform ;

      try{
        tf_.lookupTransform(fixed_frame_, scan_in.header.frame_id, ray_time, transform) ;
      }
      catch(tf::LookupException& ex) {
        ROS_INFO("No Transform available Error\n");
        continue;
      }
      catch(tf::ConnectivityException& ex) {
        ROS_INFO("Connectivity Error\n");
        continue;
      }
      catch(tf::ExtrapolationException& ex) {
        ROS_INFO("Extrapolation Error\n");
        continue;
      }
      catch(tf::TransformException e) {
        continue;
      }
      tf::PoseStampedTFToMsg(transform, scan_annotated.poses[i]) ;
    }

    publish("scan_annotated", scan_annotated);
  }

protected:
  std_msgs::LaserScan scan_in_ ;
  string fixed_frame_ ;


  tf::TransformListener tf_ ;
  tf::MessageNotifier<LaserScan>* scan_notifier_ ;
};

}

using namespace laser_scan_annotator ;

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  LaserScanAnnotatorNode t ;
  t.spin();
  ros::fini();
  return 0;
}

