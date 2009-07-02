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



#include "annotated_map_msgs/TaggedPolygonalMap.h"
#include "annotated_planar_patch_map/generic_map_base_assembler_srv.h"

// Service
#include "point_cloud_assembler/BuildCloud.h"


using namespace robot_msgs;
using namespace annotated_map_msgs;
using namespace std ;
using namespace point_cloud_assembler;

namespace annotated_planar_patch_map
{

/**
 * \brief Maintains a history of laser scans and generates a point cloud upon request
 * \todo Clean up the doxygen part of this header
 * params
 *  * (Several params are inherited from BaseAssemblerSrv)
 */
class PcdAssemblerSrv : public GenericMapBaseAssemblerSrv<PointCloud,PointCloud>
{
protected:
  ros::ServiceServer svc_;

public:
  PcdAssemblerSrv() : GenericMapBaseAssemblerSrv<PointCloud,PointCloud>()
  {
    // ***** Start Services *****
    svc_ = n_.advertiseService<BuildCloud::Request,BuildCloud::Response>("~build_cloud", boost::bind(&PcdAssemblerSrv::buildCloud, this,_1,_2)) ;

  }

  ~PcdAssemblerSrv()
  {

  }


  unsigned int GetPointsInCloud(const PointCloud& scan)
  {
    return scan.pts.size();
  }

  void ConvertScanToCloud(const string& fixed_frame_id, const PointCloud& scan_in, PointCloud& cloud_out)
  {
    tf_->transformPointCloud(fixed_frame_id, scan_in, cloud_out) ;
    return ;
  }


  bool buildCloud(BuildCloud::Request& req, BuildCloud::Response& resp)
  {
    ROS_DEBUG("Starting Service Request\n") ;
    ROS_DEBUG_STREAM( "\tFrom: \t" << req.begin << "\n\tTo:\t" << req.end <<"\n") ;
    
    scan_hist_mutex_.lock() ;
    // Determine where in our history we actually are
    unsigned int i = 0 ;

    // Find the beginning of the request. Probably should be a search
    while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
            scan_hist_[i].header.stamp < req.begin )                                    // Keep stepping until we've exceeded the start time
    {
      i++ ;
    }
    unsigned int start_index = i ;
    if(i>0)
      ROS_DEBUG_STREAM( "Last scan before the interval\t" << scan_hist_[i-1].header.stamp<<"\n" );
    else
      ROS_DEBUG_STREAM( "No scans before the interval\n");
    

    

    unsigned int req_pts = 0 ;                                                          // Keep a total of the points in the current request
  // Find the end of the request
    while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
            scan_hist_[i].header.stamp < req.end )                                      // Don't go past the end-time of the request
    {
      req_pts += (GetPointsInCloud(scan_hist_[i])) ;
      i += 1 ;
    }
    unsigned int past_end_index = i ;
    if(i<scan_hist_.size())
      ROS_DEBUG_STREAM( "First scan after the interval:\t" << scan_hist_[i].header.stamp << "\n");
    else
      ROS_DEBUG_STREAM( "No scans after the interval\n");
    
    if (start_index == past_end_index)
    {
      resp.cloud.header.frame_id = fixed_frame_ ;
      resp.cloud.header.stamp = req.end ;
      resp.cloud.set_pts_size(0) ;
      resp.cloud.set_chan_size(0) ;
    }
    else
    {
      // Note: We are assuming that channel information is consistent across multiple scans. If not, then bad things (segfaulting) will happen
      // Allocate space for the cloud
      unsigned int nC=scan_hist_[start_index].chan.size();

      PointCloud& map=resp.cloud;

      map.header.frame_id = fixed_frame_ ;
      map.header.stamp = req.end ;
      map.set_pts_size(req_pts) ;
      map.set_chan_size(nC) ;

      for(unsigned int iC=0;iC<nC;iC++)
      {
        map.chan[iC].vals.resize(req_pts);
        map.chan[iC].name=scan_hist_[start_index].chan[iC].name;
      }
    
      unsigned int cloud_count = 0 ;
      for (i=start_index; i<past_end_index; i+=downsample_factor_)
      {
        for(unsigned int j=0; j<scan_hist_[i].pts.size(); j+=1)
        {
          map.pts[cloud_count] = scan_hist_[i].pts[j] ;
          for(unsigned int iC=0;iC<nC;iC++)
          {
            map.chan[iC].vals[cloud_count]=scan_hist_[i].chan[iC].vals[j];
          }
          cloud_count++ ;
      }
      map.header.stamp = scan_hist_[i].header.stamp;
    }
  }
  scan_hist_mutex_.unlock() ;

  ROS_DEBUG("Aggregate map results: Aggregated from index %u->%u. BufferSize: %u", start_index, past_end_index, scan_hist_.size()) ;
  ROS_DEBUG_STREAM("Sending data in "<<     resp.cloud.header.frame_id << " frame");
  return true ;
}

private:

};

}

using namespace annotated_planar_patch_map;

int main(int argc, char **argv)
{
  ros::init(argc, argv,"cloud_assembler_srv");

  PcdAssemblerSrv assembler;

  ros::spin();
  
  return 0;
}
