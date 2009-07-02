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

//! \author Vijay Pradeep
//! \author Alex Sorokin (modified for annoated maps from point clouds)

#include "ros/node.h"
#include "tf/transform_listener.h"
#include "tf/message_notifier.h"


#include <deque>



#include "point_cloud_mapping/kdtree/kdtree_ann.h"
//include "point_cloud_mapping/kdtree/kdtree_flann.h"
#include "boost/thread.hpp"
#include "math.h"


#include "annotated_planar_patch_map/annotated_map_lib.h"

namespace annotated_planar_patch_map
{

/**
 * \brief Maintains a history of annotated maps and generates an aggregate map upon request
 * \todo Clean up the doxygen part of this header
 *
 * @section parameters ROS Parameters
 *
 * Reads the following parameters from the parameter server
 *  - \b "~tf_cache_time_secs" (double) - The cache time (in seconds) to holds past transforms
 *  - \b "~tf_tolerance_secs (double) - The time (in seconds) to wait after the transform for scan_in is available.
 *  - \b "~max_scans" (unsigned int) - The number of scans to store in the assembler's history, until they're thrown away
 *  - \b "~fixed_frame" (string) - The frame to which received data should immeadiately be transformed to
 *  - \b "~downsampling_factor" (int) - Specifies how often to sample from a scan. 1 preserves all the data. 3 keeps only 1/3 of the points.
 *  - \b "~rejection_radius (double) - Rejection radius. How far each patch should extend.

 *
 *  @section services ROS Service Calls
 *  - \b "~build_map" (BuildAnnotatedMap.srv) - Accumulates scans between begin time and
 *              end time and returns the aggregated data as a point cloud
 */
template<class T,class F>
class GenericMapBaseAssemblerSrv
{
public:
  GenericMapBaseAssemblerSrv() ;
  ~GenericMapBaseAssemblerSrv() ;

  /** \brief Returns the number of points in the current scan
   * \param scan The scan for for which we want to know the number of points
   * \return the number of points in scan
   */
  virtual unsigned int GetPointsInCloud(const F& cloud) = 0 ;

  /** \brief Converts the current scan into a cloud in the specified fixed frame
   *
   * Note: Once implemented, ConvertToCloud should NOT catch TF exceptions. These exceptions are caught by
   * MapBaseAssemblerSrv, and will be counted for diagnostic information
   * \param fixed_frame_id The name of the frame in which we want cloud_out to be in
   * \param scan_in The scan that we want to convert
   * \param cloud_out The result of transforming scan_in into a cloud in frame fixed_frame_id
   */
  virtual void ConvertScanToCloud(const std::string& fixed_frame_id, const T& scan_in, F& cloud_out) = 0 ;

protected:
  tf::TransformListener* tf_ ;
  ros::NodeHandle n_ ;

protected:
  //! \brief Callback function for every time we receive a new scan
  //void scansCallback(const tf::MessageNotifier<T>::MessagePtr& scan_ptr, const T& testA)
  void scansCallback(const boost::shared_ptr<T>& scan_ptr) ;

  //! \brief Service Callback function called whenever we need to build a cloud
  //bool buildMap(SReq& req, SResp& resp)=0;

  tf::MessageNotifier<T>* scan_notifier_ ;

  //! \brief Stores history of scans
  std::deque<F> scan_hist_ ;
  boost::mutex scan_hist_mutex_ ;

  //! \brief The number points currently in the scan history
  unsigned int total_pts_ ;

  //! \brief The max number of scans to store in the scan history
  unsigned int max_scans_ ;

  //! \brief The frame to transform data into upon receipt
  std::string fixed_frame_ ;

  //! \brief Specify how much to downsample the data. A value of 1 preserves all the data. 3 would keep 1/3 of the data.
  unsigned int downsample_factor_ ;


} ;

template <class T,class F>
GenericMapBaseAssemblerSrv<T,F>::GenericMapBaseAssemblerSrv()
{
  // **** Initialize TransformListener ****
  double tf_cache_time_secs ;
  n_.param("~tf_cache_time_secs", tf_cache_time_secs, 10.0) ;
  if (tf_cache_time_secs < 0)
    ROS_ERROR("Parameter tf_cache_time_secs<0 (%f)", tf_cache_time_secs) ;

  tf_ = new tf::TransformListener(ros::Duration(tf_cache_time_secs)) ;
  ROS_INFO("TF Cache Time: %f Seconds", tf_cache_time_secs) ;

  // ***** Set max_scans *****
  const int default_max_scans = 400 ;
  int tmp_max_scans ;
  n_.param("~max_scans", tmp_max_scans, default_max_scans) ;
  if (tmp_max_scans < 0)
  {
    ROS_ERROR("Parameter max_scans<0 (%i)", tmp_max_scans) ;
    tmp_max_scans = default_max_scans ;
  }
  max_scans_ = tmp_max_scans ;
  ROS_INFO("Max Scans in History: %u", max_scans_) ;
  total_pts_ = 0 ;    // We're always going to start with no points in our history

  // ***** Set fixed_frame *****
  n_.param("~fixed_frame", fixed_frame_, std::string("ERROR_NO_NAME")) ;
  ROS_INFO("Fixed Frame: %s", fixed_frame_.c_str()) ;
  if (fixed_frame_ == "ERROR_NO_NAME")
    ROS_ERROR("Need to set parameter fixed_frame") ;

  // ***** Set downsample_factor *****
  int tmp_downsample_factor ;
  n_.param("~downsample_factor", tmp_downsample_factor, 1) ;
  if (tmp_downsample_factor != 1)
  {
    ROS_ERROR("Downsampling is not supported. Parameter downsample_factor should be 1.") ;
    tmp_downsample_factor = 1 ;
  }
  downsample_factor_ = tmp_downsample_factor ;
  ROS_INFO("Downsample Factor: %u", downsample_factor_) ;




  // **** Get the TF Notifier Tolerance ****
  double tf_tolerance_secs ;
  n_.param("~tf_tolerance_secs", tf_tolerance_secs, 0.0) ;
  if (tf_tolerance_secs < 0)
    ROS_ERROR("Parameter tf_tolerance_secs<0 (%f)", tf_tolerance_secs) ;
  ROS_INFO("tf Tolerance: %f seconds", tf_tolerance_secs) ;

  // ***** Start Listening to Data *****
  scan_notifier_ = new tf::MessageNotifier<T>(*tf_, boost::bind(&GenericMapBaseAssemblerSrv<T,F>::scansCallback, this, _1), "scan", fixed_frame_, 10) ;
  scan_notifier_->setTolerance(ros::Duration(tf_tolerance_secs)) ;

}

template <class T,class F>
GenericMapBaseAssemblerSrv<T,F>::~GenericMapBaseAssemblerSrv()
{
  delete scan_notifier_ ;
  //n_.unadvertiseService("~build_cloud") ;
  delete tf_ ;
}

 template <class T,class F>
 void GenericMapBaseAssemblerSrv<T,F>::scansCallback(const boost::shared_ptr<T>& scan_ptr)
{
  ROS_INFO("got map") ;

  const T scan = *scan_ptr ;

  F cur_cloud ;

  // Convert the scan data into a universally known datatype: PointCloud
  try
  {
    ConvertScanToCloud(fixed_frame_, scan, cur_cloud) ;              // Convert scan into a point cloud
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what()) ;
    return ;
  }


  // Add the current scan (now of type PointCloud) into our history of scans
  scan_hist_mutex_.lock() ;
  if (scan_hist_.size() == max_scans_)                           // Is our deque full?
  {
    //total_pts_ -= scan_hist_.front().get_pts_size() ;            // We're removing an elem, so this reduces our total point count
    scan_hist_.pop_front() ;                                     // The front of the deque has the oldest elem, so we can get rid of it
  }
  scan_hist_.push_back(cur_cloud) ;                              // Add the newest scan to the back of the deque
  //total_pts_ += cur_cloud.get_pts_size() ;                       // Add the new scan to the running total of points

  //printf("Scans: %4u  Points: %10u\n", scan_hist_.size(), total_pts_) ;

  scan_hist_mutex_.unlock() ;
}


}
