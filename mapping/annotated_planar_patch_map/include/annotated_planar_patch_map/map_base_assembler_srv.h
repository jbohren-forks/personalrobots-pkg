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

// Service
#include "annotated_planar_patch_map/BuildAnnotatedMap.h"
#include "annotated_planar_patch_map/QueryAnnotatedMap.h"
#include "annotated_planar_patch_map/QueryAnnotatedMap.h"

#include "point_cloud_mapping/kdtree/kdtree_ann.h"
//include "point_cloud_mapping/kdtree/kdtree_flann.h"
#include "boost/thread.hpp"
#include "math.h"

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
class MapBaseAssemblerSrv
{
public:
  MapBaseAssemblerSrv(const std::string& node_name) ;
  ~MapBaseAssemblerSrv() ;

  /** \brief Returns the number of points in the current scan
   * \param scan The scan for for which we want to know the number of points
   * \return the number of points in scan
   */
  virtual unsigned int GetPointsInScan(const T& scan) = 0 ;

  /** \brief Converts the current scan into a cloud in the specified fixed frame
   *
   * Note: Once implemented, ConvertToCloud should NOT catch TF exceptions. These exceptions are caught by
   * MapBaseAssemblerSrv, and will be counted for diagnostic information
   * \param fixed_frame_id The name of the frame in which we want cloud_out to be in
   * \param scan_in The scan that we want to convert
   * \param cloud_out The result of transforming scan_in into a cloud in frame fixed_frame_id
   */
  virtual void ConvertToCloud(const std::string& fixed_frame_id, const T& scan_in, F& cloud_out) = 0 ;

protected:
  tf::TransformListener* tf_ ;
  ros::Node node_ ;

private:
  //! \brief Callback function for every time we receive a new scan
  //void scansCallback(const tf::MessageNotifier<T>::MessagePtr& scan_ptr, const T& testA)
  void scansCallback(const boost::shared_ptr<T>& scan_ptr) ;

  //! \brief Service Callback function called whenever we need to build a cloud
  bool buildMap(BuildAnnotatedMap::Request& req, BuildAnnotatedMap::Response& resp) ;

  //! \brief Service Callback function called whenever we need to build a cloud
  bool queryMap(QueryAnnotatedMap::Request& req, QueryAnnotatedMap::Response& resp) ;


  bool isQueryMatchPoly(std::string query,const annotated_map_msgs::TaggedPolygon3D poly);
  bool isQueryMatch(std::string query,std::string name);

  void simplify_output(const F& map_in, F& map_out);

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

  //! \brief Specify the rejection radius. There won't be (m)any patches within this distance in the map. Some duplicates may come from approximate nearest neighbors.
  double max_radius_;


} ;

 template <class T,class F>
   MapBaseAssemblerSrv<T,F>::MapBaseAssemblerSrv(const std::string& node_name) : node_(node_name)
{
  // **** Initialize TransformListener ****
  double tf_cache_time_secs ;
  ros::Node::instance()->param("~tf_cache_time_secs", tf_cache_time_secs, 10.0) ;
  if (tf_cache_time_secs < 0)
    ROS_ERROR("Parameter tf_cache_time_secs<0 (%f)", tf_cache_time_secs) ;

  tf_ = new tf::TransformListener(*ros::Node::instance(), true, ros::Duration(tf_cache_time_secs)) ;
  ROS_INFO("TF Cache Time: %f Seconds", tf_cache_time_secs) ;

  // ***** Set max_scans *****
  const int default_max_scans = 400 ;
  int tmp_max_scans ;
  ros::Node::instance()->param("~max_scans", tmp_max_scans, default_max_scans) ;
  if (tmp_max_scans < 0)
  {
    ROS_ERROR("Parameter max_scans<0 (%i)", tmp_max_scans) ;
    tmp_max_scans = default_max_scans ;
  }
  max_scans_ = tmp_max_scans ;
  ROS_INFO("Max Scans in History: %u", max_scans_) ;
  total_pts_ = 0 ;    // We're always going to start with no points in our history

  // ***** Set fixed_frame *****
  ros::Node::instance()->param("~fixed_frame", fixed_frame_, std::string("ERROR_NO_NAME")) ;
  ROS_INFO("Fixed Frame: %s", fixed_frame_.c_str()) ;
  if (fixed_frame_ == "ERROR_NO_NAME")
    ROS_ERROR("Need to set parameter fixed_frame") ;

  // ***** Set downsample_factor *****
  int tmp_downsample_factor ;
  ros::Node::instance()->param("~downsample_factor", tmp_downsample_factor, 1) ;
  if (tmp_downsample_factor != 1)
  {
    ROS_ERROR("Downsampling is not supported. Parameter downsample_factor should be 1.") ;
    tmp_downsample_factor = 1 ;
  }
  downsample_factor_ = tmp_downsample_factor ;
  ROS_INFO("Downsample Factor: %u", downsample_factor_) ;

  // ***** Set downsample_factor *****
  ros::Node::instance()->param("~rejection_radius", max_radius_, 0.1) ;
  ROS_INFO("Rejection radius: %f", max_radius_) ;

  // ***** Start Services *****
  ros::Node::instance()->advertiseService(ros::Node::instance()->getName()+"/build_map", &MapBaseAssemblerSrv<T,F>::buildMap, this, 0) ;

  ros::Node::instance()->advertiseService(ros::Node::instance()->getName()+"/query_map", &MapBaseAssemblerSrv<T,F>::queryMap, this, 0) ;

  // **** Get the TF Notifier Tolerance ****
  double tf_tolerance_secs ;
  ros::Node::instance()->param("~tf_tolerance_secs", tf_tolerance_secs, 0.0) ;
  if (tf_tolerance_secs < 0)
    ROS_ERROR("Parameter tf_tolerance_secs<0 (%f)", tf_tolerance_secs) ;
  ROS_INFO("tf Tolerance: %f seconds", tf_tolerance_secs) ;

  // ***** Start Listening to Data *****
  scan_notifier_ = new tf::MessageNotifier<T>(tf_, ros::Node::instance(), boost::bind(&MapBaseAssemblerSrv<T,F>::scansCallback, this, _1), "/poly_object_map", fixed_frame_, 10) ;
  scan_notifier_->setTolerance(ros::Duration(tf_tolerance_secs)) ;

}

 template <class T,class F>
   MapBaseAssemblerSrv<T,F>::~MapBaseAssemblerSrv()
{
  delete scan_notifier_ ;
  ros::Node::instance()->unadvertiseService(ros::Node::instance()->getName()+"/build_cloud") ;
  delete tf_ ;
}

 template <class T,class F>
   void MapBaseAssemblerSrv<T,F>::scansCallback(const boost::shared_ptr<T>& scan_ptr)
{
  ROS_INFO("got map") ;
  printf("got map") ;

  const T scan = *scan_ptr ;

  F cur_cloud ;

  // Convert the scan data into a universally known datatype: PointCloud
  try
  {
    ConvertToCloud(fixed_frame_, scan, cur_cloud) ;              // Convert scan into a point cloud
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

 template <class T,class F>
   bool MapBaseAssemblerSrv<T,F>::buildMap(BuildAnnotatedMap::Request& req, BuildAnnotatedMap::Response& resp)
{
  //printf("Starting Service Request\n") ;

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

  unsigned int req_pts = 0 ;                                                          // Keep a total of the points in the current request
  // Find the end of the request
  while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
          scan_hist_[i].header.stamp < req.end )                                      // Don't go past the end-time of the request
  {
    req_pts += (scan_hist_[i].get_polygons_size()) ;
    i += 1 ;
  }
  unsigned int past_end_index = i ;

  if (start_index == past_end_index)
  {
    resp.map.header.frame_id = fixed_frame_ ;
    resp.map.header.stamp = req.end ;
    resp.map.set_polygons_size(0) ;
  }
  else
  {
    // Note: We are assuming that channel information is consistent across multiple scans. If not, then bad things (segfaulting) will happen
    // Allocate space for the cloud
    annotated_map_msgs::TaggedPolygonalMap map;
    
    map.set_polygons_size( req_pts ) ;
    //resp.cloud.header.stamp = req.end ;

    map.header.frame_id = fixed_frame_ ;
    unsigned int cloud_count = 0 ;
    for (i=start_index; i<past_end_index; i+=downsample_factor_)
    {
      for(unsigned int j=0; j<scan_hist_[i].get_polygons_size(); j+=1)
      {
        map.polygons[cloud_count] = scan_hist_[i].polygons[j] ;

        cloud_count++ ;
      }
      map.header.stamp = scan_hist_[i].header.stamp;
    }

    simplify_output(map,resp.map);
  }
  scan_hist_mutex_.unlock() ;

  ROS_DEBUG("Aggregate map results: Aggregated from index %u->%u. BufferSize: %u", start_index, past_end_index, scan_hist_.size()) ;
  return true ;
}

    robot_msgs::Point32
      computeMean (const robot_msgs::Polygon3D& poly)
    {
      robot_msgs::Point32 mean;
      mean.x=0;mean.y=0;mean.z=0;

      unsigned int sz= poly.points.size();;
      for (unsigned int i = 0; i < sz; i++)
      {
        mean.x += poly.points[i].x;
        mean.y += poly.points[i].y;
        mean.z += poly.points[i].z;
      }
      if(sz>0){
	mean.x /= sz;
	mean.y /= sz;
	mean.z /= sz;
      }
      return mean;
    }

 template <class T,class F>
   void MapBaseAssemblerSrv<T,F>::simplify_output(const F& map_in, F& map_out)
   {
     //Compute the center of mass for each planar polygon
     unsigned int num_poly=map_in.polygons.size();
     std::vector<bool> valid_polygons;
     valid_polygons.resize(num_poly);

     robot_msgs::PointCloud centers;
     centers.pts.resize(num_poly);
     centers.chan.resize(0);

     for(unsigned int iPoly=0;iPoly<num_poly;iPoly++){
       centers.pts[iPoly]=computeMean(map_in.polygons[iPoly].polygon);
     }


     //Build kd-tree on centers of mass
     cloud_kdtree::KdTreeANN kd_tree(centers);

     //Eliminate duplicate polygons greedily
     int num_poly_out=0;
     for(unsigned int iPoly=0;iPoly<num_poly;iPoly++){
       std::vector<int> k_indices;
       std::vector<float> k_distances;
       kd_tree.radiusSearch(centers, iPoly, max_radius_, k_indices, k_distances);
       bool bKeep=true;

       for(unsigned int iI=0;iI<k_indices.size();iI++){
	 if(k_indices[iI]<int(iPoly) && (k_distances[iI]<max_radius_)){
	   bKeep=false;
	   break;
	 }
       }
       /*int first_nei=*min_element(k_indices.begin(),k_indices.end());

       if(first_nei<iPoly)
	 {
	   bKeep=false;
	   }*/
       valid_polygons[iPoly]=bKeep;
       if(bKeep)
	 {
	   num_poly_out++;
	 }
     }

     unsigned int iPolyOut=0;
     map_out.set_polygons_size(num_poly_out) ;
     for(unsigned int iPoly=0;iPoly<num_poly;iPoly++)
       {
	 if(valid_polygons[iPoly])
	   {
	     map_out.polygons[iPolyOut]=map_in.polygons[iPoly];
	     iPolyOut++;
	   }
       }

     std::cout << iPolyOut << std::endl;

     map_out.header=map_in.header;
   }

 template <class T,class F>
   bool MapBaseAssemblerSrv<T,F>::isQueryMatchPoly(std::string query,const annotated_map_msgs::TaggedPolygon3D poly)
   {
     for(unsigned int iT=0;iT<poly.get_tags_size();iT++)
       {
	 if(isQueryMatch(query,poly.tags[iT]))
	   {
	     return true;
	   }
       }
     return false;
   }
 template <class T,class F>
   bool MapBaseAssemblerSrv<T,F>::isQueryMatch(std::string query,std::string name)
   {
     if(query=="*")
       return true;
     if(query==name)
       return true;
     return false;	 
   }


 template <class T,class F>
   bool MapBaseAssemblerSrv<T,F>::queryMap(QueryAnnotatedMap::Request& req, QueryAnnotatedMap::Response& resp)
{
  //printf("Starting Service Request\n") ;

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

  unsigned int req_pts = 0 ;                                                          // Keep a total of the points in the current request
  // Find the end of the request
  while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
          scan_hist_[i].header.stamp < req.end )                                      // Don't go past the end-time of the request
  {
    unsigned int nPoly=scan_hist_[i].get_polygons_size();
    for(uint iP=0;iP<nPoly;iP++){
      if(!isQueryMatchPoly(req.query,scan_hist_[i].polygons[iP]))
	continue;
      else
	req_pts ++;
      }
    i += 1 ;
  }
  unsigned int past_end_index = i ;

  if (start_index == past_end_index)
  {
    resp.map.header.frame_id = fixed_frame_ ;
    resp.map.header.stamp = req.end ;
    resp.map.set_polygons_size(0) ;
  }
  else
  {
    // Note: We are assuming that channel information is consistent across multiple scans. If not, then bad things (segfaulting) will happen
    // Allocate space for the cloud
    resp.map.set_polygons_size( req_pts ) ;
    //resp.cloud.header.stamp = req.end ;

    resp.map.header.frame_id = fixed_frame_ ;
    unsigned int cloud_count = 0 ;
    for (i=start_index; i<past_end_index; i+=downsample_factor_)
    {
      for(unsigned int j=0; j<scan_hist_[i].get_polygons_size(); j+=1)
      {
	if(!isQueryMatchPoly(req.query,scan_hist_[i].polygons[j]))
	  continue;

	
        resp.map.polygons[cloud_count] = scan_hist_[i].polygons[j] ;

        cloud_count++ ;
      }
      resp.map.header.stamp = scan_hist_[i].header.stamp;
    }
  }
  scan_hist_mutex_.unlock() ;

  ROS_DEBUG("Aggregate map results: Aggregated from index %u->%u. BufferSize: %u", start_index, past_end_index, scan_hist_.size()) ;
  return true ;
}

}
