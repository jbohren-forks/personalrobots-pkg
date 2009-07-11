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

//! \author Alex Sorokin 

#ifndef PCD_NOVELTY_NOVELTY_ESTIMATOR_H
#define PCD_NOVELTY_NOVELTY_ESTIMATOR_H

// ROS core
#include <ros/ros.h>
// ROS messages
#include <robot_msgs/PointCloud.h>


// Cloud kd-tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/geometry/projections.h>

#include <angles/angles.h>

#include <vector>
#include <deque>


namespace pcd_novelty
{


class PcdHolder
{
public:
  robot_msgs::PointCloud cloud_3d_;
  robot_msgs::PointCloud cloud_2d_;
  boost::shared_ptr<cloud_kdtree::KdTreeANN> kdtree_2d_;
  boost::shared_ptr<cloud_kdtree::KdTreeANN> kdtree_3d_;
};

typedef std::deque<boost::shared_ptr<PcdHolder> > PcdHolderDeque;


/** \brief The estimator of which points in the point cloud contain new information
 */
class NoveltyEstimator
{
protected:
  ros::NodeHandle n_;
  boost::mutex proc_mutex_;
  

public:  
  //FYI std::deque<boost::shared_ptr<PcdHolder> > 
  PcdHolderDeque clouds_;

public:
  double max_2d_search_radius_;
  double max_3d_search_radius_;

  int num_required_nei_2d_ ;
  int num_required_nei_3d_;

  std::string novelty_channel_;




public:
  NoveltyEstimator();

  /** \brief Add the cloud to cloud history and create supporting search structures.
   * \param cloud the cloud to add
   */

  void addCloudToHistory(const robot_msgs::PointCloud &cloud);

  /** \brief Allocate new novelty channel in the point cloud
   * \param The point cloud to get the "novelty" channel
   * \param The pointer to the channel once it's allocated
   */
  void allocateNoveltyChannel(robot_msgs::PointCloud& point_cloud,std::vector<float>** ptr_channel);

  /** \brief Allocate new novelty channel in the point cloud
   * \param The point cloud to get the "novelty" channel
   * \param The pointer to the channel once it's allocated
   */
  void computeNovelty(const robot_msgs::PointCloud& point_cloud,std::vector<float> &novelty_holder);


};



}


#endif
