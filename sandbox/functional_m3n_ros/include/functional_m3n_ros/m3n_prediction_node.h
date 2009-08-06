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

//! \author Daniel Munoz, Alex Sorokin 

#ifndef M3N_PREDICTION_NODE_H
#define M3N_PREDICTION_NODE_H

// ROS core
#include <ros/ros.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>


// Cloud kd-tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>

#include <functional_m3n/example/pt_cloud_rf_creator.h>

#include <functional_m3n/m3n_model.h>

#include <functional_m3n_ros/SetModel.h>
#include <functional_m3n_ros/QueryPerformanceStats.h>

namespace m3n
{

class PredictionNode
{
protected:
  ros::NodeHandle n_;
  
public:
  PredictionNode();

  void cloudCallback(const sensor_msgs::PointCloudConstPtr& the_cloud);
  bool setModel(functional_m3n_ros::SetModel::Request  &req,
		functional_m3n_ros::SetModel::Response &res );

  bool queryPerformanceStats(
			     functional_m3n_ros::QueryPerformanceStats::Request  &req,
			     functional_m3n_ros::QueryPerformanceStats::Response &res );

  boost::shared_ptr<PtCloudRFCreator> rf_creator_;
  bool use_colors_;

  unsigned int nbr_clique_sets;
  M3NModel m3n_model2;
  bool has_model_;

  std::string model_file_name_;
  std::string ground_truth_channel_name_;

  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  ros::ServiceServer set_model_svc_;
  ros::ServiceServer query_perf_stats_svc_;

  unsigned int nbr_correct;
  unsigned int nbr_gt;
  double total_accuracy;

  void computeClassificationRates(const vector<float>& inferred_labels, const vector<float>& gt_labels, 
				  const vector<unsigned int>& labels,
				  unsigned int& nbr_correct,
				  unsigned int& nbr_gt,
				  double& accuracy);

    
};



}


#endif
