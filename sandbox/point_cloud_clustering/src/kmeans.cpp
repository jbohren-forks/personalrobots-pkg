/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage
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

#include <point_cloud_clustering/kmeans.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
point_cloud_clustering::KMeans::KMeans(double k_factor, unsigned int nbr_attempts)
{
  if (k_factor < 0.0 || k_factor > 1.0)
  {
    throw "point_cloud_clustering::KMeans() invalid k_factor, must be between [0,1]";
  }

  if (nbr_attempts == 0)
  {
    throw "point_cloud_clustering::KMeans() invalid attempts, must be non-zero";
  }

  k_factor_ = k_factor;
  nbr_attempts_ = nbr_attempts;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int point_cloud_clustering::KMeans::cluster(const sensor_msgs::PointCloud& pt_cloud,
                                            cloud_kdtree::KdTree& pt_cloud_kdtree,
                                            const set<unsigned int>& indices_to_cluster,
                                            map<unsigned int, vector<int> >& created_clusters)
{
  created_clusters.clear();

  // ----------------------------------------------------------
  // Calculate clustering information
  const unsigned int nbr_cluster_samples = indices_to_cluster.size();
  const unsigned int cluster_feature_dim = 3; // x y z

  // ----------------------------------------------------------
  // Verify will create at least 1 cluster
  const int nbr_clusters = static_cast<int> (k_factor_ * static_cast<double> (nbr_cluster_samples));
  if (nbr_clusters < 1)
  {
    ROS_WARN("KMeans::cluster creating no clusters");
    return 0;
  }

  // ----------------------------------------------------------
  // Create matrix for clustering
  // create n-by-d matrix where n are the number of samples and d is the cluster dimension
  // and save the min and max extremas for each dimension
  float feature_matrix[nbr_cluster_samples * cluster_feature_dim];
  unsigned int curr_sample_idx = 0;
  for (set<unsigned int>::const_iterator iter_indices_to_cluster = indices_to_cluster.begin() ; iter_indices_to_cluster
      != indices_to_cluster.end() ; iter_indices_to_cluster++)
  {
    const unsigned int curr_pt_cloud_idx = *iter_indices_to_cluster;
    const geometry_msgs::Point32& curr_pt = pt_cloud.points.at(curr_pt_cloud_idx);

    // offset over previous samples in feature_matrix
    const unsigned int curr_offset = curr_sample_idx * cluster_feature_dim;

    // copy xyz coordinates
    feature_matrix[curr_offset] = curr_pt.x;
    feature_matrix[curr_offset + 1] = curr_pt.y;
    feature_matrix[curr_offset + 2] = curr_pt.z;

    curr_sample_idx++;
  }

  // ----------------------------------------------------------
  // Do kmeans clustering
  int cluster_labels[nbr_cluster_samples];
  RunKMeansPlusPlus(nbr_cluster_samples, nbr_clusters, cluster_feature_dim, feature_matrix,
      nbr_attempts_, NULL, cluster_labels);

  // ----------------------------------------------------------
  // Associate each point with its cluster label
  curr_sample_idx = 0;
  for (set<unsigned int>::const_iterator iter_indices_to_cluster = indices_to_cluster.begin() ; iter_indices_to_cluster
      != indices_to_cluster.end() ; iter_indices_to_cluster++)
  {
    // retrieve the current point index and its cluster label
    const unsigned int curr_pt_cloud_idx = *iter_indices_to_cluster;
    const unsigned int curr_cluster_label = starting_label_ + cluster_labels[curr_sample_idx];

    // Instantiate container if never encountered label before
    if (created_clusters.count(curr_cluster_label) == 0)
    {
      created_clusters[curr_cluster_label] = vector<int> ();
    }

    // Associate point index with the cluster label
    created_clusters.find(curr_cluster_label)->second.push_back(
        static_cast<int> (curr_pt_cloud_idx));

    curr_sample_idx++;
  }

  return 0;
}
