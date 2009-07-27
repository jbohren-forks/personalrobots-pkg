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

#include <point_cloud_clustering/kmeans_clustering.h>

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int point_cloud_clustering::KMeans::setParameters(double k_factor, double accuracy, int max_iter)
{
  if (k_factor < 0.0 || k_factor > 1.0)
  {
    ROS_ERROR("Invalid kmeans factor");
    return -1;
  }

  if (accuracy < 0.0 || accuracy > 1.0)
  {
    ROS_ERROR("Invalid kmeans accuracy");
    return -1;
  }

  if (max_iter < 1)
  {
    ROS_ERROR("Invalid kmeans max iteration");
    return -1;
  }

  k_factor_ = k_factor;
  accuracy_ = accuracy;
  max_iter_ = max_iter;
  parameters_defined_ = true;
  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int point_cloud_clustering::KMeans::cluster(const robot_msgs::PointCloud& pt_cloud,
                                            cloud_kdtree::KdTree& pt_cloud_kdtree,
                                            const set<unsigned int>& indices_to_cluster,
                                            map<unsigned int, vector<int> >& created_clusters,
                                            map<unsigned int, vector<float> >* cluster_centroids)
{
  if (parameters_defined_ == false)
  {
    return -1;
  }

  unsigned int nbr_total_pts = pt_cloud.pts.size();
  unsigned int nbr_cluster_samples = indices_to_cluster.size();
  unsigned int cluster_feature_dim = 3; // x y z
  if (nbr_cluster_samples == 0 || nbr_total_pts == 0)
  {
    ROS_ERROR("There are no samples to cluster on");
    return -1;
  }

  // ----------------------------------------------------------
  // Create matrix for clustering
  // create n-by-d matrix where n are the number of samples and d is the cluster dimension
  // and save the min and max extremas for each dimension
  float* feature_matrix = static_cast<float*> (malloc(nbr_cluster_samples * cluster_feature_dim
      * sizeof(float)));
  set<unsigned int>::const_iterator iter_indices_to_cluster;
  unsigned int curr_sample_idx = 0;
  for (iter_indices_to_cluster = indices_to_cluster.begin(); iter_indices_to_cluster
      != indices_to_cluster.end() ; iter_indices_to_cluster++)
  {
    unsigned int curr_idx = *iter_indices_to_cluster;
    if (curr_idx >= nbr_total_pts)
    {
      ROS_ERROR("Invalid index to cluster: %u", curr_idx);
      free(feature_matrix);
      return -1;
    }

    // offset over previous samples in feature_matrix
    unsigned int curr_offset = curr_sample_idx * cluster_feature_dim;

    // copy xyz coordinates
    feature_matrix[curr_offset] = pt_cloud.pts[curr_idx].x;
    feature_matrix[curr_offset + 1] = pt_cloud.pts[curr_idx].y;
    feature_matrix[curr_offset + 2] = pt_cloud.pts[curr_idx].z;

    curr_sample_idx++;
  }

  // ----------------------------------------------------------
  // Do kmeans clustering
  CvMat cv_feature_matrix;
  cvInitMatHeader(&cv_feature_matrix, nbr_cluster_samples, cluster_feature_dim, CV_32F, feature_matrix);
  CvMat* cluster_labels = cvCreateMat(nbr_cluster_samples, 1, CV_32SC1);
  int nbr_clusters = static_cast<int> (static_cast<double> (nbr_cluster_samples) * k_factor_);
  cvKMeans2(&cv_feature_matrix, nbr_clusters, cluster_labels, cvTermCriteria(CV_TERMCRIT_ITER
      + CV_TERMCRIT_EPS, max_iter_, accuracy_));

  // ----------------------------------------------------------
  // Associate each point with its cluster label
  created_clusters.clear();
  curr_sample_idx = 0;
  for (iter_indices_to_cluster = indices_to_cluster.begin(); iter_indices_to_cluster
      != indices_to_cluster.end() ; iter_indices_to_cluster++)
  {
    unsigned int curr_pt_cloud_idx = *iter_indices_to_cluster;
    unsigned int curr_cluster_label = static_cast<unsigned int> (cluster_labels->data.i[curr_sample_idx]);

    // Instantiate container if never encountered label before
    if (created_clusters.count(curr_cluster_label) == 0)
    {
      created_clusters[curr_cluster_label] = vector<int> ();
      if (cluster_centroids != NULL)
      {
        (*cluster_centroids)[curr_cluster_label] = vector<float> (3, 0.0);
      }
    }

    // Associate point index with the cluster
    created_clusters[curr_cluster_label].push_back(static_cast<int> (curr_pt_cloud_idx));

    // Record centroid information, if necessary
    if (cluster_centroids != NULL)
    {
      // accumulate total xyz coordinates in cluster
      (*cluster_centroids)[curr_cluster_label][0] += pt_cloud.pts[curr_pt_cloud_idx].x;
      (*cluster_centroids)[curr_cluster_label][1] += pt_cloud.pts[curr_pt_cloud_idx].y;
      (*cluster_centroids)[curr_cluster_label][2] += pt_cloud.pts[curr_pt_cloud_idx].z;
    }

    curr_sample_idx++;
  }

  // ----------------------------------------------------------
  // Finalize xyz centroid, if necessary
  if (cluster_centroids != NULL)
  {
    map<unsigned int, vector<float> >::iterator iter_cluster_centroids;
    for (iter_cluster_centroids = cluster_centroids->begin(); iter_cluster_centroids
        != cluster_centroids->end() ; iter_cluster_centroids++)
    {
      float curr_cluster_nbr_pts =
          static_cast<float> (created_clusters[iter_cluster_centroids->first].size());
      iter_cluster_centroids->second[0] /= curr_cluster_nbr_pts;
      iter_cluster_centroids->second[1] /= curr_cluster_nbr_pts;
      iter_cluster_centroids->second[2] /= curr_cluster_nbr_pts;
    }
  }

  // ----------------------------------------------------------
  // Cleanup
  cvReleaseMat(&cluster_labels);
  free(feature_matrix);

  return 0;
}
