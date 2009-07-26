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

int point_cloud_clustering::pcKMeans(const robot_msgs::PointCloud& pt_cloud,
                                     const set<unsigned int>& ignore_indices,
                                     const kmeans_params_t& kmeans_params,
                                     map<unsigned int, vector<float> >& cluster_xyz_centroids,
                                     map<unsigned int, vector<int> >& cluster_pt_indices)

{
  double kmeans_factor = kmeans_params.factor;
  double kmeans_accuracy = kmeans_params.accuracy;
  int kmeans_max_iter = kmeans_params.max_iter;
  vector<unsigned int> kmeans_channel_indices = kmeans_params.channel_indices;

  // ----------------------------------------------------------
  // Cluster of xyz coordinates and the specified channel dimensions
  unsigned int cluster_feature_dim = 3;
  for (unsigned int i = 0 ; i < kmeans_channel_indices.size() ; i++)
  {
    if (kmeans_channel_indices[i] >= pt_cloud.chan.size())
    {
      ROS_ERROR("Channel index %u exceeds number of channels %u", kmeans_channel_indices[i],
          pt_cloud.chan.size());
      return -1;
    }
    cluster_feature_dim += pt_cloud.chan[kmeans_channel_indices[i]].vals.size();
  }

  unsigned int nbr_total_pts = pt_cloud.pts.size();
  unsigned int nbr_cluster_samples = nbr_total_pts - ignore_indices.size();

  if (nbr_cluster_samples == 0 || nbr_total_pts == 0)
  {
    ROS_ERROR("There are no samples to cluster on");
    return -1;
  }

  // ----------------------------------------------------------
  // Create matrix for clustering
  // create n-by-d matrix where n are the number of samples and d is the feature dimension
  // and save the min and max extremas for each dimension
  float* feature_matrix = static_cast<float*> (malloc(nbr_cluster_samples * cluster_feature_dim
      * sizeof(float)));
  unsigned int nbr_skipped = 0;
  for (unsigned int i = 0 ; i < nbr_total_pts ; i++)
  {
    if (ignore_indices.count(i) != 0)
    {
      nbr_skipped++;
      continue;
    }

    unsigned int curr_sample_idx = i - nbr_skipped;

    // offset over previous samples in feature_matrix
    unsigned int curr_offset = curr_sample_idx * cluster_feature_dim;

    // copy xyz coordinates
    feature_matrix[curr_offset] = pt_cloud.pts[i].x;
    feature_matrix[curr_offset + 1] = pt_cloud.pts[i].y;
    feature_matrix[curr_offset + 2] = pt_cloud.pts[i].z;

    // offset past xyz coordinates
    curr_offset += 3;

    // cluster the channel values
    // TODO normalize these values?
    for (unsigned int j = 0 ; j < kmeans_channel_indices.size() ; j++)
    {
      const vector<float>& chan_vals = pt_cloud.chan[kmeans_channel_indices[j]].vals;
      for (unsigned int k = 0 ; k < chan_vals.size() ; k++)
      {
        feature_matrix[curr_offset + k] = chan_vals[k];
      }
      curr_offset += chan_vals.size();
    }
  }

  // ----------------------------------------------------------
  // Do kmeans clustering
  CvMat cv_feature_matrix;
  cvInitMatHeader(&cv_feature_matrix, nbr_cluster_samples, cluster_feature_dim, CV_32F, feature_matrix);
  CvMat* cluster_labels = cvCreateMat(nbr_cluster_samples, 1, CV_32SC1);
  int nbr_clusters = static_cast<int> (static_cast<double> (nbr_cluster_samples) * kmeans_factor);
  cvKMeans2(&cv_feature_matrix, nbr_clusters, cluster_labels, cvTermCriteria(CV_TERMCRIT_ITER
      + CV_TERMCRIT_EPS, kmeans_max_iter, kmeans_accuracy));

  // ----------------------------------------------------------
  // Associate each point with its cluster label
  cluster_xyz_centroids.clear();
  cluster_pt_indices.clear();
  nbr_skipped = 0;
  for (unsigned int i = 0 ; i < nbr_total_pts ; i++)
  {
    if (ignore_indices.count(i) != 0)
    {
      nbr_skipped++;
      continue;
    }

    // Important: Some points may have been skipped when creating matrix for clustering,
    // so we need to adjust the index appropriately
    unsigned int curr_sample_idx = i - nbr_skipped;

    // Get the cluster label of the current point
    unsigned int curr_cluster_label = static_cast<unsigned int> (cluster_labels->data.i[curr_sample_idx]);

    // Instantiate container if never encountered label before
    if (cluster_xyz_centroids.count(curr_cluster_label) == 0)
    {
      cluster_xyz_centroids[curr_cluster_label] = vector<float> (3, 0.0);
      cluster_pt_indices[curr_cluster_label] = vector<int> ();
    }

    // accumulate total xyz coordinates in cluster
    cluster_xyz_centroids[curr_cluster_label][0] += pt_cloud.pts[i].x;
    cluster_xyz_centroids[curr_cluster_label][1] += pt_cloud.pts[i].y;
    cluster_xyz_centroids[curr_cluster_label][2] += pt_cloud.pts[i].z;

    // associate node with the cluster label
    cluster_pt_indices[curr_cluster_label].push_back(static_cast<int> (i));
  }

  // ----------------------------------------------------------
  // Finalize xyz centroid for each cluster
  map<unsigned int, vector<float> >::iterator iter_cluster_xyz_centroids;
  for (iter_cluster_xyz_centroids = cluster_xyz_centroids.begin(); iter_cluster_xyz_centroids
      != cluster_xyz_centroids.end() ; iter_cluster_xyz_centroids++)
  {
    float curr_cluster_nbr_pts =
        static_cast<float> (cluster_pt_indices[iter_cluster_xyz_centroids->first].size());
    iter_cluster_xyz_centroids->second[0] /= curr_cluster_nbr_pts;
    iter_cluster_xyz_centroids->second[1] /= curr_cluster_nbr_pts;
    iter_cluster_xyz_centroids->second[2] /= curr_cluster_nbr_pts;
  }

  // ----------------------------------------------------------
  // Cleanup
  cvReleaseMat(&cluster_labels);
  free(feature_matrix);

  return 0;
}
