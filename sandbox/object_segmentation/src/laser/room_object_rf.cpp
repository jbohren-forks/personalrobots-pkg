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

#include <object_segmentation/laser/room_object_rf.h>

using namespace std;

const std::string RoomObjectRF::CHANNEL_LABEL = "label";
const std::string RoomObjectRF::CHANNEL_INTENSITY = "intensities";

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
RoomObjectRF::RoomObjectRF()
{
  vector<Descriptor3D*> node_feature_descriptors;
  vector<vector<Descriptor3D*> > clique_set_feature_descriptors;
  vector<vector<pair<bool, point_cloud_clustering::PointCloudClustering*> > >
      clique_set_clusterings;

  // -----------------------------------------
  // Node Features
  SpectralAnalysis* sa_nodes = new SpectralAnalysis(0.15); // 0.15 m ~ 6 inch
  ShapeSpectral* ss_nodes = new ShapeSpectral(*sa_nodes);
  Position* p_nodes = new Position(-0.58);
  Curvature* curv_nodes = new Curvature(*sa_nodes);
  OrientationNormal* on_nodes = new OrientationNormal(0, 0, 1.0, *sa_nodes);
  OrientationTangent* ot_nodes = new OrientationTangent(0, 0, 1.0, *sa_nodes);
  SpectralAnalysis* sa2_nodes = new SpectralAnalysis(0.2413); // 9.5 inch
  ShapeSpectral* ss2_nodes = new ShapeSpectral(*sa2_nodes);
  Channel* ch_nodes = new Channel(RoomObjectRF::CHANNEL_INTENSITY);
  SpinImageCustom* sic_nodes = new SpinImageCustom(0, 0, 1.0, 0.0762, 0.0762, 5, 2, false);

  //
  node_feature_descriptors.push_back(ss_nodes);
  node_feature_descriptors.push_back(p_nodes);
  node_feature_descriptors.push_back(curv_nodes);
  node_feature_descriptors.push_back(on_nodes);
  node_feature_descriptors.push_back(ot_nodes);
  node_feature_descriptors.push_back(ss2_nodes);
  node_feature_descriptors.push_back(ch_nodes);
  node_feature_descriptors.push_back(sic_nodes);

  // -----------------------------------------
  // Clique-set 0 features
  SpectralAnalysis* sa_cluster_cs0 = new SpectralAnalysis(-1);
  SpectralAnalysis* sa_radius_cs0 = new SpectralAnalysis(0.39624); // 1.3 ft
  ShapeSpectral* ss_cluster_cs0 = new ShapeSpectral(*sa_cluster_cs0);
  ShapeSpectral* ss_radius_cs0 = new ShapeSpectral(*sa_radius_cs0);
  OrientationNormal* on_cluster_cs0 = new OrientationNormal(0, 0, 1.0, *sa_cluster_cs0);
  SpinImageCustom* sic_cs0 = new SpinImageCustom(0, 0, 1.0, 0.0762, 0.0762, 11, 4, false);
  SpinImageNormal* sin_cs0 = new SpinImageNormal(0.0762, 0.0762, 11, 4, false, *sa_cluster_cs0);
  BoundingBoxSpectral* bbs_radius_cs0 = new BoundingBoxSpectral(-1.0, *sa_radius_cs0);
  vector<Descriptor3D*> cs0_feature_descriptors;
  cs0_feature_descriptors.push_back(ss_cluster_cs0);
  cs0_feature_descriptors.push_back(ss_radius_cs0);
  cs0_feature_descriptors.push_back(on_cluster_cs0);
  cs0_feature_descriptors.push_back(sic_cs0);
  cs0_feature_descriptors.push_back(sin_cs0);
  cs0_feature_descriptors.push_back(bbs_radius_cs0);
  //
  clique_set_feature_descriptors.push_back(cs0_feature_descriptors);

  // -----------------------------------------
  // Clique-set 0 clustering (kmeans)
  point_cloud_clustering::KMeans* kmeans0_cs0 = new point_cloud_clustering::KMeans(0.005, 2); // TODO 1
  point_cloud_clustering::KMeans* kmeans1_cs0 = new point_cloud_clustering::KMeans(0.004, 2); // 1
  vector<pair<bool, point_cloud_clustering::PointCloudClustering*> > cs0_clusterings(2);
  cs0_clusterings[0].first = false; // true indicates to cluster over only the nodes
  cs0_clusterings[0].second = kmeans0_cs0;
  cs0_clusterings[1].first = false; // true indicates to cluster over only the nodes
  cs0_clusterings[1].second = kmeans1_cs0;
  //
  clique_set_clusterings.push_back(cs0_clusterings);

  // -----------------------------------------
  // Clique-set 1 features (edges)
  vector<Descriptor3D*> cs1_feature_descriptors; // intentionally empty since using edges
  clique_set_feature_descriptors.push_back(cs1_feature_descriptors);

  // -----------------------------------------
  // Clique-set 1 clustering (edges)
  point_cloud_clustering::PairwiseNeighbors* pn_cs1 =
      new point_cloud_clustering::PairwiseNeighbors(0.15, 4);
  vector<pair<bool, point_cloud_clustering::PointCloudClustering*> > cs1_clusterings(1);
  cs1_clusterings[0].first = true; // true indicates to cluster over only the nodes
  cs1_clusterings[0].second = pn_cs1;
  clique_set_clusterings.push_back(cs1_clusterings);

  // -----------------------------------------
  // Initialize RFCreator3D with the above features and clustering
  rf_creator_3d_ = new RFCreator3D(node_feature_descriptors, clique_set_feature_descriptors,
      clique_set_clusterings);

  // 1 inch = 0.0254 meters
  voxel_x_ = 0.0254;
  voxel_y_ = 0.0254;
  voxel_z_ = 0.0254;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
boost::shared_ptr<RandomField> RoomObjectRF::createRandomField(const string& fname_pcd, bool use_only_labeled)
{
  // --------------------------------------------------
  // Load point cloud
  sensor_msgs::PointCloud full_point_cloud;
  ROS_INFO("Loading point cloud....");
  if (cloud_io::loadPCDFile(fname_pcd.c_str(), full_point_cloud) == -1)
  {
    ROS_FATAL("Could not load: %s", fname_pcd.c_str());
    abort();
  }
  ROS_INFO("done.");

  // --------------------------------------------------
  // Downsample point cloud
  sensor_msgs::PointCloud ds_point_cloud;
  vector<unsigned int> ds_labels;
  downsamplePointCloud(full_point_cloud, ds_point_cloud, ds_labels);

  // --------------------------------------------------
  // Create random field from down sampled point cloud
  boost::shared_ptr<RandomField> created_rf = rf_creator_3d_->createRandomField(ds_point_cloud,
      ds_labels, use_only_labeled);
  return created_rf;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void RoomObjectRF::downsamplePointCloud(sensor_msgs::PointCloud& full_point_cloud,
                                        sensor_msgs::PointCloud& ds_point_cloud,
                                        std::vector<unsigned int>& ds_labels)
{
  // HARD CODE
  ROS_WARN("IGNORING LABEL 16 and 15");
  set<unsigned int> ignore_labels;
  ignore_labels.insert(16);
  ignore_labels.insert(15);

  // not guaranteed to exist
  int label_channel =
      cloud_geometry::getChannelIndex(full_point_cloud, RoomObjectRF::CHANNEL_LABEL);

  int intensity_channel = cloud_geometry::getChannelIndex(full_point_cloud,
      RoomObjectRF::CHANNEL_INTENSITY);
  if (intensity_channel < 0)
  {
    ROS_FATAL("Need channel %s to proceed", RoomObjectRF::CHANNEL_INTENSITY.c_str());
    abort();
  }

  // create kdtree of original point cloud to find downsample's closest point
  cloud_kdtree::KdTreeANN full_cloud_kdtree(full_point_cloud);

  // Downsample the cloud
  geometry_msgs::Point voxel_dim;
  voxel_dim.x = voxel_x_;
  voxel_dim.y = voxel_y_;
  voxel_dim.z = voxel_z_;
  cloud_geometry::downsamplePointCloud(full_point_cloud, ds_point_cloud, voxel_dim);
  int nbr_downsampled_pts = ds_point_cloud.points.size();

  unsigned int ds_nbr_chans = ds_point_cloud.get_channels_size();
  ds_point_cloud.set_channels_size(ds_nbr_chans + 1);
  ds_point_cloud.channels[ds_nbr_chans].name = RoomObjectRF::CHANNEL_INTENSITY;
  ds_point_cloud.channels[ds_nbr_chans].set_values_size(nbr_downsampled_pts);

  // Initialize all down sample points' labels to RandomField::UNKNOWN_LABEL
  ds_labels.assign(nbr_downsampled_pts, RandomField::UNKNOWN_LABEL);

  // For each downsampled point, find its closest point in the original full point cloud
  // and copy label if available
  for (int i = 0 ; i < nbr_downsampled_pts ; i++)
  {
    const geometry_msgs::Point32& curr_ds_pt = ds_point_cloud.points[i];

    // find the voxel's centroid closest point in the full point cloud
    vector<int> k_indices;
    vector<float> k_distances;
    full_cloud_kdtree.nearestKSearch(curr_ds_pt, 1, k_indices, k_distances);
    const unsigned int closest_full_idx = static_cast<unsigned int> (k_indices.at(0));

    ds_point_cloud.channels[ds_nbr_chans].values[i]
        = full_point_cloud.channels[intensity_channel].values[closest_full_idx];

    // if present, assign the down sampled point the label of the closest point in the original point cloud
    if (label_channel >= 0)
    {
      ds_labels[i] = full_point_cloud.channels[label_channel].values[closest_full_idx];
      if (ignore_labels.count(ds_labels[i]) != 0)
      {
        ds_labels[i] = RandomField::UNKNOWN_LABEL;
      }
    }
  }
}
