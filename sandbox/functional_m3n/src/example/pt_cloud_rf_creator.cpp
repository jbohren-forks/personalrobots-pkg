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

#include <functional_m3n/example/pt_cloud_rf_creator.h>

using namespace std;

// --------------------------------------------------------------
/* Features for the nodes */
// --------------------------------------------------------------
vector<Descriptor3D*> initNodeFeatures()
{

  //
  SpectralShape* spectral_shape = new SpectralShape();
  spectral_shape->setSpectralRadius(0.15);

  //
  Orientation* orientation = new Orientation();
  orientation->useSpectralInformation(spectral_shape);
  orientation->useNormalOrientation(0.0, 0.0, 1.0);
  orientation->useTangentOrientation(0.0, 0.0, 1.0);

  Position* position = new Position();

  vector<Descriptor3D*> node_feature_descriptors;
  node_feature_descriptors.push_back(spectral_shape);
  node_feature_descriptors.push_back(orientation);
  node_feature_descriptors.push_back(position);

  /*
   //
   Position* position = new Position();

   SpectralAnalysis* sa = new SpectralAnalysis(0.15);
   ShapeSpectral* spectral_shape = new ShapeSpectral(*sa);
   OrientationTangent* o_tangent = new OrientationTangent(0, 0, 1.0, *sa);
   OrientationNormal* o_normal = new OrientationNormal(0, 0, 1.0, *sa);

   // ---------------
   vector<Descriptor3D*> node_feature_descriptors;
   node_feature_descriptors.push_back(spectral_shape);
   node_feature_descriptors.push_back(o_tangent);
   node_feature_descriptors.push_back(o_normal);
   node_feature_descriptors.push_back(position);
   */
  return node_feature_descriptors;
}

// --------------------------------------------------------------
/* Features for clique set 0 */
// --------------------------------------------------------------
vector<Descriptor3D*> initCS0Features()
{

  //
  SpectralShape* spectral_shape = new SpectralShape();
  spectral_shape->setSpectralRadius(0.2286);

  //
  Orientation* orientation = new Orientation();
  orientation->useSpectralInformation(spectral_shape);
  orientation->useNormalOrientation(0.0, 0.0, 1.0);
  orientation->useTangentOrientation(0.0, 0.0, 1.0);

  //
  Position* position = new Position();

  //
  SpinImage* spin_image = new SpinImage();
  spin_image->setAxisCustom(0.0, 0.0, 1.0);
  spin_image->setImageDimensions(0.0762, 0.0762, 5, 4);

  //
  BoundingBox* bounding_box = new BoundingBox(true, false);
  bounding_box->useSpectralInformation(spectral_shape);
  bounding_box->setBoundingBoxRadius(-1.0);

  vector<Descriptor3D*> cs0_feature_descriptors;
  cs0_feature_descriptors.push_back(spectral_shape);
  cs0_feature_descriptors.push_back(orientation);
  cs0_feature_descriptors.push_back(position);
  cs0_feature_descriptors.push_back(spin_image);
  cs0_feature_descriptors.push_back(bounding_box);

  /*
   SpectralAnalysis* sa = new SpectralAnalysis(0.2286);
   ShapeSpectral* spectral_shape = new ShapeSpectral(*sa);
   OrientationTangent* o_tangent = new OrientationTangent(0, 0, 1.0, *sa);
   OrientationNormal* o_normal = new OrientationNormal(0, 0, 1.0, *sa);
   Position* position = new Position();
   SpinImageCustom* spin_image = new SpinImageCustom(0, 0, 1.0, 0.0762, 0.0762, 5, 4, false);
   BoundingBoxSpectral* bbox_spectral = new BoundingBoxSpectral(-1.0, *sa);

   // ---------------
   vector<Descriptor3D*> cs0_feature_descriptors;
   cs0_feature_descriptors.push_back(spectral_shape);
   cs0_feature_descriptors.push_back(o_tangent);
   cs0_feature_descriptors.push_back(o_normal);
   cs0_feature_descriptors.push_back(position);
   cs0_feature_descriptors.push_back(spin_image);
   cs0_feature_descriptors.push_back(bbox_spectral);
   */

  return cs0_feature_descriptors;
}

// --------------------------------------------------------------
/* Features for clique set 1 */
// --------------------------------------------------------------
vector<Descriptor3D*> initCS1Features()
{

  //
  SpectralShape* spectral_shape = new SpectralShape();
  spectral_shape->setSpectralRadius(-1);

  //
  Orientation* orientation = new Orientation();
  orientation->useSpectralInformation(spectral_shape);
  orientation->useNormalOrientation(0.0, 0.0, 1.0);
  orientation->useTangentOrientation(0.0, 0.0, 1.0);

  //
  Position* position = new Position();

  //
  SpinImage* spin_image = new SpinImage();
  spin_image->useSpectralInformation(spectral_shape);
  spin_image->setAxisNormal();
  spin_image->setImageDimensions(0.0762, 0.0762, 7, 5);

  //
  BoundingBox* bounding_box = new BoundingBox(true, false);
  bounding_box->useSpectralInformation(spectral_shape);
  bounding_box->setBoundingBoxRadius(-1.0);

  vector<Descriptor3D*> cs1_feature_descriptors;
  cs1_feature_descriptors.push_back(spectral_shape);
  cs1_feature_descriptors.push_back(orientation);
  cs1_feature_descriptors.push_back(position);
  cs1_feature_descriptors.push_back(spin_image);
  cs1_feature_descriptors.push_back(bounding_box);

  /*
   SpectralAnalysis* sa = new SpectralAnalysis(-1);
   ShapeSpectral* spectral_shape = new ShapeSpectral(*sa);
   OrientationTangent* o_tangent = new OrientationTangent(0, 0, 1.0, *sa);
   OrientationNormal* o_normal = new OrientationNormal(0, 0, 1.0, *sa);
   Position* position = new Position();
   SpinImageNormal* spin_image = new SpinImageNormal(0.0762, 0.0762, 7, 5, false, *sa);
   BoundingBoxSpectral* bbox_spectral = new BoundingBoxSpectral(-1.0, *sa);

   // ---------------
   vector<Descriptor3D*> cs1_feature_descriptors;
   cs1_feature_descriptors.push_back(spectral_shape);
   cs1_feature_descriptors.push_back(o_tangent);
   cs1_feature_descriptors.push_back(o_normal);
   cs1_feature_descriptors.push_back(position);
   cs1_feature_descriptors.push_back(spin_image);
   cs1_feature_descriptors.push_back(bbox_spectral);
   */
  return cs1_feature_descriptors;
}

// --------------------------------------------------------------
/* Clusters for clique set 0 */
// --------------------------------------------------------------
std::vector<std::pair<bool, point_cloud_clustering::PointCloudClustering*> > initCS0Clusters()
{
  //
  point_cloud_clustering::KMeans* kmeans_cluster = new point_cloud_clustering::KMeans();
  kmeans_cluster->setParameters(0.003, 1.0, 10);

  // ---------------
  std::vector<std::pair<bool, point_cloud_clustering::PointCloudClustering*> > cs0_clusterings(1);
  bool cluster_only_nodes = false;
  cs0_clusterings[0].first = cluster_only_nodes;
  cs0_clusterings[0].second = kmeans_cluster;
  return cs0_clusterings;
}

// --------------------------------------------------------------
/* Clusters for clique set 1 */
// --------------------------------------------------------------
std::vector<std::pair<bool, point_cloud_clustering::PointCloudClustering*> > initCS1Clusters()
{
  //
  point_cloud_clustering::KMeans* kmeans_cluster = new point_cloud_clustering::KMeans();
  kmeans_cluster->setParameters(0.001, 1.0, 10);
  // ---------------
  std::vector<std::pair<bool, point_cloud_clustering::PointCloudClustering*> > cs1_clusterings(1);
  bool cluster_only_nodes = false;
  cs1_clusterings[0].first = cluster_only_nodes;
  cs1_clusterings[0].second = kmeans_cluster;
  return cs1_clusterings;
}

// -------------------------------------------------------------------------------------
// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// -------------------------------------------------------------------------------------

// --------------------------------------------------------------
/* Instantiate the parameters for the how the random field is constructed */
// --------------------------------------------------------------
void PtCloudRFCreator::createDescriptors()
{
  const unsigned int nbr_clique_sets = 2;

  for (unsigned int i = 0 ; i < node_feature_descriptors_.size() ; i++)
  {
    delete node_feature_descriptors_[i];
  }
  for (unsigned int i = 0 ; i < clique_set_feature_descriptors_.size() ; i++)
  {
    for (unsigned int j = 0 ; j < clique_set_feature_descriptors_[i].size() ; j++)
    {
      delete clique_set_feature_descriptors_[i][j];
    }
  }
  node_feature_descriptors_.clear();
  clique_set_feature_descriptors_.clear();
  clique_set_clusterings_.clear();

  node_feature_descriptors_ = initNodeFeatures();

  clique_set_feature_descriptors_.resize(nbr_clique_sets);
  clique_set_clusterings_.resize(nbr_clique_sets);
  clique_set_feature_descriptors_[0] = initCS0Features();
  clique_set_feature_descriptors_[1] = initCS1Features();
  clique_set_clusterings_[0] = initCS0Clusters();
  clique_set_clusterings_[1] = initCS1Clusters();
}

// --------------------------------------------------------------
/*
 * \brief Create nodes in the random field
 */
// --------------------------------------------------------------
void PtCloudRFCreator::createNodes(RandomField& rf,
                                   const robot_msgs::PointCloud& pt_cloud,
                                   cloud_kdtree::KdTree& pt_cloud_kdtree,
                                   const vector<float>& labels,
                                   set<unsigned int>& failed_indices)

{
  bool use_labels = labels.size() != 0;

  // ----------------------------------------------
  // Create interests points over the whole point cloud
  unsigned int nbr_pts = pt_cloud.pts.size();
  cv::Vector<const robot_msgs::Point32*> interest_pts(nbr_pts, NULL);
  for (unsigned int i = 0 ; i < nbr_pts ; i++)
  {
    interest_pts[(size_t) i] = &(pt_cloud.pts[i]);
  }

  // ----------------------------------------------
  // Compute features over all point cloud
  vector<float*> concatenated_features(nbr_pts, NULL);
  unsigned int nbr_concatenated_vals = Descriptor3D::computeAndConcatFeatures(pt_cloud, pt_cloud_kdtree,
      interest_pts, node_feature_descriptors_, concatenated_features, failed_indices);
  if (nbr_concatenated_vals == 0)
  {
    ROS_FATAL("Could not compute node features at all. This should never happen");
    abort();
  }

  // ----------------------------------------------
  // Create nodes for features that were okay
  unsigned int nbr_created_nodes = 0;
  for (unsigned int i = 0 ; i < nbr_pts ; i++)
  {
    // NULL indicates couldnt compute features for interest point
    if (concatenated_features[i] != NULL)
    {
      const RandomField::Node* created_node = NULL;
      if (use_labels)
      {
        if (labels[i] == RandomField::UNKNOWN_LABEL)
        {
          continue;
        }
        created_node = rf.createNode(i, concatenated_features[i], nbr_concatenated_vals,
            static_cast<unsigned int> (labels[i]), pt_cloud.pts[i].x, pt_cloud.pts[i].y, pt_cloud.pts[i].z);
      }
      else
      {
        created_node = rf.createNode(i, concatenated_features[i], nbr_concatenated_vals,
            RandomField::UNKNOWN_LABEL, pt_cloud.pts[i].x, pt_cloud.pts[i].y, pt_cloud.pts[i].z);
      }

      if (created_node == NULL)
      {
        ROS_FATAL("Could not create node for point %u.  This should never happen (conflicting ids probably)", i);
        abort();
      }
      else
      {
        nbr_created_nodes++;
      }
    }
  }

  ROS_INFO("    @@@@@@@@@@ Created %u nodes from %u total points @@@@@@@@@@", nbr_created_nodes, nbr_pts);
}

// --------------------------------------------------------------
/*!
 * \brief Create clique set in the RandomField using kmeans clustering
 */
// --------------------------------------------------------------
void PtCloudRFCreator::createCliqueSet(RandomField& rf,
                                       const robot_msgs::PointCloud& pt_cloud,
                                       cloud_kdtree::KdTree& pt_cloud_kdtree,
                                       const set<unsigned int>& node_indices,
                                       const unsigned int clique_set_idx)
{
  const map<unsigned int, RandomField::Node*>& rf_nodes = rf.getNodesRandomFieldIDs();
  vector<pair<bool, point_cloud_clustering::PointCloudClustering*> >& clique_set_info =
      clique_set_clusterings_[clique_set_idx];

  // total number of clusters and cliques created from this function call
  unsigned int nbr_created_cliques = 0;
  unsigned int nbr_created_clusters = 0;

  unsigned int nbr_constructions = clique_set_info.size();
  for (unsigned int i = 0 ; i < nbr_constructions ; i++)
  {
    // ----------------------------------------------
    // Clustering
    // ----------------------------------------------
    // create new clusters starting from previous count
    clique_set_info[i].second->setStartingClusterLabel(nbr_created_clusters);
    map<unsigned int, vector<int> > created_clusters;
    bool cluster_only_nodes = clique_set_info[i].first;
    int cluster_ret_val = -1;
    if (cluster_only_nodes)
    {
      cluster_ret_val = clique_set_info[i].second->cluster(pt_cloud, pt_cloud_kdtree, node_indices,
          created_clusters);
    }
    else
    {
      cluster_ret_val = clique_set_info[i].second->cluster(pt_cloud, pt_cloud_kdtree, created_clusters);
    }
    if (cluster_ret_val < 0)
    {
      abort();
    }
    map<unsigned int, vector<float> > cluster_centroids;
    point_cloud_clustering::PointCloudClustering::computeClusterCentroids(pt_cloud, created_clusters,
        cluster_centroids);
    unsigned int curr_nbr_clusters = created_clusters.size();
    nbr_created_clusters += curr_nbr_clusters;

    // ----------------------------------------------
    // Feature computation
    // ----------------------------------------------
    // Create interests regions from the clustering
    cv::Vector<const vector<int>*> interest_region_indices(curr_nbr_clusters, NULL);
    size_t cluster_idx = 0;
    for (map<unsigned int, vector<int> >::iterator iter_created_clusters = created_clusters.begin() ; iter_created_clusters
        != created_clusters.end() ; iter_created_clusters++)
    {
      interest_region_indices[cluster_idx++] = (&iter_created_clusters->second);
    }
    // ----------------------------------------------
    // Compute features over clusters
    vector<float*> concatenated_features(curr_nbr_clusters, NULL);
    set<unsigned int> failed_region_indices; // unused
    unsigned int nbr_concatenated_vals = Descriptor3D::computeAndConcatFeatures(pt_cloud, pt_cloud_kdtree,
        interest_region_indices, clique_set_feature_descriptors_[clique_set_idx], concatenated_features,
        failed_region_indices);
    if (nbr_concatenated_vals == 0)
    {
      ROS_FATAL("Could not compute cluster features at all. This should never happen");
      abort();
    }

    // ----------------------------------------------
    // Clique construction
    // Assumes that point index in point cloud is equivalent to random field node id
    // ----------------------------------------------
    cluster_idx = 0;
    for (map<unsigned int, vector<int> >::iterator iter_created_clusters = created_clusters.begin() ; iter_created_clusters
        != created_clusters.end() ; iter_created_clusters++)
    {
      float* curr_cluster_features = concatenated_features[cluster_idx++];

      // Only create cliques where could compute cluster features
      if (curr_cluster_features != NULL)
      {
        // Retrieve point cloud indices within the cluster.
        const vector<int>& curr_cluster_pt_indices = iter_created_clusters->second;
        unsigned int nbr_pts_in_cluster = curr_cluster_pt_indices.size();

        // For each point, find its corresponding node (it may not exist if clusting over all points),
        // and create a list of the nodes that represent the points in the cluster
        list<const RandomField::Node*> clique_nodes;
        unsigned int nbr_nodes_in_clique = 0;
        for (unsigned int j = 0 ; j < nbr_pts_in_cluster ; j++)
        {
          unsigned int curr_pt_idx = curr_cluster_pt_indices[j];
          if (rf_nodes.count(curr_pt_idx) != 0)
          {
            clique_nodes.push_back(rf_nodes.find(curr_pt_idx)->second);
            nbr_nodes_in_clique++;
          }
        }

        // Ensure the clique has at least two nodes
        if (nbr_nodes_in_clique < 2)
        {
          free(curr_cluster_features);
          ROS_DEBUG("Skipping clique of size less than 2");
        }
        else
        {
          // Retrieve the cluster's label
          const unsigned int curr_cluster_label = iter_created_clusters->first;

          // Retrieve the cluster's centroid
          const vector<float>& centroid = cluster_centroids.find(curr_cluster_label)->second;

          // Create clique using the cluster label as the clique id
          if (rf.createClique(curr_cluster_label, clique_set_idx, clique_nodes, curr_cluster_features,
              nbr_concatenated_vals, centroid[0], centroid[1], centroid[2]) == NULL)
          {
            abort();
          }
          else
          {
            nbr_created_cliques++;
          }
        }
      } // end if (curr_cluster_features != NULL)
    } // end iterate over clusters
  }

  ROS_INFO("    ########### Created clique set %u with %u cliques from %u clusters #############", clique_set_idx, nbr_created_cliques, nbr_created_clusters);
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
RandomField* PtCloudRFCreator::createRandomField(const robot_msgs::PointCloud& pt_cloud,
                                                 const vector<float>& labels)
{
  createDescriptors();
  unsigned int nbr_clique_sets = clique_set_clusterings_.size();

  cloud_kdtree::KdTreeANN pt_cloud_kdtree(pt_cloud);
  RandomField* rf = new RandomField(nbr_clique_sets);

  ROS_INFO("=============== CREATING RANDOM FIELD =================");

  // ----------------------------------------------------------
  // Create nodes
  set<unsigned int> failed_indices;
  createNodes(*rf, pt_cloud, pt_cloud_kdtree, labels, failed_indices);
  set<unsigned int> node_indices;
  const unsigned int nbr_pts = pt_cloud.pts.size();
  for (unsigned int i = 0 ; i < nbr_pts ; i++)
  {
    if (failed_indices.count(i) == 0)
    {
      node_indices.insert(i);
    }
  }

  // ----------------------------------------------------------
  // Create clique sets
  for (unsigned int i = 0 ; i < nbr_clique_sets ; i++)
  {
    createCliqueSet(*rf, pt_cloud, pt_cloud_kdtree, node_indices, i);
  }

  ROS_INFO("=============== FINISHED RANDOM FIELD =================\n");
  return rf;
}
