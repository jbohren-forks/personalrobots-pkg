/*
 * create_pt_cloud.cpp
 *
 *  Created on: Jul 2, 2009
 *      Author: dmunoz
 */

#include <iostream>
#include <fstream>
#include <vector>

#include <robot_msgs/PointCloud.h>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/cvaux.hpp>

#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

#include <descriptors_3d/descriptor_3d.h>
#include <descriptors_3d/spectral_shape.h>
#include <descriptors_3d/orientation.h>
#include <descriptors_3d/position.h>
#include <descriptors_3d/spin_image.h>
#include <descriptors_3d/bounding_box.h>

#include <functional_m3n/random_field.h>
#include <functional_m3n/m3n_model.h>
#include <functional_m3n/regressors/regressor_params.h>

using namespace std;

typedef struct kmeans_params
{
    double factor;
    double accuracy;
    int max_iter;
    vector<unsigned int> channel_indices;
} kmeans_params_t;

vector<Descriptor3D*> GLOBAL_node_feature_descriptors;
vector<kmeans_params_t> GLOBAL_cs_kmeans_params;
vector<vector<Descriptor3D*> > GLOBAL_cs_feature_descriptors;
M3NParams GLOBAL_m3n_params;

void initNodeParams()
{
  SpectralShape* spectral_shape = new SpectralShape();
  spectral_shape->setSupportRadius(0.15);

  Orientation* orientation = new Orientation();
  if (orientation->useSpectralInformation(spectral_shape) < 0)
  {
    abort();
  }
  orientation->useNormalOrientation(0.0, 0.0, 1.0);
  orientation->useTangentOrientation(0.0, 0.0, 1.0);

  Position* position = new Position();

  // ---------------
  GLOBAL_node_feature_descriptors.push_back(spectral_shape);
  GLOBAL_node_feature_descriptors.push_back(orientation);
  GLOBAL_node_feature_descriptors.push_back(position);
}

void initCS0Params()
{
  kmeans_params_t cs0_kmeans_params;
  vector<Descriptor3D*> cs0_feature_descriptors;

  // ---------------
  // kmeans parameters for constructing cliques
  cs0_kmeans_params.factor = 0.003;
  cs0_kmeans_params.accuracy = 1.0;
  cs0_kmeans_params.max_iter = 10;
  cs0_kmeans_params.channel_indices.clear();

  // ---------------
  // Features
  SpectralShape* spectral_shape = new SpectralShape();
  spectral_shape->setSupportRadius(0.2286);

  Orientation* orientation = new Orientation();
  if (orientation->useSpectralInformation(spectral_shape) < 0)
  {
    abort();
  }
  orientation->useNormalOrientation(0.0, 0.0, 1.0);
  orientation->useTangentOrientation(0.0, 0.0, 1.0);

  Position* position = new Position();

  SpinImage* spin_image = new SpinImage();
  spin_image->setAxisCustom(0.0, 0.0, 1.0);
  spin_image->setImageDimensions(0.0762, 0.0762, 5, 4);

  BoundingBox* bounding_box = new BoundingBox(true, false);
  bounding_box->useSpectralInformation(spectral_shape);

  // ---------------
  cs0_feature_descriptors.push_back(spectral_shape);
  cs0_feature_descriptors.push_back(orientation);
  cs0_feature_descriptors.push_back(position);
  cs0_feature_descriptors.push_back(spin_image);
  cs0_feature_descriptors.push_back(bounding_box);

  // ---------------
  GLOBAL_cs_kmeans_params.push_back(cs0_kmeans_params);
  GLOBAL_cs_feature_descriptors.push_back(cs0_feature_descriptors);
}

void initCS1Params()
{
  kmeans_params_t cs1_kmeans_params;
  vector<Descriptor3D*> cs1_feature_descriptors;

  // ---------------
  // kmeans parameters for constructing cliques
  cs1_kmeans_params.factor = 0.001;
  cs1_kmeans_params.accuracy = 1.0;
  cs1_kmeans_params.max_iter = 10;
  cs1_kmeans_params.channel_indices.clear();

  // ---------------
  // Features
  SpectralShape* spectral_shape = new SpectralShape();
  spectral_shape->setSupportRadius(-1);

  Orientation* orientation = new Orientation();
  if (orientation->useSpectralInformation(spectral_shape) < 0)
  {
    abort();
  }
  orientation->useNormalOrientation(0.0, 0.0, 1.0);
  orientation->useTangentOrientation(0.0, 0.0, 1.0);

  Position* position = new Position();

  SpinImage* spin_image = new SpinImage();
  spin_image->useSpectralInformation(spectral_shape);
  spin_image->setAxisNormal();
  spin_image->setImageDimensions(0.0762, 0.0762, 7, 5);

  BoundingBox* bounding_box = new BoundingBox(true, false);
  bounding_box->useSpectralInformation(spectral_shape);

  // ---------------
  cs1_feature_descriptors.push_back(spectral_shape);
  cs1_feature_descriptors.push_back(orientation);
  cs1_feature_descriptors.push_back(position);
  cs1_feature_descriptors.push_back(spin_image);
  cs1_feature_descriptors.push_back(bounding_box);

  // ---------------
  GLOBAL_cs_kmeans_params.push_back(cs1_kmeans_params);
  GLOBAL_cs_feature_descriptors.push_back(cs1_feature_descriptors);
}

unsigned int populateParameters()
{
  initNodeParams();

  unsigned int nbr_clique_sets = 2;
  initCS0Params();
  initCS1Params();
  vector<float> robust_potts_params(nbr_clique_sets, -1.0);
  //robust_potts_params[0] = 0.13;
  //robust_potts_params[1] = 0.20;

  // ----------------------------------------------
  // Define learning parameters
  RegressionTreeWrapperParams regression_tree_params;
  regression_tree_params.max_tree_depth_factor = 0.2; // was 0.4
  GLOBAL_m3n_params.setLearningRate(0.4);
  GLOBAL_m3n_params.setNumberOfIterations(6);
  GLOBAL_m3n_params.setRegressorRegressionTrees(regression_tree_params);
  GLOBAL_m3n_params.setInferenceRobustPotts(robust_potts_params);

  // TODO: free descriptors
  return nbr_clique_sets;
}

// TODO: delete this
void save_clusters(const map<unsigned int, vector<int> >& cluster_centroids_indices,
                   const robot_msgs::PointCloud& pt_cloud)
{
  ofstream outt("clusters.txt");

  map<unsigned int, vector<int> >::const_iterator iter;
  for (iter = cluster_centroids_indices.begin(); iter != cluster_centroids_indices.end() ; iter++)
  {
    const vector<int>& curr_indices = iter->second;
    for (unsigned int i = 0 ; i < curr_indices.size() ; i++)
    {
      outt << pt_cloud.pts[curr_indices[i]].x << " " << pt_cloud.pts[curr_indices[i]].y << " "
          << pt_cloud.pts[curr_indices[i]].z << " " << iter->first << endl;
    }
  }
  outt.close();
}

// --------------------------------------------------------------
/*!
 * \brief Creates PointCloud datastructure from file
 *
 * File format: x y z label 2
 */
// --------------------------------------------------------------
int loadPointCloud(string filename, robot_msgs::PointCloud& pt_cloud, vector<unsigned int>& labels)
{
  // ----------------------------------------------
  // Open file
  ifstream infile(filename.c_str());
  if (infile.is_open() == false)
  {
    ROS_ERROR("Could not open filename: %s", filename.c_str());
    return -1;
  }

  // ----------------------------------------------
  // Count number of lines in the file
  unsigned int nbr_samples = 0;
  char line[256];
  while (infile.getline(line, 256))
  {
    nbr_samples++;
  }

  infile.clear();
  infile.seekg(ios::beg);

  // ----------------------------------------------
  // Resize containers appropriately
  pt_cloud.pts.resize(nbr_samples);
  labels.resize(nbr_samples);

  // ----------------------------------------------
  // Read file
  // file format: x y z label 2
  unsigned int tempo;
  for (unsigned int i = 0 ; i < nbr_samples ; i++)
  {
    infile >> pt_cloud.pts[i].x;
    infile >> pt_cloud.pts[i].y;
    infile >> pt_cloud.pts[i].z;
    infile >> labels[i];
    infile >> tempo;
  }

  infile.close();
  return 0;
}

// --------------------------------------------------------------
/*!
 * \brief Performs k-means on a point cloud
 */
// --------------------------------------------------------------
int kmeansPtCloud(const robot_msgs::PointCloud& pt_cloud,
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
      ROS_ERROR("Channel index %u exceeds number of channels %u", kmeans_channel_indices[i], pt_cloud.chan.size());
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

// --------------------------------------------------------------
/*!
 * \brief Create nodes in the random field
 */
// --------------------------------------------------------------
void createNodes(RandomField& rf,
                 robot_msgs::PointCloud& pt_cloud,
                 cloud_kdtree::KdTree& pt_cloud_kdtree,
                 const vector<unsigned int>& labels,
                 set<unsigned int>& failed_indices)

{
  unsigned int nbr_pts = pt_cloud.pts.size();

  // ----------------------------------------------
  // Create interests points over the whole point cloud
  cv::Vector<robot_msgs::Point32*> interest_pts(nbr_pts, NULL);
  for (unsigned int i = 0 ; i < nbr_pts ; i++)
  {
    interest_pts[(size_t) i] = &(pt_cloud.pts[i]);
  }

  // ----------------------------------------------
  // Compute features over all point cloud
  vector<float*> concatenated_features(nbr_pts, NULL);
  unsigned int nbr_concatenated_vals = Descriptor3D::computeAndConcatFeatures(pt_cloud, pt_cloud_kdtree,
      interest_pts, GLOBAL_node_feature_descriptors, concatenated_features, failed_indices);
  if (nbr_concatenated_vals == 0)
  {
    ROS_FATAL("Could not compute node features at all. This should never happen");
    abort();
  }

  // ----------------------------------------------
  // Create nodes for features that were okay
  for (unsigned int i = 0 ; i < nbr_pts ; i++)
  {
    // NULL indicates couldnt compute features for interest point
    if (concatenated_features[i] != NULL)
    {
      if (rf.createNode(i, concatenated_features[i], nbr_concatenated_vals, labels[i], pt_cloud.pts[i].x,
          pt_cloud.pts[i].y, pt_cloud.pts[i].z) == NULL)
      {
        ROS_FATAL("Could not create node for point %u.  This should never happen", i);
        abort();
      }
    }
  }
}

unsigned int createClusterFeatures(const robot_msgs::PointCloud& pt_cloud,
                                   cloud_kdtree::KdTree& pt_cloud_kdtree,
                                   map<unsigned int, vector<int> >& cluster_indices, // TODO: change to const vector<int>
                                   vector<Descriptor3D*>& feature_descriptors,
                                   map<unsigned int, float*>& cluster_features)
{
  unsigned int nbr_clusters = cluster_indices.size();

  // ----------------------------------------------
  // Create interests regions from the clustering
  cv::Vector<vector<int>*> interest_region_indices(nbr_clusters, NULL);
  size_t cluster_idx = 0;
  for (map<unsigned int, vector<int> >::iterator iter_cluster_indices = cluster_indices.begin() ; iter_cluster_indices
      != cluster_indices.end() ; iter_cluster_indices++)
  {
    interest_region_indices[cluster_idx++] = (&iter_cluster_indices->second);
  }

  // ----------------------------------------------
  // Compute features over clusters
  vector<float*> concatenated_features(nbr_clusters, NULL);
  set<unsigned int> failed_region_indices; // unused
  unsigned int nbr_concatenated_vals = Descriptor3D::computeAndConcatFeatures(pt_cloud, pt_cloud_kdtree,
      interest_region_indices, feature_descriptors, concatenated_features, failed_region_indices);
  if (nbr_concatenated_vals == 0)
  {
    ROS_FATAL("Could not compute cluster features at all. This should never happen");
    abort();
  }

  // ----------------------------------------------
  // Create mapping cluster_label -> features
  cluster_features.clear();
  cluster_idx = 0;
  for (map<unsigned int, vector<int> >::iterator iter_cluster_indices = cluster_indices.begin() ; iter_cluster_indices
      != cluster_indices.end() ; iter_cluster_indices++)
  {
    cluster_features[iter_cluster_indices->first] = concatenated_features[cluster_idx++];
  }

  return nbr_concatenated_vals;
}

// --------------------------------------------------------------
/*!
 * \brief Create clique set in the RandomField using kmeans clustering
 */
// --------------------------------------------------------------
void createCliqueSet(RandomField& rf,
                     const unsigned int clique_set_idx,
                     const map<unsigned int, vector<int> >& cluster_node_ids,
                     const map<unsigned int, float*>& cluster_features,
                     const map<unsigned int, vector<float> >& cluster_centroids_xyz,
                     const unsigned int nbr_feature_vals)
{
  const map<unsigned int, RandomField::Node*>& rf_nodes = rf.getNodesRandomFieldIDs();

  // ----------------------------------------------
  // Create cliques for features that were okay
  for (map<unsigned int, float*>::const_iterator iter_cluster_features = cluster_features.begin() ; iter_cluster_features
      != cluster_features.end() ; iter_cluster_features++)
  {
    if (iter_cluster_features->second != NULL)
    {
      unsigned int curr_cluster_label = iter_cluster_features->first;

      // crate list of nodes from the indices
      list<const RandomField::Node*> clique_nodes;
      const vector<int>& curr_indices = cluster_node_ids.find(curr_cluster_label)->second;
      for (unsigned int i = 0 ; i < curr_indices.size() ; i++)
      {
        clique_nodes.push_back(rf_nodes.find(curr_indices[i])->second);
      }

      // create the clique
      const vector<float>& xyz = cluster_centroids_xyz.find(curr_cluster_label)->second;
      if (rf.createClique(curr_cluster_label, clique_set_idx, clique_nodes, iter_cluster_features->second,
          nbr_feature_vals, xyz[0], xyz[1], xyz[2]) == NULL)
      {
        abort();
      }
    }
  }
}

// --------------------------------------------------------------
/*!
 * \brief Create nodes in the random field
 */
// --------------------------------------------------------------
int main()
{
  // ----------------------------------------------------------
  // Load point cloud from file
  ROS_INFO("Loading point cloud...");
  robot_msgs::PointCloud pt_cloud;
  vector<unsigned int> labels;
  if (loadPointCloud("training_data.xyz_label_conf", pt_cloud, labels) < 0)
  {
    return -1;
  }
  ROS_INFO("done");

  cloud_kdtree::KdTree* pt_cloud_kdtree = new cloud_kdtree::KdTreeANN(pt_cloud);

  unsigned int nbr_clique_sets = populateParameters();

  // ----------------------------------------------------------
  // Create RandomField
  RandomField rf(nbr_clique_sets);

  // Create nodes
  set<unsigned int> skip_indices_for_clustering;
  ROS_INFO("Creating nodes...");
  createNodes(rf, pt_cloud, *pt_cloud_kdtree, labels, skip_indices_for_clustering);
  ROS_INFO("done");

  rf.saveNodeFeatures("node_features.txt");

  // Create clique sets
  for (unsigned int i = 0 ; i < nbr_clique_sets ; i++)
  {
    ROS_INFO("Creating clique set %u...", i);

    // Create clusters
    map<unsigned int, vector<float> > cluster_xyz_centroids;
    map<unsigned int, vector<int> > cluster_pt_indices;
    ROS_INFO("Clustering...");
    kmeansPtCloud(pt_cloud, skip_indices_for_clustering, GLOBAL_cs_kmeans_params[i], cluster_xyz_centroids,
        cluster_pt_indices);
    //ROS_INFO("Kmeans found %u clusters", cluster_pt_indices.size());
    ROS_INFO("done");

    save_clusters(cluster_pt_indices, pt_cloud);

    // Create features over clusters
    ROS_INFO("Creating features...");
    map<unsigned int, float*> cluster_features;
    unsigned int nbr_feature_vals = createClusterFeatures(pt_cloud, *pt_cloud_kdtree, cluster_pt_indices,
        GLOBAL_cs_feature_descriptors[i], cluster_features);
    ROS_INFO("done");

    // Create cliques
    ROS_INFO("Creating features...");
    createCliqueSet(rf, i, cluster_pt_indices, cluster_features, cluster_xyz_centroids, nbr_feature_vals);
    ROS_INFO("done");

    ROS_INFO("finished clique set %u", i);
  }

  rf.saveCliqueFeatures("ss_o_p_si");

  // ----------------------------------------------------------
  // Train M3N model
  ROS_INFO("Starting to train...");
  M3NModel m3n_model;
  vector<const RandomField*> training_rfs(1, &rf);
  if (m3n_model.train(training_rfs, GLOBAL_m3n_params) < 0)
  {
    ROS_ERROR("Failed to train M3N model");
    return -1;
  }
  ROS_INFO("Successfully trained M3n model");
  return 0;
}
