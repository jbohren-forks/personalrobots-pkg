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

#include <descriptors_3d/descriptors_3d.h>
#include <descriptors_3d/spectral_shape.h>
#include <descriptors_3d/orientation.h>
#include <descriptors_3d/position.h>

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

vector<Descriptor3D*> node_feature_descriptors;
M3NParams m3n_params;
kmeans_params_t cs0_kmeans_params;

void populateParameters()
{
  // ----------------------------------------------
  // Node features
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
  node_feature_descriptors.assign(3, spectral_shape);
  node_feature_descriptors[1] = orientation;
  node_feature_descriptors[2] = position;

  unsigned int nbr_clique_sets = 1;
  // ----------------------------------------------
  // Clique set 0
  // kmeans parameters for constructing cliques
  cs0_kmeans_params.factor = 0.003;
  cs0_kmeans_params.accuracy = 1.0;
  cs0_kmeans_params.max_iter = 10;
  cs0_kmeans_params.channel_indices.clear();

  // ----------------------------------------------
  // Define learning parameters
  vector<float> robust_potts_params(nbr_clique_sets, -1.0);
  RegressionTreeWrapperParams regression_tree_params;
  m3n_params.setLearningRate(0.5);
  m3n_params.setNumberOfIterations(15);
  m3n_params.setRegressorRegressionTrees(regression_tree_params);
  m3n_params.setInferenceRobustPotts(robust_potts_params);

  // TODO: free node_feature_descriptors
}

void save_clusters(const map<unsigned int, vector<unsigned int> >& cluster_centroids_indices,
                   const robot_msgs::PointCloud& pt_cloud)
{
  ofstream outt("clusters.txt");

  map<unsigned int, vector<unsigned int> >::const_iterator iter;
  for (iter = cluster_centroids_indices.begin(); iter != cluster_centroids_indices.end() ; iter++)
  {
    const vector<unsigned int>& curr_indices = iter->second;
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
                  kmeans_params_t& kmeans_params,
                  map<unsigned int, vector<float> >& cluster_centroids_xyz,
                  map<unsigned int, vector<unsigned int> >& cluster_centroids_indices)

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
  unsigned int curr_offset = 0; // offset into feature_matrix
  unsigned int nbr_skipped = 0;
  unsigned int curr_sample_idx = 0;
  for (unsigned int i = 0 ; i < nbr_total_pts ; i++)
  {
    if (ignore_indices.count(i) != 0)
    {
      nbr_skipped++;
      continue;
    }

    curr_sample_idx = i - nbr_skipped;

    // offset over previous samples
    curr_offset = curr_sample_idx * cluster_feature_dim;

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
  cluster_centroids_xyz.clear();
  cluster_centroids_indices.clear();
  nbr_skipped = 0;
  unsigned int curr_cluster_label = 0;
  for (unsigned int i = 0 ; i < nbr_total_pts ; i++)
  {
    if (ignore_indices.count(i) != 0)
    {
      nbr_skipped++;
      continue;
    }

    // Important: Some points may have been skipped when creating matrix for clustering,
    // so we need to adjust the index appropriately
    curr_sample_idx = i - nbr_skipped;

    // Get the cluster label of the current point
    curr_cluster_label = static_cast<unsigned int> (cluster_labels->data.i[curr_sample_idx]);

    // Instantiate container if never encountered label before
    if (cluster_centroids_xyz.count(curr_cluster_label) == 0)
    {
      cluster_centroids_xyz[curr_cluster_label] = vector<float> (3, 0.0);
      cluster_centroids_indices[curr_cluster_label] = vector<unsigned int> ();
    }

    // accumulate total xyz coordinates in cluster
    cluster_centroids_xyz[curr_cluster_label][0] += pt_cloud.pts[i].x;
    cluster_centroids_xyz[curr_cluster_label][1] += pt_cloud.pts[i].y;
    cluster_centroids_xyz[curr_cluster_label][2] += pt_cloud.pts[i].z;

    // associate node with the cluster label
    cluster_centroids_indices[curr_cluster_label].push_back(i);
  }

  // ----------------------------------------------------------
  // Finalize xyz centroid for each cluster
  map<unsigned int, vector<float> >::iterator iter_cluster_centroids_xyz;
  float curr_cluster_nbr_pts = 0.0;
  for (iter_cluster_centroids_xyz = cluster_centroids_xyz.begin(); iter_cluster_centroids_xyz
      != cluster_centroids_xyz.end() ; iter_cluster_centroids_xyz++)
  {
    curr_cluster_nbr_pts
        = static_cast<float> (cluster_centroids_indices[iter_cluster_centroids_xyz->first].size());
    iter_cluster_centroids_xyz->second[0] /= curr_cluster_nbr_pts;
    iter_cluster_centroids_xyz->second[1] /= curr_cluster_nbr_pts;
    iter_cluster_centroids_xyz->second[2] /= curr_cluster_nbr_pts;
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
      interest_pts, node_feature_descriptors, concatenated_features, failed_indices);
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

// --------------------------------------------------------------
/*!
 * \brief Create clique set 0
 */
// --------------------------------------------------------------
void createCliqueSet0(RandomField& rf,
                      const robot_msgs::PointCloud& pt_cloud,
                      cloud_kdtree::KdTree& pt_cloud_kdtree,
                      set<unsigned int>& skip_indices)
{
  // ----------------------------------------------
  // Create clusters
  map<unsigned int, vector<float> > cluster_centroids_xyz;
  map<unsigned int, vector<unsigned int> > cluster_centroids_indices;
  ROS_INFO("Clustering...");
  kmeansPtCloud(pt_cloud, skip_indices, cs0_kmeans_params, cluster_centroids_xyz, cluster_centroids_indices);
  ROS_INFO("done");
  save_clusters(cluster_centroids_indices, pt_cloud);
  ROS_INFO("Kmeans found %u clusters", cluster_centroids_indices.size());

  // TODO compute centroid
  /*

   // ----------------------------------------------
   // Geometry feature information
   // TODO HARDCODE
   LocalGeometry geometry_features;
   geometry_features.setData(&pt_cloud, &pt_cloud_kdtree);
   //geometry_features.setInterestRadius(0.15);
   geometry_features.useElevation();
   geometry_features.useNormalOrientation(0.0, 0.0, 1.0);
   geometry_features.useTangentOrientation(0.0, 0.0, 1.0);

   vector<Descriptor3D*> feature_descriptors(1, &geometry_features);
   vector<unsigned int> feature_descriptor_vals(1, geometry_features.getResultSize());
   unsigned int nbr_total_feature_vals = geometry_features.getResultSize();

   ROS_INFO("Creating cliques...");
   vector<int> region_indices;
   map<unsigned int, list<RandomField::Node*> >::iterator iter_created_clusters;
   for (iter_created_clusters = created_clusters.begin(); iter_created_clusters != created_clusters.end() ; iter_created_clusters++)
   {
   list<RandomField::Node*>& curr_list = iter_created_clusters->second;

   // create region indices to compute features over
   region_indices.clear();
   for (list<RandomField::Node*>::iterator iter = curr_list.begin() ; iter != curr_list.end() ; iter++)
   {
   region_indices.push_back(static_cast<int> ((*iter)->getRandomFieldID()));
   }

   // create concatenated feature
   float* concat_created_feature_vals = NULL;
   if (createConcatenatedFeatures(feature_descriptors, feature_descriptor_vals, nbr_total_feature_vals,
   NULL, &region_indices, &concat_created_feature_vals) < 0)
   {
   continue;
   }

   // try to create node with features
   if (rf.createClique(0, curr_list, concat_created_feature_vals, nbr_total_feature_vals,
   xyz_cluster_centroids[iter_created_clusters->first][0],
   xyz_cluster_centroids[iter_created_clusters->first][1],
   xyz_cluster_centroids[iter_created_clusters->first][2]) == NULL)
   {
   ROS_ERROR("could not create clique");
   abort();
   }
   }
   ROS_INFO("done");
   */
}

// --------------------------------------------------------------
/*!
 * \brief Create nodes in the random field
 */
// --------------------------------------------------------------
int main()
{
  populateParameters();

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

  // ----------------------------------------------------------
  // Create RandomField
  RandomField rf(1);
  set<unsigned int> failed_indices;
  ROS_INFO("Creating nodes...");
  createNodes(rf, pt_cloud, *pt_cloud_kdtree, labels, failed_indices);
  rf.saveNodeFeatures("node_features.txt");
  ROS_INFO("done");

  ROS_INFO("Creating clique set 0...");
  createCliqueSet0(rf, pt_cloud, *pt_cloud_kdtree, failed_indices);
  ROS_INFO("done");

  // ----------------------------------------------------------
  // Train M3N model
  ROS_INFO("Starting to train...");
  M3NModel m3n_model;
  vector<const RandomField*> training_rfs(1, &rf);
  if (m3n_model.train(training_rfs, m3n_params) < 0)
  {
    ROS_ERROR("Failed to train M3N model");
  }
  else
  {
    ROS_INFO("Successfully trained M3n model");
  }
  return 0;
}
