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

#include <Eigen/Core>

#include <flann.h>

#include <descriptors_3d/local_geometry.h>

#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

#include <functional_m3n/random_field.h>
#include <functional_m3n/m3n_model.h>
#include <functional_m3n/regressors/regressor_params.h>

M3NModel* trainModel(vector<const RandomField*>& training_rfs);

void save_clusters(map<unsigned int, list<RandomField::Node*> >& created_clusters)
{
  ofstream outt("clusters.txt");
  map<unsigned int, list<RandomField::Node*> >::iterator iter;
  list<RandomField::Node*>::iterator node_iter;
  RandomField::Node* curr_node = NULL;
  for (iter = created_clusters.begin(); iter != created_clusters.end() ; iter++)
  {
    list<RandomField::Node*>& curr_nodes = iter->second;
    for (node_iter = curr_nodes.begin(); node_iter != curr_nodes.end() ; node_iter++)
    {
      curr_node = *node_iter;
      outt << curr_node->getX() << " " << curr_node->getY() << " " << curr_node->getZ() << " " << iter->first
          << endl;
    }
  }
  outt.close();
}

void save_node_features(const map<unsigned int, RandomField::Node*>& nodes)
{
  ofstream outt("node_features.txt");
  map<unsigned int, RandomField::Node*>::const_iterator iter;
  RandomField::Node* curr_node = NULL;
  for (iter = nodes.begin(); iter != nodes.end() ; iter++)
  {
    curr_node = iter->second;
    outt << curr_node->getX() << " " << curr_node->getY() << " " << curr_node->getZ();
    const float* curr_feats = curr_node->getFeatureVals();
    for (unsigned int i = 0 ; i < curr_node->getNumberFeatureVals() ; i++)
    {
      outt << " " << curr_feats[i];

    }
    outt << endl;
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
 * \brief Creates long vector from multiple descriptor computations
 */
// --------------------------------------------------------------
/*
 int createConcatenatedFeatures(const vector<Descriptor3D*>& feature_descriptors,
 const vector<unsigned int>& feature_descriptor_vals,
 const unsigned int nbr_total_feature_vals,
 unsigned int* interest_pt_idx,
 vector<int>* interest_region_indices,
 float** concat_features)
 {
 vector<Eigen::MatrixXf*> created_features_eigen(feature_descriptors.size(), NULL);

 // Attempt to compute multiple descriptor feature vectors
 bool all_features_success = true;
 unsigned int j = 0;
 for (j = 0; all_features_success && j < feature_descriptors.size() ; j++)
 {
 if (interest_pt_idx != NULL)
 {
 feature_descriptors[j]->setInterestPoint(*interest_pt_idx);
 }
 else if (interest_region_indices != NULL)
 {
 feature_descriptors[j]->setInterestRegion(interest_region_indices);
 }
 else
 {
 return -1;
 }
 all_features_success = feature_descriptors[j]->compute(&(created_features_eigen[j]));
 }

 // If one descriptor failed, deallocate previously create features and quit
 if (!all_features_success)
 {
 for (unsigned int k = 0 ; k < (j - 1) ; k++)
 {
 delete created_features_eigen[k];
 }
 return -1;
 }

 // Copy each feature from above into big concatenated vector
 *concat_features = static_cast<float*> (malloc(nbr_total_feature_vals * sizeof(float)));
 unsigned int prev_total_nbr_vals = 0;
 for (j = 0; j < feature_descriptors.size() ; j++)
 {
 float* curr_feature_vals = (created_features_eigen[j])->data();
 unsigned int curr_nbr_vals = feature_descriptor_vals[j];

 memcpy(*concat_features + prev_total_nbr_vals, curr_feature_vals, curr_nbr_vals * sizeof(float));

 prev_total_nbr_vals += curr_nbr_vals;
 delete created_features_eigen[j];
 }

 return 0;
 }
 */

// cluster the node features using kmeans
void kmeansNodes(const vector<unsigned int>& cluster_feature_indices,
                 const double kmeans_factor,
                 const int kmeans_max_iter,
                 const double kmeans_accuracy,
                 const map<unsigned int, RandomField::Node*>& nodes,
                 map<unsigned int, list<RandomField::Node*> >& created_clusters,
                 map<unsigned int, vector<float> >& xyz_cluster_centroids)

{
  // dimension that clustering over (features + xyz coords)
  unsigned int cluster_feature_dim_xyz = cluster_feature_indices.size() + 3;

  // ----------------------------------------------------------
  // Create matrix for clustering
  // create n-by-d matrix where n are the number of samples and d is the feature dimension
  unsigned int nbr_samples = nodes.size();
  float* feature_matrix = static_cast<float*> (malloc(nbr_samples * cluster_feature_dim_xyz * sizeof(float)));

  map<unsigned int, RandomField::Node*>::const_iterator iter_nodes;
  const RandomField::Node* curr_node = NULL;
  const float* curr_node_features = NULL;

  unsigned int curr_offset = 0; // offset into feature_matrix
  unsigned int curr_sample_idx = 0;
  for (iter_nodes = nodes.begin(); iter_nodes != nodes.end() ; iter_nodes++)
  {
    // offset previous samples
    curr_offset = curr_sample_idx * cluster_feature_dim_xyz;

    // retrieve current sample and its features
    curr_node = iter_nodes->second;
    curr_node_features = curr_node->getFeatureVals();

    // copy xyz coordinates
    feature_matrix[curr_offset] = curr_node->getX();
    feature_matrix[curr_offset + 1] = curr_node->getY();
    feature_matrix[curr_offset + 2] = curr_node->getZ();

    // offset past coordinates
    curr_offset += 3;

    // copy requested feature indices
    for (unsigned int i = 0 ; i < cluster_feature_indices.size() ; i++)
    {
      feature_matrix[curr_offset + i] = curr_node_features[cluster_feature_indices[i]];
    }

    // proceed to next sample
    curr_sample_idx++;
  }

  // ----------------------------------------------------------
  // Do kmeans clustering
  CvMat cv_feature_matrix;
  cvInitMatHeader(&cv_feature_matrix, nbr_samples, cluster_feature_dim_xyz, CV_32F, feature_matrix);
  CvMat* cluster_labels = cvCreateMat(nbr_samples, 1, CV_32SC1);
  int nbr_clusters = static_cast<int> (nbr_samples * kmeans_factor);
  cvKMeans2(&cv_feature_matrix, nbr_clusters, cluster_labels, cvTermCriteria(CV_TERMCRIT_ITER
      + CV_TERMCRIT_EPS, kmeans_max_iter, kmeans_accuracy));

  // ----------------------------------------------------------
  // Associate each node with its cluster label
  curr_sample_idx = 0;
  unsigned int curr_cluster_label = 0;
  created_clusters.clear();
  xyz_cluster_centroids.clear();
  for (iter_nodes = nodes.begin(); iter_nodes != nodes.end() ; iter_nodes++)
  {
    curr_cluster_label = static_cast<unsigned int> (cluster_labels->data.i[curr_sample_idx]);

    // add empty list if not encountered label before
    if (created_clusters.count(curr_cluster_label) == 0)
    {
      created_clusters[curr_cluster_label] = list<RandomField::Node*> ();
      xyz_cluster_centroids[curr_cluster_label] = vector<float> (3, 0.0);
    }

    // associate node with the cluster label
    created_clusters[curr_cluster_label].push_back(iter_nodes->second);

    // accumulate total xyz coordinates in cluster
    xyz_cluster_centroids[curr_cluster_label][0] += curr_node->getX();
    xyz_cluster_centroids[curr_cluster_label][1] += curr_node->getY();
    xyz_cluster_centroids[curr_cluster_label][2] += curr_node->getZ();

    // proceed to next sample
    curr_sample_idx++;
  }

  // Finalize xyz centroid for each cluster
  map<unsigned int, vector<float> >::iterator iter_clusters;
  float curr_cluster_nbr_nodes = 0.0;
  for (iter_clusters = xyz_cluster_centroids.begin(); iter_clusters != xyz_cluster_centroids.end() ; iter_clusters++)
  {
    curr_cluster_nbr_nodes = static_cast<float> (created_clusters[iter_clusters->first].size());
    iter_clusters->second[0] /= curr_cluster_nbr_nodes;
    iter_clusters->second[1] /= curr_cluster_nbr_nodes;
    iter_clusters->second[2] /= curr_cluster_nbr_nodes;
  }

  // ----------------------------------------------------------
  // Cleanup
  cvReleaseMat(&cluster_labels);
  free(feature_matrix);
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void createNodes(RandomField& rf,
                 const robot_msgs::PointCloud& pt_cloud,
                 cloud_kdtree::KdTree& pt_cloud_kdtree,
                 const vector<unsigned int>& labels,
                 set<unsigned int>& failed_indices)

{
  // ----------------------------------------------
  // Geometry feature information
  // TODO HARDCODE
  LocalGeometry geometry_features;
  geometry_features.useSpectral(0.15);
  //  geometry_features.setData(&pt_cloud, &pt_cloud_kdtree);
  // geometry_features.setInterestRadius(0.15);
  geometry_features.useNormalOrientation(0.0, 0.0, 1.0);
  geometry_features.useTangentOrientation(0.0, 0.0, 1.0);
  geometry_features.useElevation();

  vector<Descriptor3D*> feature_descriptors(1, &geometry_features);
  //vector<unsigned int> feature_descriptor_vals(1, geometry_features.getResultSize());
  unsigned int nbr_descriptors = feature_descriptors.size();
  unsigned int nbr_total_feature_vals = geometry_features.getResultSize();

  /*
   // --------------------------------------------------
   unsigned int nbr_pts = pt_cloud.pts.size();
   for (unsigned int i = 0 ; i < nbr_pts ; i++)
   {
   // -------------------------------
   if (i % 1000 == 0)
   {
   ROS_INFO("sample: %u / %u", i, nbr_pts);
   }

   float* concat_created_feature_vals = NULL;
   if (createConcatenatedFeatures(feature_descriptors, feature_descriptor_vals, nbr_total_feature_vals, &i,
   NULL, &concat_created_feature_vals) < 0)
   {
   failed_indices.insert(i);
   continue;
   }

   // try to create node with features
   if (rf.createNode(i, concat_created_feature_vals, nbr_total_feature_vals, labels[i], pt_cloud.pts[i].x,
   pt_cloud.pts[i].y, pt_cloud.pts[i].z) == NULL)
   {
   ROS_ERROR("could not create node %u", i);
   abort();
   }
   }
   */

  vector<cv::Vector<cv::Vector<float> > > all_descriptor_results(nbr_descriptors);
  for (unsigned int i = 0 ; i < nbr_descriptors ; i++)
  {
    feature_descriptors[i]->compute(pt_cloud, pt_cloud_kdtree, all_descriptor_results[i]);
  }

  unsigned int nbr_pts = pt_cloud.pts.size();
  float* concat_features = NULL;
  unsigned int prev_val_idx = 0;
  unsigned int curr_nbr_feature_vals = 0;
  bool all_features_success = true;
  for (unsigned int i = 0 ; i < nbr_pts ; i++)
  {
    // verify all features for the point were computed successfully
    all_features_success = true;
    for (unsigned int j = 0 ; all_features_success && j < nbr_descriptors ; j++)
    {
      cv::Vector<cv::Vector<float> >& curr_descriptor_for_cloud = all_descriptor_results[j];
      cv::Vector<float>& curr_feature_vals = curr_descriptor_for_cloud[i];
      all_features_success = curr_feature_vals.size() != 0;
    }

    // if all successful, then create node
    if (all_features_success)
    {
      concat_features = static_cast<float*> (malloc(nbr_total_feature_vals * sizeof(float)));
      prev_val_idx = 0;
      for (unsigned int j = 0 ; j < nbr_descriptors ; j++)
      {
        cv::Vector<cv::Vector<float> >& curr_descriptor_for_cloud = all_descriptor_results[j];
        cv::Vector<float>& curr_feature_vals = curr_descriptor_for_cloud[i];
        curr_nbr_feature_vals = curr_feature_vals.size();
        for (unsigned int k = 0 ; k < curr_nbr_feature_vals ; k++)
        {
          concat_features[prev_val_idx + k] = curr_feature_vals[k];
        }
        prev_val_idx += curr_nbr_feature_vals;
      }

      // try to create node with features
      if (rf.createNode(i, concat_features, nbr_total_feature_vals, labels[i], pt_cloud.pts[i].x,
          pt_cloud.pts[i].y, pt_cloud.pts[i].z) == NULL)
      {
        ROS_ERROR("could not create node %u", i);
        abort();
      }
    }
  }

  save_node_features(rf.getNodesRandomFieldIDs());
}

void createCliqueSet0(RandomField& rf,
                      const robot_msgs::PointCloud& pt_cloud,
                      cloud_kdtree::KdTree& pt_cloud_kdtree)
{
  // ----------------------------------------------
  // Create clusters
  // TODO HARDCODE
  vector<unsigned int> cluster_indices(3);
  cluster_indices[0] = 0;
  cluster_indices[1] = 1;
  cluster_indices[2] = 2;
  double kmeans_factor = 0.005;
  int kmeans_max_iter = 5;
  double kmeans_accuracy = 1.0;
  map<unsigned int, list<RandomField::Node*> > created_clusters;
  map<unsigned int, vector<float> > xyz_cluster_centroids;
  ROS_INFO("Clustering...");
  kmeansNodes(cluster_indices, kmeans_factor, kmeans_max_iter, kmeans_accuracy, rf.getNodesRandomFieldIDs(),
      created_clusters, xyz_cluster_centroids);
  ROS_INFO("done");
  save_clusters(created_clusters);
  ROS_INFO("Kmeans found %u clusters", created_clusters.size());

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

int main()
{
  // ----------------------------------------------------------
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
  RandomField rf(1);
  set<unsigned int> failed_indices;
  ROS_INFO("Creating nodes...");
  createNodes(rf, pt_cloud, *pt_cloud_kdtree, labels, failed_indices);
  ROS_INFO("done");

  ROS_INFO("Creating clique set 0...");
  createCliqueSet0(rf, pt_cloud, *pt_cloud_kdtree);
  ROS_INFO("done");

  // ----------------------------------------------------------
  ROS_INFO("Starting to train...");
  vector<const RandomField*> training_rfs(1, &rf);
  M3NModel* trained_model = trainModel(training_rfs);
  if (trained_model != NULL)
  {
    ROS_INFO("Successfully trained M3n model");
    delete trained_model;
  }
  return 0;
}

// --------------------------------------------------------------
/*!
 * \brief Instantiate and train the model
 *
 * You need to define parameters in this function
 */
// --------------------------------------------------------------
M3NModel* trainModel(vector<const RandomField*>& training_rfs)
{
  // Define parameters for regressors
  // (using default values)
  RegressionTreeWrapperParams regression_tree_params;

  // Define learning parameters
  // TODO HARDCODE
  M3NParams m3n_params;
  m3n_params.setLearningRate(0.5);
  m3n_params.setNumberOfIterations(15);
  m3n_params.setRegressorRegressionTrees(regression_tree_params);
  vector<float> robust_potts_params(1, -1.0);
  m3n_params.setInferenceRobustPotts(robust_potts_params);

  M3NModel* m3n_model = new M3NModel();
  if (m3n_model->train(training_rfs, m3n_params) < 0)
  {
    ROS_ERROR("Failed to train M3N model");
    delete m3n_model;
    return NULL;
  }
  return m3n_model;
}

