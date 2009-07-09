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
 * \brief Setup the features to compute for the nodes
 */
// --------------------------------------------------------------
unsigned int setupNodeFeatures(const robot_msgs::PointCloud& pt_cloud,
                               cloud_kdtree::KdTree& pt_cloud_kdtree,
                               vector<Descriptor3D*>& feature_descriptors,
                               vector<unsigned int>& feature_descriptor_sizes)
{
  // ----------------------------------------------
  // Geometry feature information
  // TODO HARDCODE
  LocalGeometry* geometry_features = new LocalGeometry();
  geometry_features->setData(&pt_cloud, &pt_cloud_kdtree);
  geometry_features->setInterestRadius(0.15);
  geometry_features->useElevation();
  geometry_features->useNormalOrientation(0.0, 0.0, 1.0);
  geometry_features->useTangentOrientation(0.0, 0.0, 1.0);

  // ----------------------------------------------
  // Clear out any existing descriptors
  for (unsigned int i = 0 ; i < feature_descriptors.size() ; i++)
  {
    delete feature_descriptors[i];
  }

  feature_descriptors.assign(1, geometry_features);
  feature_descriptor_sizes.assign(1, geometry_features->getResultSize());
  return geometry_features->getResultSize();
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void createNodes(RandomField& rf,
                 const robot_msgs::PointCloud& pt_cloud,
                 cloud_kdtree::KdTree& pt_cloud_kdtree,
                 const vector<unsigned int>& labels,
                 set<unsigned int>& failed_indices,
                 vector<float>& feature_avgs)

{
  vector<Descriptor3D*> feature_descriptors;
  vector<unsigned int> feature_descriptor_vals;
  unsigned int nbr_total_feature_vals = setupNodeFeatures(pt_cloud, pt_cloud_kdtree, feature_descriptors,
      feature_descriptor_vals);

  feature_avgs.assign(nbr_total_feature_vals, 0.0);
  vector<Eigen::MatrixXf*> created_features_eigen(feature_descriptors.size(), NULL);

  // --------------------------------------------------
  unsigned int j = 0;
  unsigned int k = 0;
  bool all_features_success = true;
  unsigned int nbr_pts = pt_cloud.pts.size();
  const RandomField::Node* created_node = NULL;

  for (unsigned int i = 0 ; i < nbr_pts ; i++)
  {
    // -------------------------------
    if (i % 1000 == 0)
    {
      ROS_INFO("sample: %u / %u", i, nbr_pts);
    }

    // -------------------------------
    all_features_success = true;
    for (j = 0; all_features_success && j < feature_descriptors.size() ; j++)
    {
      feature_descriptors[j]->setInterestPoint(i);
      all_features_success = feature_descriptors[j]->compute(&(created_features_eigen[j]), false);
    }

    if (!all_features_success)
    {
      for (unsigned int k = 0 ; k < (j - 1) ; k++)
      {
        delete created_features_eigen[k];
      }
      failed_indices.insert(i);
      continue;
    }

    // Copy each feature from above into big concatenated vector
    float* concat_created_feature_vals = static_cast<float*> (malloc(nbr_total_feature_vals * sizeof(float)));
    unsigned int prev_total_nbr_vals = 0;
    for (j = 0; j < feature_descriptors.size() ; j++)
    {
      unsigned int curr_nbr_vals = feature_descriptor_vals[j];
      float* curr_feature_vals = (created_features_eigen[j])->data();

      for (k = 0; k < curr_nbr_vals ; k++)
      {
        concat_created_feature_vals[prev_total_nbr_vals + k] = curr_feature_vals[k];
        feature_avgs[prev_total_nbr_vals + k] += curr_feature_vals[k];
      }
      prev_total_nbr_vals += curr_nbr_vals;

      delete created_features_eigen[j];
    }

    // try to create node with features
    created_node = rf.createNode(i, concat_created_feature_vals, nbr_total_feature_vals, labels[i],
        pt_cloud.pts[i].x, pt_cloud.pts[i].y, pt_cloud.pts[i].z);
    if (created_node == NULL)
    {
      ROS_ERROR("could not create node %u", i);
      abort();
    }
  }

  // Compute average
  for (j = 0; j < nbr_total_feature_vals ; j++)
  {
    feature_avgs[j] /= (static_cast<float> (nbr_pts - failed_indices.size()));
  }
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

  M3NModel* m3n_model = new M3NModel();
  if (m3n_model->train(training_rfs, m3n_params) < 0)
  {
    ROS_ERROR("Failed to train M3N model");
    delete m3n_model;
    return NULL;
  }
  return m3n_model;
}

void tempo()
{
  /*
   //int n = 100000 + 5;
   int n = 100 + 5;
   int d = 6;
   float* matrix = (float*) malloc(n * d * sizeof(float));

   for (unsigned int i = 0 ; i < n - 5 ; i++)
   {
   for (unsigned int j = 0 ; j < d ; j++)
   {
   matrix[i * d + j] = 0.1 + rand() % 10;
   //cout << matrix[i * d + j] << " ";
   }
   //cout << endl;
   }
   for (unsigned int i = n - 5 ; i < n ; i++)
   {
   for (unsigned int j = 0 ; j < d ; j++)
   {
   matrix[i * d + j] = 99999.9;
   }
   }
   int nbr_clusters = 0.1 * n;

   CvMat train_data;
   cvInitMatHeader(&train_data, n, d, CV_32F, matrix);
   CvMat* clusters = cvCreateMat(n, 1, CV_32SC1);

   cout << "starting cluster..." << nbr_clusters;
   cvKMeans2(&train_data, nbr_clusters, clusters, cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 5, 1.0));
   cout << "done." << endl;

   for (unsigned int i = 0 ; i < n ; i++)
   {
   cout << clusters->data.i[i] << endl;
   }
   cvReleaseMat(&clusters);
   */

  /*
   FLANNParameters p;
   p.algorithm = KMEANS;
   p.checks = 2;
   p.cb_index = 0.0;
   p.trees = 2;
   p.branching = 4;
   p.iterations = 3;
   p.centers_init = CENTERS_RANDOM;
   p.target_precision = 0.8;
   p.log_level = LOG_INFO;
   p.log_destination = NULL;

   cout << "doing clustering..." << endl;
   float* poop = (float*) malloc(n * sizeof(float));
   int nbr_found = flann_compute_cluster_centers(matrix, n, d, nbr_clusters, poop, &p);
   cout << "done.   found: " << nbr_found << " (requested " << nbr_clusters << ")" << endl;

   for (int i = 0 ; i < nbr_found ; i++)
   {
   cout << "center: ";
   for (int j = 0 ; j < d ; j++)
   {
   cout << poop[i * d + j] << " ";
   }
   cout << endl;
   }
   free(poop);
   */
}

void clusterFeatures(unsigned int feature_idx_start,
                     unsigned int feature_idx_end,
                     float cluster_factor,
                     const map<unsigned int, RandomField::Node*>& nodes,
                     const vector<float>& feature_avgs,
                     map<unsigned int, list<RandomField::Node*> >& created_clusters)

{
  unsigned int feature_dim = feature_idx_end - feature_idx_start;
  unsigned int feature_dim_xyz = feature_dim + 3;

  map<unsigned int, RandomField::Node*>::const_iterator iter_nodes;
  unsigned int nbr_samples = nodes.size();

  // ----------------------------------------------------------
  // Create matrix for clustering
  float* feature_matrix = static_cast<float*> (malloc(nbr_samples * feature_dim_xyz * sizeof(float)));
  vector<float> cluster_feature_variances(feature_dim, 0.0);

  // Zero mean the feature values
  unsigned int curr_offset = 0;
  const float* curr_node_features = NULL;
  const RandomField::Node* curr_node = NULL;
  unsigned int curr_sample_idx = 0;
  for (iter_nodes = nodes.begin(); iter_nodes != nodes.end() ; iter_nodes++)
  {
    curr_offset = curr_sample_idx * feature_dim_xyz;
    curr_node = iter_nodes->second;

    // Copy xyz coordinates, unnormalized
    feature_matrix[curr_offset] = curr_node->getX();
    feature_matrix[curr_offset + 1] = curr_node->getY();
    feature_matrix[curr_offset + 2] = curr_node->getZ();

    curr_offset += 3;

    curr_node_features = curr_node->getFeatureVals();
    for (unsigned int j = 0 ; j < feature_dim ; j++)
    {
      feature_matrix[curr_offset + j] = curr_node_features[feature_idx_start + j];

      /*
       // subtract mean from features
       feature_matrix[curr_offset + j] = curr_node_features[feature_idx_start + j]
       - feature_avgs[feature_idx_start + j];

       // accumulate variance info for each dimension to cluster over
       cluster_feature_variances[j] += (feature_matrix[curr_offset + j] * feature_matrix[curr_offset + j]);
       */
    }

    // proceed to next sample
    curr_sample_idx++;
  }

  // Unit-variance the feature values (skip xyz coords)
  curr_sample_idx = 0;
  for (iter_nodes = nodes.begin(); iter_nodes != nodes.end() ; iter_nodes++)
  {
    // skip over xyz coords
    curr_offset = curr_sample_idx * feature_dim_xyz + 3;

    curr_node_features = iter_nodes->second-> getFeatureVals();
    for (unsigned int j = 0 ; j < feature_dim ; j++)
    {
      /*
      feature_matrix[curr_offset + j]
          /= sqrt(cluster_feature_variances[j] / static_cast<float> (nbr_samples));
*/
      }
    curr_sample_idx++;
  }

  // ----------------------------------------------------------
  // TODO: use FLANN or something else faster
  CvMat cv_feature_matrix;
  cvInitMatHeader(&cv_feature_matrix, nbr_samples, feature_dim_xyz, CV_32F, feature_matrix);
  CvMat* cluster_labels = cvCreateMat(nbr_samples, 1, CV_32SC1);

  int nbr_clusters = static_cast<int> (nbr_samples * cluster_factor);
  // TODO HARDCODE
  cvKMeans2(&cv_feature_matrix, nbr_clusters, cluster_labels, cvTermCriteria(CV_TERMCRIT_ITER
      + CV_TERMCRIT_EPS, 5, 1.0));

  // Save nodes associated with each cluster
  curr_sample_idx = 0;
  unsigned int curr_cluster_label = 0;
  for (iter_nodes = nodes.begin(); iter_nodes != nodes.end() ; iter_nodes++)
  {
    curr_cluster_label = static_cast<unsigned int> (cluster_labels->data.i[curr_sample_idx]);

    // add empty list if not encountered label before
    if (created_clusters.count(curr_cluster_label) == 0)
    {
      created_clusters[curr_cluster_label] = list<RandomField::Node*> ();
    }

    created_clusters[curr_cluster_label].push_back(iter_nodes->second);

    curr_sample_idx++;
  }

  /*
  // TODO delete this is debug
  ofstream outt("features.txt");
  for (unsigned int i = 0 ; i < nbr_samples ; i++)
  {
    for (unsigned int j = 0 ; j < feature_dim_xyz ; j++)
    {
      outt << feature_matrix[i * feature_dim_xyz + j] << " ";
    }
    outt << endl;
  }
  outt.close();
*/
  cvReleaseMat(&cluster_labels);
  free(feature_matrix);
}

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
  ROS_INFO("Creating random field...");
  RandomField rf(0);
  set<unsigned int> failed_indices;
  vector<float> feature_avgs;
  createNodes(rf, pt_cloud, *pt_cloud_kdtree, labels, failed_indices, feature_avgs);
  ROS_INFO("done");

  map<unsigned int, list<RandomField::Node*> > created_clusters;
  // TODO HARDCODE
  clusterFeatures(0, 3, 0.005, rf.getNodesRandomFieldIDs(), feature_avgs, created_clusters);
  save_clusters(created_clusters);

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

void createCliques(RandomField& rf,
                   const robot_msgs::PointCloud& pt_cloud,
                   cloud_kdtree::KdTree& pt_cloud_kdtree,
                   set<unsigned int>& failed_indices)
{

}
