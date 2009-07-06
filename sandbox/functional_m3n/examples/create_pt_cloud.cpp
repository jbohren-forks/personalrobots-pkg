/*
 * create_pt_cloud.cpp
 *
 *  Created on: Jul 2, 2009
 *      Author: dmunoz
 */

#include <robot_msgs/PointCloud.h>

#include <Eigen/Core>

#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

#include <descriptors_3d/descriptors_3d.h>

#include <functional_m3n/random_field.h>
#include <functional_m3n/m3n_model.h>
#include <functional_m3n/regressors/regressor_params.h>

#include <iostream>
#include <fstream>
#include <vector>

void createNodes(RandomField& rf,
                 const robot_msgs::PointCloud& pt_cloud,
                 cloud_kdtree::KdTree& pt_cloud_kdtree,
                 vector<unsigned int>& labels);

int main()
{
  // ----------------------------------------------------------
  unsigned int nbr_samples = 90000;
  robot_msgs::PointCloud pt_cloud;
  pt_cloud.pts.resize(nbr_samples);
  vector<unsigned int> labels(nbr_samples);

  unsigned int tempo;

  ROS_INFO("loading point cloud...");
  ifstream infile("training_data.xyz_label_conf");
  for (unsigned int i = 0 ; i < nbr_samples ; i++)
  {
    infile >> pt_cloud.pts[i].x;
    infile >> pt_cloud.pts[i].y;
    infile >> pt_cloud.pts[i].z;
    infile >> labels[i];
    infile >> tempo;
  }
  infile.close();
  ROS_INFO("done");
  cloud_kdtree::KdTree* pt_cloud_kdtree = new cloud_kdtree::KdTreeANN(pt_cloud);

  // ----------------------------------------------------------
  RandomField rf(0);
  createNodes(rf, pt_cloud, *pt_cloud_kdtree, labels);

  // ----------------------------------------------------------
  ROS_INFO("Starting to train");
  M3NParams m3n_params;
  m3n_params.setLearningRate(0.1);
  m3n_params.setNumberOfIterations(20);
  RegressionTreeWrapperParams regression_tree_params;
  m3n_params.setRegressorRegressionTrees(regression_tree_params);
  M3NModel m3n_model;
  vector<const RandomField*> training_rfs(1, &rf);
  if (m3n_model.train(training_rfs, m3n_params) < 0)
  {
    ROS_ERROR("wtf");
    abort();
  }

  ROS_INFO("SUCCESS");

  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void createNodes(RandomField& rf,
                 const robot_msgs::PointCloud& pt_cloud,
                 cloud_kdtree::KdTree& pt_cloud_kdtree,
                 vector<unsigned int>& labels)

{
  // Create one feature TYPE
  list<pair<unsigned int, Eigen::MatrixXf*> > created_features;
  list<pair<unsigned int, Eigen::MatrixXf*> >::iterator iter_created_features;
  LocalGeometry geometric_features;
  geometric_features.setData(&pt_cloud, &pt_cloud_kdtree);
  geometric_features.defineNeighborhood(0.15);
  geometric_features.useElevation();
  geometric_features.useNormalOrientation(0.0, 0.0, 1.0);
  geometric_features.useTangentOrientation(0.0, 0.0, 1.0);

  Eigen::MatrixXf* curr_features = NULL;
  float* curr_feature_vals = NULL;
  unsigned int nbr_feature_vals = 0;

  unsigned int nbr_primitives = pt_cloud.pts.size();
  for (unsigned int i = 0 ; i < nbr_primitives ; i++)
  {
    if (i % 1000 == 0)
    {
      ROS_INFO("sample: %u / %u", i, nbr_primitives);
    }
    created_features.clear();
    nbr_feature_vals = 0;

    // -------------------------------
    // Attempt to compute all features
    geometric_features.setInterestPoint(i);
    if (geometric_features.compute(&curr_features, false) == true)
    {
      created_features.push_back(pair<unsigned int, Eigen::MatrixXf*> (geometric_features.getResultSize(),
          curr_features));
      nbr_feature_vals += geometric_features.getResultSize();
    }
    else
    {
      continue;
    }

    curr_feature_vals = static_cast<float*> (malloc(nbr_feature_vals * sizeof(float)));

    // Copy each feature from above
    unsigned int prev_length = 0;
    unsigned int curr_length = 0;
    for (iter_created_features = created_features.begin(); iter_created_features != created_features.end() ; iter_created_features++)
    {
      curr_length = iter_created_features->first;
      memcpy(curr_feature_vals + prev_length, iter_created_features->second->data(), curr_length
          * sizeof(float));
      prev_length += curr_length;
      delete iter_created_features->second;
    }

    // try to create node with features
    if (rf.createNode(curr_feature_vals, nbr_feature_vals, labels[i]) == NULL)
    {
      ROS_ERROR("could not create node %u", i);
      abort();
    }
  }
}
