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

#include <descriptors_3d/local_geometry.h>

#include <functional_m3n/random_field.h>
#include <functional_m3n/m3n_model.h>
#include <functional_m3n/regressors/regressor_params.h>

#include <iostream>
#include <fstream>
#include <vector>

unsigned int setupNodeFeatures(const robot_msgs::PointCloud& pt_cloud,
                               cloud_kdtree::KdTree& pt_cloud_kdtree,
                               vector<Descriptor3D*>& feature_descriptors,
                               vector<unsigned int>& feature_descriptor_sizes);

void createNodes(RandomField& rf,
                 const robot_msgs::PointCloud& pt_cloud,
                 cloud_kdtree::KdTree& pt_cloud_kdtree,
                 vector<unsigned int>& labels);

void loadPointCloud(string filename, robot_msgs::PointCloud& pt_cloud, vector<unsigned int>& labels)
{
  unsigned int nbr_samples = 90000;
  pt_cloud.pts.resize(nbr_samples);
  labels.resize(nbr_samples);

  unsigned int tempo;

  ifstream infile(filename.c_str());
  for (unsigned int i = 0 ; i < nbr_samples ; i++)
  {
    infile >> pt_cloud.pts[i].x;
    infile >> pt_cloud.pts[i].y;
    infile >> pt_cloud.pts[i].z;
    infile >> labels[i];
    infile >> tempo;
  }
  infile.close();
}

int main()
{
  // ----------------------------------------------------------
  ROS_INFO("loading point cloud...");
  robot_msgs::PointCloud pt_cloud;
  vector<unsigned int> labels;
  loadPointCloud("training_data.xyz_label_conf", pt_cloud, labels);
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
  vector<Descriptor3D*> feature_descriptors;
  vector<unsigned int> feature_descriptor_sizes;
  unsigned int nbr_total_feature_vals = setupNodeFeatures(pt_cloud, pt_cloud_kdtree, feature_descriptors,
      feature_descriptor_sizes);

  vector<Eigen::MatrixXf*> created_features(feature_descriptors.size());
  float* concat_created_feature_vals = NULL;

  // --------------------------------------------------
  bool all_features_success = true;
  unsigned int nbr_primitives = pt_cloud.pts.size();
  for (unsigned int i = 0 ; i < nbr_primitives ; i++)
  {
    all_features_success = true;

    // -------------------------------
    if (i % 1000 == 0)
    {
      ROS_INFO("sample: %u / %u", i, nbr_primitives);
    }

    // -------------------------------
    unsigned int j = 0;
    for (j = 0; all_features_success && j < feature_descriptors.size() ; j++)
    {
      feature_descriptors[j]->setInterestPoint(i);
      all_features_success = feature_descriptors[j]->compute(&(created_features[j]), false);
    }

    if (!all_features_success)
    {
      for (unsigned int k = 0 ; k < (j-1) ; k++)
      {
        delete created_features[k];
      }
      continue;
    }

    concat_created_feature_vals = static_cast<float*> (malloc(nbr_total_feature_vals * sizeof(float)));

    // Copy each feature from above
    unsigned int prev_length = 0;
    unsigned int curr_length = 0;
    float* curr_feature_vals = NULL;
    for (j = 0; j < feature_descriptors.size() ; j++)
    {
      curr_length = feature_descriptor_sizes[j];
      curr_feature_vals = (created_features[j])->data();

      memcpy(concat_created_feature_vals + prev_length, curr_feature_vals, curr_length * sizeof(float));

      prev_length += curr_length;

      delete created_features[j];
    }

    // try to create node with features
    if (rf.createNode(concat_created_feature_vals, nbr_total_feature_vals, labels[i]) == NULL)
    {
      ROS_ERROR("could not create node %u", i);
      abort();
    }
  }
}

unsigned int setupNodeFeatures(const robot_msgs::PointCloud& pt_cloud,
                               cloud_kdtree::KdTree& pt_cloud_kdtree,
                               vector<Descriptor3D*>& feature_descriptors,
                               vector<unsigned int>& feature_descriptor_sizes)
{
  for (unsigned int i = 0 ; i < feature_descriptors.size() ; i++)
  {
    delete feature_descriptors[i];
  }

  feature_descriptors.clear();
  feature_descriptor_sizes.clear();

  unsigned int total_result_size = 0;

  // Geometry feature information
  LocalGeometry* geometry_features = new LocalGeometry();
  geometry_features->setData(&pt_cloud, &pt_cloud_kdtree);
  geometry_features->setInterestRadius(0.15);
  geometry_features->useElevation();
  geometry_features->useNormalOrientation(0.0, 0.0, 1.0);
  geometry_features->useTangentOrientation(0.0, 0.0, 1.0);

  feature_descriptors.push_back(geometry_features);
  feature_descriptor_sizes.push_back(geometry_features->getResultSize());
  total_result_size += geometry_features->getResultSize();

  return total_result_size;
}

void createCliques()
{

}
