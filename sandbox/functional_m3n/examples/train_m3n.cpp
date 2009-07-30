/*
 * train_m3n.cpp
 *
 *  Created on: Jul 29, 2009
 *      Author: dmunoz
 */

#include <robot_msgs/PointCloud.h>

#include "random_field_creator.h"

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
int loadPointCloud(string filename, robot_msgs::PointCloud& pt_cloud, vector<float>& labels)
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

int main()
{
  // ----------------------------------------------------------
  // Load point cloud from file
  robot_msgs::PointCloud pt_cloud;
  vector<float> labels;
  loadPointCloud("training_data.xyz_label_conf", pt_cloud, labels);

  // ----------------------------------------------------------
  // Create random field
  RandomFieldCreator rf_creator;
  const RandomField* training_rf = rf_creator.createRandomField(pt_cloud, labels);
  training_rf->saveNodeFeatures("tempo/train_node_unknown.txt");
  training_rf->saveCliqueFeatures("tempo/train_rf_unknown");

  // ----------------------------------------------
  // Define learning parameters
  vector<float> robust_potts_params(training_rf->getNumberOfCliqueSets(), -1.0);
  RegressionTreeWrapperParams regression_tree_params;
  regression_tree_params.max_tree_depth_factor = 0.2;
  M3NParams m3n_learning_params;
  m3n_learning_params.setLearningRate(0.4);
  m3n_learning_params.setNumberOfIterations(6);
  m3n_learning_params.setRegressorRegressionTrees(regression_tree_params);
  m3n_learning_params.setInferenceRobustPotts(robust_potts_params);

  // ----------------------------------------------------------
  // Train M3N model
  ROS_INFO("Starting to train...");
  M3NModel m3n_model;
  vector<const RandomField*> training_rfs(1, training_rf);
  if (m3n_model.train(training_rfs, m3n_learning_params) < 0)
  {
    ROS_ERROR("Failed to train M3N model");
    return -1;
  }
  ROS_INFO("Successfully trained M3n model");
  m3n_model.saveToFile("m3n_models/2cs_pn_potts");
}
