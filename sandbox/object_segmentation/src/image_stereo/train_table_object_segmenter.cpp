/*
 * train_m3n.cpp
 *
 *  Created on: Jul 29, 2009
 *      Author: dmunoz
 */

#include <stdlib.h>

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/PointCloud.h>

#include <functional_m3n/random_field.h>
#include <functional_m3n/m3n_model.h>
#include <functional_m3n/regressors/regressor_params.h>

#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main(int argc, char *argv[])
{
  unsigned int NBR_CLIQUE_SETS = 2;
  // ----------------------------
  if (argc != 6)
  {
    ROS_WARN("%s usage: <random field file list> <tree depth factor> <learning rate> <number of iterations> <model basename>", argv[0]);
    return -1;
  }
  char* rf_file_list = argv[1];
  double max_tree_depth = atof(argv[2]); // 0.1
  double learning_rate = atof(argv[3]);   // 0.3
  unsigned int nbr_iters = atoi(argv[4]); // 100
  char* model_basename = argv[5];

  ifstream file_list(rf_file_list);
  if (file_list.is_open() == false)
  {
    ROS_FATAL("Could not open random field file list %s", rf_file_list);
    return -1;
  }

  // ----------------------------------------------------------
  // Load random fields
  vector<const RandomField*> training_rfs;
  while (file_list.eof() == false)
  {
    char rf_basename[256];
    file_list >> rf_basename;
    RandomField* curr_rf = new RandomField(NBR_CLIQUE_SETS);
    if (curr_rf->loadRandomField(rf_basename) < 0)
    {
      abort();
    }
    training_rfs.push_back(curr_rf);
  }

  // ----------------------------------------------
  // Define learning parameters
  vector<float> robust_potts_params(NBR_CLIQUE_SETS, -1.0);
  robust_potts_params[0] = 0.15; // robust potts

  RegressionTreeWrapperParams regression_tree_params;
  regression_tree_params.max_tree_depth_factor = max_tree_depth;

  M3NParams m3n_learning_params;
  m3n_learning_params.setLearningRate(learning_rate);
  m3n_learning_params.setNumberOfIterations(nbr_iters);
  m3n_learning_params.setRegressorRegressionTrees(regression_tree_params);
  m3n_learning_params.setInferenceRobustPotts(robust_potts_params);

  // ----------------------------------------------------------
  // Train M3N model
  ROS_INFO("Starting to train...");
  M3NModel m3n_model;
  if (m3n_model.train(training_rfs, m3n_learning_params) < 0)
  {
    ROS_ERROR("Failed to train M3N model");
    return -1;
  }
  ROS_INFO("Successfully trained M3n model");
  m3n_model.saveToFile(model_basename);
  return 0;
}
