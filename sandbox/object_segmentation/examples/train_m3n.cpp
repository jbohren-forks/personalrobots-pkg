/*
 * train_m3n.cpp
 *
 *  Created on: Jul 29, 2009
 *      Author: dmunoz
 */

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/PointCloud.h>

#include <functional_m3n/random_field.h>
#include <functional_m3n/m3n_model.h>
#include <functional_m3n/regressors/regressor_params.h>

#include <object_segmentation/util/rf_creator_3d.h>

extern RFCreator3D* initRFCreator();

extern int loadPointCloud(string filename,
                          unsigned int nbr_cols,
                          sensor_msgs::PointCloud& pt_cloud,
                          vector<unsigned int>& labels);
int main()
{
  unsigned int NUMBER_OF_COLUMNS = 5;

  // ----------------------------------------------------------
  // Load point cloud from file
  sensor_msgs::PointCloud pt_cloud;
  vector<unsigned int> labels;
  loadPointCloud("training_data.xyz_label_conf", NUMBER_OF_COLUMNS, pt_cloud, labels);

  // ----------------------------------------------------------
  // Create random field
  RFCreator3D* rf_creator = initRFCreator();
  const boost::shared_ptr<RandomField> training_rf = rf_creator->createRandomField(pt_cloud,
      labels, true);
  training_rf->saveRandomField("rf_from_memory/train_rf");

  //const boost::shared_ptr<RandomField> loaded_rf(new RandomField(2));
  //loaded_rf->loadRandomField("rf_from_memory/train_rf");
  //loaded_rf->saveRandomField("rf_from_file/train_rf");

  // ----------------------------------------------
  // Define learning parameters
  vector<float> robust_potts_params(training_rf->getNumberOfCliqueSets(), -1.0);
  RegressionTreeWrapperParams regression_tree_params;
  regression_tree_params.max_tree_depth_factor = 0.07; // was 0.2
  M3NParams m3n_learning_params;
  m3n_learning_params.setLearningRate(0.4);
  m3n_learning_params.setNumberOfIterations(6);
  m3n_learning_params.setRegressorRegressionTrees(regression_tree_params);
  m3n_learning_params.setInferenceRobustPotts(robust_potts_params);

  // ----------------------------------------------------------
  // Train M3N model
  ROS_INFO("Starting to train...");
  M3NModel m3n_model;
  vector<const RandomField*> training_rfs(1, training_rf.get());
  if (m3n_model.train(training_rfs, m3n_learning_params) < 0)
  {
    ROS_ERROR("Failed to train M3N model");
    return -1;
  }
  ROS_INFO("Successfully trained M3n model");
  m3n_model.saveToFile("m3n_models/2cs_pn_potts");
}
