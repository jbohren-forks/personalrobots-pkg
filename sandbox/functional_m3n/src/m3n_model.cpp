/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Daniel Munoz
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

#include <functional_m3n/m3n_model.h>

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
M3NModel::~M3NModel()
{
  clear();
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void M3NModel::clear()
{
  trained_ = false;

  node_feature_dim_ = 0;
  clique_set_feature_dims_.clear();

  total_stack_feature_dim_ = 0;
  node_stacked_feature_start_idx_.clear();
  clique_set_stacked_feature_start_idx_.clear();

  loss_augmented_inference_ = false;
  training_labels_.clear();
  robust_potts_params_.clear();

  for (unsigned int i = 0 ; i < regressors_.size() ; i++)
  {
    delete (regressors_[i]).second;
  }
  regressors_.clear();
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NModel::loadFromFile(const string& basename)
{
  clear();

  // -------------------------------------------
  // Create filename: <basename>.m3n_model
  string in_filename = basename;
  in_filename.append(".m3n_model");
  ifstream infile(in_filename.c_str());
  if (infile.is_open() == false)
  {
    ROS_ERROR("Could not open file %s to load frome", in_filename.c_str());
    return -1;
  }

  // -------------------------------------------
  // File format:
  // L = <training_labels_.size()>
  // training_labels_[0] ... training_labels_[L-1]
  //
  // S = <robust_potts_params_.size()>
  // robust_potts_params_[0] ... robust_potts_params_[S-1]
  //
  // node_feature_dim_
  // clique_set_feature_dims_[0] ... clique_set_feature_dims_[S-1]
  //
  // T = <regressors_.size()>
  // regressors_[0] step-size
  // regressors_[0] algorithm_type
  // <basename>_regressor0 string length
  // <basename>_regressor0
  // ...
  // regressors_[T-1] step-size
  // regressors_[T-1] algorithm_type
  // <basename>_regressor{T-1} string length
  // <basename>_regressor{T-1}

  // ------------------------
  // read labels
  unsigned int nbr_training_labels = 0;
  infile >> nbr_training_labels;
  training_labels_.assign(nbr_training_labels, 0);
  for (unsigned int i = 0 ; i < nbr_training_labels ; i++)
  {
    infile >> training_labels_[i];
  }

  // ------------------------
  // read robust potts params
  unsigned int nbr_clique_sets = 0;
  infile >> nbr_clique_sets;
  robust_potts_params_.assign(nbr_clique_sets, -1.0);
  for (unsigned int i = 0 ; i < nbr_clique_sets ; i++)
  {
    infile >> robust_potts_params_[i];
  }

  // ------------------------
  // read feature dimensions
  infile >> node_feature_dim_;
  clique_set_feature_dims_.assign(nbr_clique_sets, -1.0);
  for (unsigned int i = 0 ; i < nbr_clique_sets ; i++)
  {
    infile >> clique_set_feature_dims_[i];
  }

  // ------------------------
  // This MUST be called ONLY when the following are defined:
  //   training_labels_
  //   node_feature_dim_
  //   clique_set_feature_dims_
  initStackedFeatureIndices();

  // ------------------------
  // read regressor information
  int algorithm_type = 0;
  unsigned int nbr_regressors = 0;
  unsigned int regressor_basename_strlength = 0;
  RegressorWrapper* curr_regressor = NULL;

  infile >> nbr_regressors;
  regressors_.assign(nbr_regressors, pair<double, RegressorWrapper*> (0.0, NULL));
  for (unsigned int i = 0 ; i < nbr_regressors ; i++)
  {
    // --------------
    // read step-size
    infile >> regressors_[i].first;

    // --------------
    // read algorithm type and instantiate appropriate regressor
    infile >> algorithm_type;
    if (algorithm_type == RegressorWrapper::REGRESSION_TREE)
    {
      curr_regressor = new RegressionTreeWrapper(total_stack_feature_dim_);
    }
    else
    {
      ROS_ERROR("Unknown algorithm type: %d", algorithm_type);
      clear();
      return -1;
    }

    // --------------
    // read basename string length
    infile >> regressor_basename_strlength;

    // --------------
    // read basename
    char regressor_basename[regressor_basename_strlength];
    infile >> regressor_basename;

    // --------------
    // load regressor from file
    if (curr_regressor->loadFromFile(regressor_basename) < 0)
    {
      ROS_ERROR("Could not load regressor %s", regressor_basename);
      clear();
      return -1;
    }

    regressors_[i].second = curr_regressor;
  }

  infile.close();
  trained_ = true;
  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NModel::saveToFile(const string& basename)
{
  // -------------------------------------------
  // Verify the model is trained
  if (!trained_)
  {
    ROS_ERROR("Cannot save untrained M3N model");
    return -1;
  }

  // -------------------------------------------
  // Create filename: <basename>.m3n_model
  string out_filename = basename;
  out_filename.append(".m3n_model");
  ofstream outfile(out_filename.c_str());
  if (outfile.is_open() == false)
  {
    ROS_ERROR("Could not open file %s to save to", out_filename.c_str());
    return -1;
  }

  // -------------------------------------------
  // File format:
  // L = <training_labels_.size()>
  // training_labels_[0] ... training_labels_[L-1]
  //
  // S = <robust_potts_params_.size()>
  // robust_potts_params_[0] ... robust_potts_params_[S-1]
  //
  // node_feature_dim_
  // clique_set_feature_dims_[0] ... clique_set_feature_dims_[S-1]
  //
  // T = <regressors_.size()>
  // regressors_[0] step-size
  // regressors_[0] algorithm_type
  // <basename>_regressor0 string length
  // <basename>_regressor0
  // ...
  // regressors_[T-1] step-size
  // regressors_[T-1] algorithm_type
  // <basename>_regressor{T-1} string length
  // <basename>_regressor{T-1}

  // ------------------------
  // labels
  outfile << training_labels_.size() << endl;
  for (unsigned int i = 0 ; i < training_labels_.size() ; i++)
  {
    outfile << training_labels_[i] << " ";
  }
  outfile << endl;

  outfile << endl;

  // ------------------------
  // robust potts params
  outfile << robust_potts_params_.size() << endl;
  for (unsigned int i = 0 ; i < robust_potts_params_.size() ; i++)
  {
    outfile << robust_potts_params_[i] << " ";
  }
  outfile << endl;

  outfile << endl;

  // ------------------------
  // node & clique set feature dimensions
  outfile << node_feature_dim_ << endl;
  for (unsigned int i = 0 ; i < clique_set_feature_dims_.size() ; i++)
  {
    outfile << clique_set_feature_dims_[i] << " ";
  }
  outfile << endl;

  outfile << endl;

  // ------------------------
  // regressors info
  outfile << regressors_.size() << endl;
  for (unsigned int i = 0 ; i < regressors_.size() ; i++)
  {
    // step-size
    outfile << regressors_[i].first << endl;

    // algorithm type
    outfile << (regressors_[i].second)->getAlgorithmType() << endl;

    // create regressor basename
    stringstream regressor_basename;
    regressor_basename << basename << "_regressor" << i;

    // write regressor basename string legnth
    outfile << regressor_basename.str().length() << endl;

    // write regressor basename
    outfile << regressor_basename.str() << endl;

    if ((regressors_[i].second)->saveToFile(regressor_basename.str()) < 0)
    {
      ROS_ERROR("Could not save regressor %u properly", i);
      outfile.close();
      return -1;
    }
  }

  outfile.close();
  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void M3NModel::initStackedFeatureIndices()
{
  vector<unsigned int>::iterator iter_labels;

  unsigned int curr_start_idx = 0;

  // Populate start index for nodes
  for (iter_labels = training_labels_.begin(); iter_labels != training_labels_.end() ; iter_labels++)
  {
    node_stacked_feature_start_idx_[*iter_labels] = curr_start_idx;
    curr_start_idx += node_feature_dim_;
  }

  // Populate start index for cliques
  clique_set_stacked_feature_start_idx_.assign(clique_set_feature_dims_.size(),
      map<unsigned int, unsigned int> ());
  for (unsigned int i = 0 ; i < clique_set_stacked_feature_start_idx_.size() ; i++)
  {
    for (iter_labels = training_labels_.begin(); iter_labels != training_labels_.end() ; iter_labels++)
    {
      clique_set_stacked_feature_start_idx_[i][*iter_labels] = curr_start_idx;
      curr_start_idx += clique_set_feature_dims_[i];
    }
  }

  total_stack_feature_dim_ = curr_start_idx;
}
