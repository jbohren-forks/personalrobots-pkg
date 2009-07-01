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

using namespace std;

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NModel::train(const vector<const RandomField*>& training_rfs, const M3NParams& m3n_params)
{
  // -------------------------------------------
  // Extract the labels to train on
  if (extractVerifyLabelsFeatures(training_rfs) < 0)
  {
    return -1;
  }

  // -------------------------------------------
  // All feature information and labels now defined.
  // Now define the dimension information
  // (This must be called after extractVerifyLabelsFeatures())
  initStackedFeatureIndices();

  // -------------------------------------------
  // Extract training parameters
  // INVARIANT: M3NParams ensures defined parameters are valid
  if (m3n_params.parametersDefined() == false)
  {
    ROS_ERROR("Training parameters are not all defined");
    return -1;
  }
  double step_size = m3n_params.getLearningRate();
  unsigned int nbr_iterations = m3n_params.getNumberOfIterations();
  const vector<double>& robust_potts_params = m3n_params.getRobustPottsParams();

  // -------------------------------------------
  // Ensure the Robust Potts truncation parameters are the same if doing online learning
  if (trained_)
  {
    for (unsigned int i = 0 ; i < robust_potts_params.size() ; i++)
    {
      if (fabs(robust_potts_params[i] - robust_potts_params_[i]) > 1e-5)
      {
        ROS_ERROR("Robust Potts truncation parameters are different");
        return -1;
      }
    }
  }
  robust_potts_params_ = robust_potts_params;

  // -------------------------------------------
  // RandomField related
  const RandomField* curr_rf = NULL;
  map<unsigned int, RandomField::Node*>::const_iterator iter_nodes;
  map<unsigned int, RandomField::Clique*>::const_iterator iter_cliques;
  unsigned int curr_node_id = 0;

  // -------------------------------------------
  // Functional gradient information
  RegressorWrapper* curr_regressor = NULL;
  unsigned int curr_node_gt_label = 0;
  unsigned int curr_node_infer_label = 0;
  unsigned int curr_clique_gt_mode1_label = 0;
  unsigned int curr_clique_gt_mode1_count = 0;
  unsigned int curr_clique_infer_mode1_label = 0;
  unsigned int curr_clique_infer_mode1_count = 0;
  unsigned int curr_clique_gt_mode2_label = 0; // unused
  unsigned int curr_clique_gt_mode2_count = 0; // unused
  unsigned int curr_clique_infer_mode2_label = 0; // unused
  unsigned int curr_clique_infer_mode2_count = 0; // unused
  map<unsigned int, unsigned int> curr_inferred_labeling;
  double curr_step_size = 0.0;
  double gt_residual = 0.0;
  double infer_residual = 0.0;

  // Learn with loss-augmented inference to act as a margin
  // (currently using hamming loss only)
  loss_augmented_inference_ = true;

  // -------------------------------------------
  // Train for the specified number of iterations
  for (unsigned int t = 0 ; t < nbr_iterations ; t++)
  {
    // ---------------------------------------------------
    // Iterate over each RandomField
    for (unsigned int i = 0 ; i < training_rfs.size() ; i++)
    {
      curr_rf = training_rfs[i];
      const map<unsigned int, RandomField::Node*>& nodes = curr_rf->getNodesRandomFieldIDs();
      const vector<map<unsigned int, RandomField::Clique*> >& clique_sets = curr_rf->getCliqueSets();

      // ---------------------------------------------------
      // Perform inference with the current model.
      // If first iteration, generate a random labeling,
      // otherwise, perform inference with the current model.
      if (t == 0)
      {
        // Generate random labeling
        unsigned int nbr_labels = training_labels_.size();
        unsigned int random_label = 0;
        for (iter_nodes = nodes.begin(); iter_nodes != nodes.end() ; iter_nodes++)
        {
          random_label = training_labels_[rand() % nbr_labels];
          curr_inferred_labeling[iter_nodes->first] = random_label;
        }
      }
      else
      {
        curr_inferred_labeling.clear();
        inferPrivate(*curr_rf, curr_inferred_labeling);
      }

      // ---------------------------------------------------
      // Instantiate new regressor
      curr_regressor = instantiateRegressor(m3n_params);
      if (curr_regressor == NULL)
      {
        ROS_ERROR("Could not create new regressor at iteration %u", t);
        return -1;
      }

      // ---------------------------------------------------
      // Create training set for the new regressor from node and clique features.
      // When classification is wrong, do +1/-1 with features with ground truth/inferred label

      // ------------------------
      // Node features
      for (iter_nodes = nodes.begin(); iter_nodes != nodes.end() ; iter_nodes++)
      {
        curr_node_id = iter_nodes->first;
        curr_node_gt_label = iter_nodes->second->getLabel();
        curr_node_infer_label = curr_inferred_labeling[curr_node_id];
        if (curr_node_gt_label != curr_node_infer_label)
        {
          // +1 features with ground truth label
          curr_regressor->addTrainingSample(iter_nodes->second->getFeatureVals(), node_feature_dim_,
              node_stacked_feature_start_idx_[curr_node_gt_label], 1.0);

          // -1 features with inferred label
          curr_regressor->addTrainingSample(iter_nodes->second->getFeatureVals(), node_feature_dim_,
              node_stacked_feature_start_idx_[curr_node_infer_label], -1.0);
        }
      }

      // ------------------------
      // Iterate over clique sets to get clique features
      for (unsigned int clique_set_idx = 0 ; clique_set_idx < clique_sets.size() ; clique_set_idx++)
      {
        // ------------------------
        // Iterate over cliques
        const map<unsigned int, RandomField::Clique*>& curr_cliques = clique_sets[clique_set_idx];
        for (iter_cliques = curr_cliques.begin(); iter_cliques != curr_cliques.end() ; iter_cliques++)
        {
          if (iter_cliques->second->getModeLabels(curr_clique_gt_mode1_label, curr_clique_gt_mode1_count,
              curr_clique_gt_mode2_label, curr_clique_gt_mode2_count) < 0)
          {
            return -1;
          }
          if (iter_cliques->second->getModeLabels(curr_clique_infer_mode1_label,
              curr_clique_infer_mode1_count, curr_clique_infer_mode2_label, curr_clique_infer_mode2_count,
              &curr_inferred_labeling) < 0)
          {
            return -1;
          }

          // ------------------------
          // Compute functional gradient for current clique
          // If mode labels of ground truth and inferred labelings agree, then it will not affect it
          // Otherwise, need to determine if the labeling has non-zero potential based on interaction model
          // (all labels must agree if using Potts model), this is done in calcFuncGradResidual().
          // Only contribute if the residual is non-zero.
          if (curr_clique_gt_mode1_label != curr_clique_infer_mode1_count)
          {
            // ------------------------
            // Compute functional gradient residual from ground truth label (gt_residual)
            gt_residual = calcFuncGradResidual(robust_potts_params_[clique_set_idx],
                iter_cliques->second->getOrder(), curr_clique_gt_mode1_label);

            if (gt_residual > 0.0)
            {
              // +features with ground truth label
              curr_regressor->addTrainingSample(iter_cliques->second->getFeatureVals(),
                  clique_set_feature_dims_[clique_set_idx],
                  clique_set_stacked_feature_start_idx_[clique_set_idx][curr_clique_gt_mode1_label],
                  gt_residual);
            }

            // ------------------------
            // Compute functional gradient residual from inferred label (gt_infer)
            infer_residual = calcFuncGradResidual(robust_potts_params_[clique_set_idx],
                iter_cliques->second->getOrder(), curr_clique_infer_mode1_count);

            if (infer_residual > 0.0)
            {
              curr_regressor->addTrainingSample(iter_cliques->second->getFeatureVals(),
                  clique_set_feature_dims_[clique_set_idx],
                  clique_set_stacked_feature_start_idx_[clique_set_idx][curr_clique_infer_mode1_label],
                  -infer_residual);
            }
          }
        } // end iterate over cliques
      } // end iterate over clique sets

      // ---------------------------------------------------
      // Train and augment regressor to the model
      if (curr_regressor->train() < 0)
      {
        return -1;
      }
      curr_step_size = step_size / sqrt(static_cast<double> (t + 1));
      regressors_.push_back(pair<double, RegressorWrapper*> (curr_step_size, curr_regressor));
    } // end iterate over random fields
  } // end training iterations

  trained_ = true;
  return 0;
}

// --------------------------------------------------------------
/*!
 * See function definition
 * Invariant: truncation_params are valid
 */
// --------------------------------------------------------------
double M3NModel::calcFuncGradResidual(const double truncation_param,
                                      const unsigned int clique_order,
                                      const unsigned int nbr_mode_label)
{
  // If using Robust Potts, determine if allowed number of disagreeing nodes is allowable
  if (truncation_param > 0.0)
  {
    double Q = truncation_param * static_cast<double> (clique_order);

    double residual = static_cast<double> (nbr_mode_label - clique_order) / Q + 1.0;
    if (residual > 0.0)
    {
      return residual;
    }
    else
    {
      return 0.0;
    }
  }
  // Otherwise, a truncation param of 0.0 indicates to use Potts model
  else
  {
    return static_cast<double> (clique_order == nbr_mode_label);
  }
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
RegressorWrapper* M3NModel::instantiateRegressor(const M3NParams& m3n_params)
{
  RegressorWrapper* created_regressor = NULL;

  RegressorWrapper::algorithm_t regressor_algorithm = m3n_params.getRegressorAlgorithm();
  if (regressor_algorithm == RegressorWrapper::REGRESSION_TREE)
  {
    created_regressor = new RegressionTreeWrapper(total_stack_feature_dim_,
        m3n_params.getRegressionTreeParams());
  }
  return created_regressor;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NModel::extractVerifyLabelsFeatures(const vector<const RandomField*>& training_rfs)
{
  map<unsigned int, RandomField::Node*>::const_iterator iter_nodes;
  map<unsigned int, RandomField::Clique*>::const_iterator iter_cliques;

  // flag indicating if its the first time encountering node/clique in random field/clique-set
  bool is_first = true;

  unsigned int curr_label = 0;
  unsigned int curr_feature_dim = 0;

  // ---------------------------------------------------
  // Extract and verify the feature dimensions and labels for each random field
  for (unsigned int i = 0 ; i < training_rfs.size() ; i++)
  {
    // ---------------------------------------------------
    // Check for NULL pointers to random fields
    if (training_rfs[i] == NULL)
    {
      return -1;
    }

    // ---------------------------------------------------
    // Reset flag for encountering first node in the random field
    is_first = true;

    // ---------------------------------------------------
    // Nodes: extract/verify feature dimension and labels
    const map<unsigned int, RandomField::Node*>& nodes = (training_rfs[i])->getNodesRandomFieldIDs();
    for (iter_nodes = nodes.begin(); iter_nodes != nodes.end() ; iter_nodes++)
    {
      curr_label = iter_nodes->second->getLabel();
      curr_feature_dim = iter_nodes->second->getNumberFeatureVals();

      // --------------------------
      // Extract or verify the node feature dimension to be used by this model
      if (!trained_ && is_first)
      {
        node_feature_dim_ = curr_feature_dim;
        is_first = false;
      }
      else if (curr_feature_dim != node_feature_dim_)
      {
        ROS_ERROR("Node features dimensions are NOT the same: %u and %u (node %u, random field %u)",
            node_feature_dim_, curr_feature_dim, iter_nodes->first, i);
        return -1;
      }

      // --------------------------
      // If model not already trained, extract label and feature dimension information
      if (!trained_)
      {
        // Ensure the random field is fully labeled
        if (curr_label == RandomField::UNKNOWN_LABEL)
        {
          ROS_ERROR("RandomField %u contains unlabeled node with id %u", i, iter_nodes->first);
          return -1;
        }
        else
        {
          // Add the label if not encountered it before
          if (find(training_labels_.begin(), training_labels_.end(), curr_label) == training_labels_.end())
          {
            training_labels_.push_back(curr_label);
          }
        }
      }
      // Otherwise, the model is already trained so verify it can handle all encountered labels
      else
      {
        if (find(training_labels_.begin(), training_labels_.end(), curr_label) == training_labels_.end())
        {
          {
            ROS_ERROR("Trained model encountered new label %u in RandomField %u", curr_label, i);
            return -1;
          }
        }
      }
    }// end iteration over nodes

    // ---------------------------------------------------
    // Ensure the node feature dimensions are defined
    if (node_feature_dim_ == 0)
    {
      ROS_ERROR("Did not find any nodes from the RandomField %u", i);
      return -1;
    }

    // ---------------------------------------------------
    // Allocate for the number clique sets if never trained before,
    // Otherwise ensure the number of clique sets are consistent
    const vector<map<unsigned int, RandomField::Clique*> >& clique_sets = (training_rfs[i])->getCliqueSets();
    if (!trained_)
    {
      clique_set_feature_dims_.assign(clique_sets.size(), 0);
    }
    else if (trained_ && clique_set_feature_dims_.size() != clique_sets.size())
    {
      ROS_ERROR("Number of clique sets do NOT match from RandomField %u", i);
      return -1;
    }

    // ---------------------------------------------------
    // Cliques: extract/verify feature dimensions for each clique-set
    for (unsigned int j = 0 ; j < clique_sets.size() ; j++)
    {
      // Reset flag for encountering first clique in the clique-set
      is_first = true;

      // --------------------------
      // Iterate over the cliques in the current clique-set
      for (iter_cliques = clique_sets[j].begin(); iter_cliques != clique_sets[j].end() ; iter_cliques++)
      {
        curr_feature_dim = iter_cliques->second->getNumberFeatureVals();

        // --------------------------
        // If never trained before and is the first clique in the clique set, then record the feature dimensions
        if (!trained_ && is_first)
        {
          clique_set_feature_dims_[j] = curr_feature_dim;
          is_first = false;
        }
        // Otherwise, verify the feature dimensions are consistent
        else if (curr_feature_dim != clique_set_feature_dims_[j])
        {
          ROS_ERROR("Clique features are NOT the same: %u and %u (clique %u, cs idx %u, random field %u)",
              clique_set_feature_dims_[j], curr_feature_dim, iter_cliques->first, j, i);
          return -1;
        }
      } // end iteration over cliques

      // --------------------------
      // Ensure the node feature dimensions are defined
      if (clique_set_feature_dims_[j] == 0)
      {
        ROS_ERROR("Did not find any cliques in clique-set %u from RandomField %u", j, i);
        return -1;
      }
    } // end iteration over clique sets
  } // end iteration over random fields

  // ---------------------------------------------------
  // Store labels in ascending order
  sort(training_labels_.begin(), training_labels_.end());
  return 0;
}
