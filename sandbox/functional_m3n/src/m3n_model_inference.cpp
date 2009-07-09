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
int M3NModel::infer(const RandomField& random_field,
                    map<unsigned int, unsigned int>& inferred_labels,
                    unsigned int max_iterations)
{
  // -------------------------------------------
  // Ensure the model is trained for public use
  if (!trained_)
  {
    ROS_ERROR("Cannot perform inference because model is not trained yet");
    return -1;
  }

  loss_augmented_inference_ = false; // only use during learning
  return inferPrivate(random_field, inferred_labels, max_iterations);
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NModel::inferPrivate(const RandomField& random_field,
                           map<unsigned int, unsigned int>& inferred_labels,
                           unsigned int max_iterations)
{
  // WARNING: if you change this value, you must change the calls in addNodeEnergy, addCliqueEnergy
  const unsigned int ALPHA_VALUE = 0;

  // -------------------------------------------
  // Retrieve random field information
  const map<unsigned int, RandomField::Node*>& nodes = random_field.getNodesRandomFieldIDs();
  const vector<map<unsigned int, RandomField::Clique*> >& clique_sets = random_field.getCliqueSets();
  map<unsigned int, RandomField::Node*>::const_iterator iter_nodes;
  map<unsigned int, RandomField::Clique*>::const_iterator iter_cliques;
  unsigned int nbr_clique_sets = clique_sets.size();
  const RandomField::Node* curr_node = NULL;
  const RandomField::Clique* curr_clique = NULL;
  unsigned int curr_clique_order = 0;
  unsigned int curr_node_id = 0;
  //unsigned int curr_clique_id = 0;

  // -------------------------------------------
  // Verify clique interaction parameters agree
  if (clique_sets.size() != robust_potts_params_.size())
  {
    ROS_ERROR("Number of clique sets in random field (%u) does not match the model (%u)",
        clique_sets.size(), robust_potts_params_.size());
    return -1;
  }

  // -------------------------------------------
  // Setup label information.
  // Generate random initializing labeling if passed empty labeling
  unsigned int nbr_labels = training_labels_.size();
  bool use_random_init_labeling = inferred_labels.size() != nodes.size();
  if (use_random_init_labeling)
  {
    // This will be populated on first iteration below
    inferred_labels.clear();
  }

  // -------------------------------------------
  // Setup alpha-expansion information
  map<unsigned int, SubmodularEnergyMin::EnergyVar> energy_vars;
  map<unsigned int, SubmodularEnergyMin::EnergyVar>::iterator iter_energy_vars;
  SubmodularEnergyMin* energy_func = NULL;
  unsigned int alpha_label = RandomField::UNKNOWN_LABEL;
  double prev_energy = 0.0;
  double curr_energy = 0.0;
  if (max_iterations == 0)
  {
    max_iterations = numeric_limits<unsigned int>::max();
  }

  // -------------------------------------------
  // Loop until hit max number of iterations or converged on minimum energy
  int ret_val = 0;
  bool done = false;
  for (unsigned int t = 0 ; ret_val == 0 && !done && t < max_iterations ; t++)
  {
    done = true; // will be set false if energy changes
    // -------------------------------------------
    // Alpha-expand over each label
    for (unsigned int label_idx = 0 ; ret_val == 0 && label_idx < nbr_labels ; label_idx++)
    {
      alpha_label = training_labels_[label_idx];

      // -------------------------------------------
      // Create new energy function
      energy_func = new SubmodularEnergyMin();
      energy_vars.clear();

      // -------------------------------------------
      // Create energy variables and Compute node scores
      for (iter_nodes = nodes.begin(); ret_val == 0 && iter_nodes != nodes.end() ; iter_nodes++)
      {
        curr_node_id = iter_nodes->first;
        curr_node = iter_nodes->second;

        // ------------------------
        // If passed empty labeling, the generate random labeling on very first pass
        if (use_random_init_labeling)
        {
          inferred_labels[iter_nodes->first] = training_labels_[rand() % nbr_labels];
        }

        // Add new energy variable
        energy_vars[curr_node_id] = energy_func->addVariable();

        ret_val = addNodeEnergy(*curr_node, *energy_func, energy_vars[curr_node_id],
            inferred_labels[curr_node_id], alpha_label);
      }
      use_random_init_labeling = false;

      // -------------------------------------------
      // Iterate over clique sets to compute cliques' scores
      for (unsigned int cs_idx = 0 ; ret_val == 0 && cs_idx < nbr_clique_sets ; cs_idx++)
      {
        // -------------------------------------------
        // Iterate over clique scores
        const map<unsigned int, RandomField::Clique*>& curr_clique_set = clique_sets[cs_idx];
        for (iter_cliques = curr_clique_set.begin(); ret_val == 0 && iter_cliques != curr_clique_set.end() ; iter_cliques++)
        {
          curr_clique = iter_cliques->second;
          curr_clique_order = curr_clique->getOrder();
          if (curr_clique_order < 2)
          {
            ROS_WARN("Clique with id %u in clique-set %u has order %u. This should not happen...skipping it",
                iter_cliques->first, cs_idx, curr_clique_order);
            continue;
          }
          // add edge potential (assuming all edges are associative)
          else if (curr_clique_order == 2)
          {
            ret_val = addEdgeEnergy(*curr_clique, cs_idx, *energy_func, energy_vars, inferred_labels,
                alpha_label);
          }
          // add high-order clique potential
          else
          {
            // add Robust Pn Potts
            if (robust_potts_params_[cs_idx] > 1e-5)
            {
              ret_val = addCliqueEnergyRobustPotts(*curr_clique, cs_idx, *energy_func, energy_vars,
                  inferred_labels, alpha_label);
            }
            // add Pn Potts
            else
            {
              ret_val = addCliqueEnergyPotts(*curr_clique, cs_idx, *energy_func, energy_vars,
                  inferred_labels, alpha_label);
            }
          }
        }
      }

      // -------------------------------------------
      // Minimize function if there are no errors
      if (ret_val == 0)
      {
        curr_energy = energy_func->minimize();

        // Update labeling if: first iteration OR energy decreases with expansion move
        if ((t == 0 && label_idx == 0) || (curr_energy + 0.0001 < prev_energy))
        {
          // Change respective labels to the alpha label
          for (iter_energy_vars = energy_vars.begin(); iter_energy_vars != energy_vars.end() ; iter_energy_vars++)
          {
            if (energy_func->getValue(iter_energy_vars->second) == ALPHA_VALUE)
            {
              inferred_labels[iter_energy_vars->first] = alpha_label;
            }
          }

          // Made an alpha expansion, so did not reach convergence yet
          prev_energy = curr_energy;
          done = false;
        }
      }

      delete energy_func;
    }
  }
  return ret_val;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NModel::addNodeEnergy(const RandomField::Node& node,
                            SubmodularEnergyMin& energy_func,
                            const SubmodularEnergyMin::EnergyVar& energy_var,
                            const unsigned int curr_label,
                            const unsigned int alpha_label)
{
  // Compute scores for assigning specified labels
  double curr_score = 0.0;
  double alpha_score = 0.0;
  if (computePotential(node, alpha_label, alpha_score) < 0)
  {
    return -1;

  }
  if (curr_label == alpha_label)
  {
    // reuse computation
    curr_score = alpha_score;
  }
  else if (computePotential(node, curr_label, curr_score) < 0)
  {
    return -1;
  }

  // Implement hamming loss margin (during training only)
  if (loss_augmented_inference_)
  {
    if (node.getLabel() != curr_label)
    {
      curr_score += 1.0;
    }
    if (node.getLabel() != alpha_label)
    {
      alpha_score += 1.0;
    }
  }

  // WARNING, this follows that ALPHA_VALUE == 0
  // max +score = min -score
  energy_func.addUnary(energy_var, -alpha_score, -curr_score);
  return 0;
}

// --------------------------------------------------------------
/*! See function definition.  Assumes edge contains only 2 nodes */
// --------------------------------------------------------------
int M3NModel::addEdgeEnergy(const RandomField::Clique& edge,
                            const unsigned int clique_set_idx,
                            SubmodularEnergyMin& energy_func,
                            const map<unsigned int, SubmodularEnergyMin::EnergyVar>& energy_vars,
                            const map<unsigned int, unsigned int>& curr_labeling,
                            const unsigned int alpha_label)
{
  // Retrieve the ids of the node in the edge
  const list<unsigned int>& node_ids = edge.getNodeIDs();
  unsigned int node1_id = node_ids.front();
  unsigned int node2_id = node_ids.back();

  // Retrieve the nodes current labels
  unsigned int node1_label = curr_labeling.find(node1_id)->second;
  unsigned int node2_label = curr_labeling.find(node2_id)->second;

  double E00 = 0.0;
  double E01 = 0.0;
  double E10 = 0.0;
  double E11 = 0.0;

  // Compute score if both nodes switch to alpha (0)
  if (computePotential(edge, clique_set_idx, alpha_label, E00) < 0)
  {
    return -1;
  }

  // Compute score if node1 switches to alpha (0) & node2 stays the same (1)
  if (node2_label == alpha_label)
  {
    // reuse computation
    E01 = E00;
  }

  // Compute score if node1 stays the same (1) & node2 switches to alpha (0)
  if (node1_label == alpha_label)
  {
    // reuse computation
    E10 = E00;
  }

  // Compute score if both nodes stay the same (1)
  if (node1_label == node2_label)
  {
    if (node1_label == alpha_label)
    {
      // reuse computation
      E11 = E00;
    }
    else if (computePotential(edge, clique_set_idx, node1_label, E11) < 0)
    {
      return -1;
    }
  }

  // WARNING, this follows that ALPHA_VALUE == 0
  // max +score = min -score
  if (energy_func.addPairwise(energy_vars.find(node1_id)->first, energy_vars.find(node2_id)->first, -E00,
      -E01, -E10, -E11) < 0)
  {
    return -1;
  }
  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NModel::addCliqueEnergyPotts(const RandomField::Clique& clique,
                                   const unsigned int clique_set_idx,
                                   SubmodularEnergyMin& energy_func,
                                   const map<unsigned int, SubmodularEnergyMin::EnergyVar>& energy_vars,
                                   const map<unsigned int, unsigned int>& curr_labeling,
                                   const unsigned int alpha_label)
{
  unsigned int mode1_label = 0;
  unsigned int mode1_count = 0;
  unsigned int mode2_label = 0;
  unsigned int mode2_count = 0;

  // -----------------------------------
  // Compute potential if all node switch to alpha
  double Ec0 = 0.0;
  if (computePotential(clique, clique_set_idx, alpha_label, Ec0) < 0)
  {
    return -1;
  }

  // -----------------------------------
  // Compute potential if all nodes keep their current labeling
  if (clique.getModeLabels(mode1_label, mode1_count, mode2_label, mode2_count, NULL, &curr_labeling) < 0)
  {
    return -1;
  }
  // This will be non-zero only when all nodes are labeled the same.
  double Ec1 = 0.0;
  if (mode2_label == RandomField::UNKNOWN_LABEL)
  {
    if (mode1_label == alpha_label)
    {
      // use precomputed value
      Ec1 = Ec0;
    }
    else if (computePotential(clique, clique_set_idx, mode1_label, Ec1) < 0)
    {
      return -1;
    }
  }

  // -----------------------------------
  // Create list of energy variables that represent the nodes in this clique
  list<SubmodularEnergyMin::EnergyVar> node_vars;
  const list<unsigned int>& node_ids = clique.getNodeIDs();
  list<unsigned int>::const_iterator iter_node_ids;
  for (iter_node_ids = node_ids.begin(); iter_node_ids != node_ids.end() ; iter_node_ids++)
  {
    node_vars.push_back(energy_vars.find(*iter_node_ids)->second);
  }

  // -----------------------------------
  // WARNING, this follows that ALPHA_VALUE == 0
  // max +score = min -score
  if (energy_func.addPnPotts(node_vars, -Ec0, -Ec1, 0.0) < 0)
  {
    return -1;
  }
  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NModel::addCliqueEnergyRobustPotts(const RandomField::Clique& clique,
                                         const unsigned int clique_set_idx,
                                         SubmodularEnergyMin& energy_func,
                                         const map<unsigned int, SubmodularEnergyMin::EnergyVar>& energy_vars,
                                         const map<unsigned int, unsigned int>& curr_labeling,
                                         const unsigned int alpha_label)
{
  unsigned int mode1_label = 0;
  unsigned int mode1_count = 0;
  unsigned int mode2_label = 0;
  unsigned int mode2_count = 0;

  // -----------------------------------
  // Compute the mode labels in the clique
  list<unsigned int> dominant_node_ids;
  if (clique.getModeLabels(mode1_label, mode1_count, mode2_label, mode2_count, &dominant_node_ids,
      &curr_labeling) < 0)
  {
    return -1;
  }

  // -----------------------------------
  // Determine if a "dominant" label exists: D > P-Q where D is the number of
  // nodes labeled d != alpha, P = number of nodes in the clique, Q = truncation parameter
  // (Note: it must be the mode1_label if dominant label exists).
  // If it exists, compute the clique potential as if all nodes
  // in the clique were assigned that label
  bool found_dominant_label = false;
  double gamma_dominant = -1.0;
  double Q = 0.0;
  if (mode1_label != alpha_label)
  {
    double D = static_cast<double> (mode1_count);
    double P = static_cast<double> (clique.getOrder());
    Q = static_cast<double> (robust_potts_params_[clique_set_idx]) * P;
    if ((D - 1e-5) > (P - Q)) // condition if dominant label exists
    {
      if (computePotential(clique, clique_set_idx, mode1_label, gamma_dominant) < 0)
      {
        return -1;
      }
      found_dominant_label = true;
    }
  }

  // -----------------------------------
  // Compute potential if all nodes switch to alpha
  double gamma_alpha = 0.0;
  if (computePotential(clique, clique_set_idx, alpha_label, gamma_alpha) < 0)
  {
    return -1;
  }

  // -----------------------------------
  // Create list of energy variables of the nodes in the clique.
  // Also save another list containing the variables of just the
  // dominant nodes, if indicated to.
  list<unsigned int>::iterator iter_dominant_node_ids;
  if (found_dominant_label)
  {
    iter_dominant_node_ids = dominant_node_ids.begin();
  }
  else
  {
    iter_dominant_node_ids = dominant_node_ids.end();
  }
  list<SubmodularEnergyMin::EnergyVar> node_vars;
  list<SubmodularEnergyMin::EnergyVar> dominant_vars;
  const list<unsigned int>& node_ids = clique.getNodeIDs();
  list<unsigned int>::const_iterator iter_node_ids;
  for (iter_node_ids = node_ids.begin(); iter_node_ids != node_ids.end() ; iter_node_ids++)
  {
    // save energy variable of all nodes in the clique
    node_vars.push_back(energy_vars.find(*iter_node_ids)->second);

    // save the energy variables of only the nodes that take on the dominant label
    if (iter_dominant_node_ids != dominant_node_ids.end())
    {
      dominant_vars.push_back(energy_vars.find(*iter_dominant_node_ids)->second);
      iter_dominant_node_ids++;
    }
  }

  // -----------------------------------
  // WARNING, this follows that ALPHA_VALUE == 0
  // max +score = min -score
  int ret_val = 0;
  if (found_dominant_label)
  {
    ret_val = energy_func.addRobustPottsDominantExpand0(node_vars, dominant_vars, -gamma_alpha,
        -gamma_dominant, 0.0, Q);
  }
  else
  {
    ret_val = energy_func.addRobustPottsNoDominantExpand0(node_vars, -gamma_alpha, 0.0, Q);
  }
  return ret_val;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NModel::computePotential(const RandomField::Node& node, const unsigned int label, double& potential_val)
{
  potential_val = 0.0;

  // Sum the scores of each regressor
  unsigned int nbr_regressors = regressors_.size();
  float curr_predicted_val = 0.0;
  double curr_step_size = 0.0;
  RegressorWrapper* curr_regressor = NULL;
  for (unsigned int i = 0 ; i < nbr_regressors ; i++)
  {
    curr_step_size = regressors_[i].first;
    curr_regressor = regressors_[i].second;

    if (curr_regressor->predict(node.getFeatureVals(), node_feature_dim_,
        node_stacked_feature_start_idx_[label], curr_predicted_val) < 0)
    {
      return -1;
    }

    potential_val += (curr_step_size * static_cast<double> (curr_predicted_val));
  }

  // Exponentiated functional gradient descent
  potential_val = exp(potential_val);
  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NModel::computePotential(const RandomField::Clique& clique,
                               const unsigned int clique_set_idx,
                               const unsigned int label,
                               double& potential_val)
{
  potential_val = 0.0;

  // Sum the scores of each regressor
  unsigned int nbr_regressors = regressors_.size();
  float curr_predicted_val = 0.0;
  double curr_step_size = 0.0;
  RegressorWrapper* curr_regressor = NULL;
  for (unsigned int i = 0 ; i < nbr_regressors ; i++)
  {
    curr_step_size = regressors_[i].first;
    curr_regressor = regressors_[i].second;

    if (curr_regressor->predict(clique.getFeatureVals(), clique_set_feature_dims_[clique_set_idx],
        clique_set_stacked_feature_start_idx_[clique_set_idx][label], curr_predicted_val) < 0)
    {
      return -1;
    }

    potential_val += (curr_step_size * static_cast<double> (curr_predicted_val));
  }

  // Exponentiated functional gradient descent
  potential_val = exp(potential_val);
  return 0;
}
