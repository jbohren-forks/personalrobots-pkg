#ifndef __M3N_MODEL_H__
#define __M3N_MODEL_H__
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

#include <stdlib.h>

#include <algorithm>
#include <string>
#include <limits>
#include <vector>
#include <map>
#include <list>
#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>

#include <functional_m3n/submodular_energy_min.h>
#include <functional_m3n/random_field.h>
#include <functional_m3n/m3n_params.h>
#include <functional_m3n/regressors/regressor_includes.h>

using namespace std;

// --------------------------------------------------------------
//* M3NModel
/**
 * \brief Implements training and inference with a Max-Margin Markov Network.
 *
 * This class implements/heavily uses techniques/ideas described in the following publications:
 *
 * [1] D. Munoz, J. A. Bagnell, N. Vandapel, and M. Hebert, "Contextual Classification with Functional Max-Margin Markov Networks", CVPR 2009.
 * [2] B. Taskar, C. Guestrin, and D. Koller, "Max-Margin Markov Networks", NIPS 2003
 * [3] N. Ratliff, D. Silver, and J. A. Bagnell, "Learning to Search: Functional Gradient Techniques for Imitation Learning", Autonomous Robots 2009
 * [4] P. Kohli, L. Ladicky, and P. H. S. Torr, "Robust Higher Order Potentials for Enforcing Label Consistency", CVPR 2008
 * [5] P. Kohli, M. P. Kumar, and P. H. S. Torr, "P3 & Beyond: Solving Energies with Higher Order Cliques", CVPR 2007
 * [6] V. Kolmogorov and R. Zabih, "What Energy Functions can be Minimized via Graph Cuts?", PAMI 2002
 * [7] Y. Boykov, O. Veksler, and R. Zabih, "Fast Approximate Energy Minimization via Graph Cuts", PAMI 2001
 *
 */
// --------------------------------------------------------------
class M3NModel
{
  public:
    M3NModel() :
      trained_(false), node_feature_dim_(0), total_stack_feature_dim_(0)
    {
    }

    ~M3NModel();

    // --------------------------------------------------------------
    /*!
     * \brief Clears this M3N model so it can be retrained from scratch.
     */
    // --------------------------------------------------------------
    void clear();

    // --------------------------------------------------------------
    /*!
     * \brief Saves this M3N model to file
     *
     * \param basename The basename of the file to save the M3N
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int saveToFile(const string& basename);

    // --------------------------------------------------------------
    /*!
     * \brief Loads this M3N model from file
     *
     * \param basename The basename of the file to load the M3N
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int loadFromFile(const string& basename);

    // --------------------------------------------------------------
    /*!
     * \brief Infers the best labeling for the given RandomField using this trained M3NModel.
     *
     *  Edges always follow the Potts model.
     *
     * \param random_field The RandomField to classify, containing Nodes and Cliques
     * \param inferred_labels Mapping node_id -> inferred labels.  If
     * \param max_iterations (Optional) The max number of alpha-expansions to perform.  0 indicates to run
     *                       until convergence.
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int infer(const RandomField& random_field,
              map<unsigned int, unsigned int>& inferred_labels,
              unsigned int max_iterations = 0);
    // TODO auxially, projection (RRR)

    // --------------------------------------------------------------
    /*!
     * \brief Trains this M3NModel using the given labeled RandomFields
     *
     * This method can be called multiple times as when doing online learning.
     *
     * \param training_rfs The RandomFields to train upon.
     * \param m3n_params Training parameters
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int train(const vector<const RandomField*>& training_rfs, const M3NParams& m3n_params);

  private:
    // --------------------------------------------------------------
    /*!
     * \brief Initialize stacked feature indices information
     */
    // --------------------------------------------------------------
    void initStackedFeatureIndices();

    // ===================================================================
    /*! \name Learning related  */
    // ===================================================================
    //@{

    // --------------------------------------------------------------
    /*!
     * \brief Verifies this M3NModel can handle the labels contained in the
     *        given RandomFields.
     *
     * \param training_rfs The RandomFields containing the labels this classifier should handle
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int extractVerifyLabelsFeatures(const vector<const RandomField*>& training_rfs);

    // --------------------------------------------------------------
    /*!
     * \brief Computes the functional gradient residual for high-order cliques.
     *
     * \param truncation_param The Robust Potts truncation factor for the clique
     * \param clique_order The order of the clique
     * \param nbr_mode_label The number of nodes in the clique containing the mode label
     *
     * \return The functional gradient residual
     */
    // --------------------------------------------------------------
    double calcFuncGradResidual(const double truncation_param,
                                const unsigned int clique_order,
                                const unsigned int nbr_mode_label);

    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the specified RegressorWrapper
     *
     * \param m3n_params Parameters specifying the type of RegressorWrapper and
     *                   its parameters
     *
     * \warning Assumes parameters for the RegressorWrapper are valid.
     *
     * \return the instantiated RegressorWrapper on success, otherwise NULL on error
     */
    // --------------------------------------------------------------
    RegressorWrapper* instantiateRegressor(const M3NParams& m3n_params);
    //@}

    // ===================================================================
    /*! \name Inference related  */
    // ===================================================================
    //@{
    // --------------------------------------------------------------
    /*!
     * \brief See infer().  This method does not ensure the model is trained already.
     */
    // --------------------------------------------------------------
    int inferPrivate(const RandomField& random_field,
                     map<unsigned int, unsigned int>& inferred_labels,
                     unsigned int max_iterations = 0);

    // --------------------------------------------------------------
    /*!
     * \brief Computes the potential value for assigning the node the specified label
     *
     * \param node The node containing its features
     * \param label The label to assign node
     * \param potential_val Reference to hold the computed potential value
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int computePotential(const RandomField::Node& node, const unsigned int label, double& potential_val);

    // --------------------------------------------------------------
    /*!
     * \brief Computes the potential value for assigning all the nodes within the
     *        clique the specified label
     *
     * \param clique The clique containing its features
     * \param clique_set_idx The clique set index of clique
     * \param label The label to assign all the nodes in the clique
     * \param potential_val Reference to hold the computed potential value
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int computePotential(const RandomField::Clique& clique,
                         const unsigned int clique_set_idx,
                         const unsigned int label,
                         double& potential_val);

    int addNodeEnergy(const RandomField::Node& node,
                      SubmodularEnergyMin& energy_func,
                      const SubmodularEnergyMin::EnergyVar& energy_var,
                      const unsigned int curr_label,
                      const unsigned int alpha_label);
    //@}

    bool trained_;

    unsigned int node_feature_dim_;
    vector<unsigned int> clique_set_feature_dims_;

    unsigned int total_stack_feature_dim_;
    map<unsigned int, unsigned int> node_stacked_feature_start_idx_;
    vector<map<unsigned int, unsigned int> > clique_set_stacked_feature_start_idx_;

    vector<unsigned int> training_labels_;

    vector<double> robust_potts_params_;

    vector<pair<double, RegressorWrapper*> > regressors_;
};

#endif
