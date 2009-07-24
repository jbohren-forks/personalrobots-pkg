#ifndef __VK_SUBMODULAR_ENERGY_MIN_H__
#define __VK_SUBMODULAR_ENERGY_MIN_H__
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

#include <assert.h>
#include <string>

#include <3rd_party/maxflow-v3.0/graph.h>

#include <ros/ros.h>

using namespace std;

// --------------------------------------------------------------
//* SubmodularEnergyMin
/** 
 * \brief Minimizes a submodular energy function with binary variables
 *
 * Minimization is done using Vladimir Kolmogorov's maxflow library, which
 * is under "research-only" license.
 *
 * This implementation is based on a class originally written by Vladimir Kolmogorov,
 * implementing the graph structure as described in:
 *  What Energy Functions can be Minimized via Graph Cuts?
 *  Vladimir Kolmogorov and Ramin Zabih.
 *  IEEE Transactions on Pattern Analysis and Machine Intelligence, 2004
 */
// --------------------------------------------------------------
namespace vk_energy
{
  class SubmodularEnergyMin
  {
    public:
      typedef Graph<double, double, double>::node_id EnergyVar;

      // --------------------------------------------------------------
      /*!
       * \brief Instantiates empty energy function
       */
      // --------------------------------------------------------------
      SubmodularEnergyMin(int node_num_max, int edge_num_max);

      ~SubmodularEnergyMin()
      {
        delete graph_;
      }
      ;

      // --------------------------------------------------------------
      /*!
       * \brief Adds a new variable to this function
       */
      // --------------------------------------------------------------
      inline const EnergyVar addVariable()
      {
        return graph_->add_node(1);
      }

      // --------------------------------------------------------------
      /*!
       * \brief Adds a unary term E(x) to this function.
       *
       * This function can be called multiple times per variable
       *
       * \param x The variable
       * \param E0 The unary energy when x = 0
       * \param E1 The unary energy when x = 1
       *
       * \return 0 on success, otherwise negative value on error
       */
      // --------------------------------------------------------------
      inline void addUnary(const EnergyVar& x, double E0, double E1)
      {
        // 0 = source
        // 1 = sink
        graph_->add_tweights(x, E1, E0); // it should be backwards
      }

      // --------------------------------------------------------------
      /*!
       * \brief Adds a submodular pairwise term E(x,y) to this function.
       *
       * This function can be called multiple times per variable pairs
       *
       * Warning, the term must be submodular:
       *         E00 + E11 <= E01 + E10
       *
       * \param x The variable
       * \param y The variable
       * \param E00 The pairwise energy when x = 0, y = 0
       * \param E01 The pairwise energy when x = 0, y = 1
       * \param E10 The pairwise energy when x = 1, y = 0
       * \param E11 The pairwise energy when x = 1, y = 1
       *
       * \return 0 on success, otherwise negative value on error
       */
      // --------------------------------------------------------------
      int addPairwise(const EnergyVar& x, const EnergyVar& y, double E00, double E01, double E10, double E11);

      // --------------------------------------------------------------
      /*!
       * \brief Adds a Pn Potts high-order term E(x_1,..,x_n) over n variables, as described
       *        in Kohli et al., "P3 & Beyond: Solving Energies with Higher Order Cliques", CVPR 2007
       *
       * If implementing alpha-expansion and using 0 to represent the alpha label and 1 for the current
       * label, then Ec0 = gamma_alpha and Ec1 = gamma (where gamma = gamma_beta if all nodes in the
       * clique are currently labeled beta, and gamma = Emax otherwise), as described in Equation 38.
       *
       * \param clique_vars List of energy variables that constitute the clique
       * \param Ec0 The energy if all variables in the clique take value 0
       * \param Ec1 The energy if all variables in the clique take value 1
       * \param Emax The max energy value the clique can have
       *
       * \return 0 on success, otherwise negative value on error
       */
      // --------------------------------------------------------------
      int addPnPotts(const list<EnergyVar>& clique_vars, double Ec0, double Ec1, double Emax);

      // --------------------------------------------------------------
      /*!
       * \brief Adds a Robust Pn Potts high-order term E(x_1,...,x_n) over n variables, as described
       *        in Kohli et al., "Robust Higher Order Potentials for Enforcing Label Consistency", IJCV 2009
       *
       * This method implements the alpha-expansion move for the high-order potential. It uses a basic form
       * of the potential where all nodes have equal weight for all labels and the truncation parameter Q is
       * the same for all labels.
       *
       * This method should be called when a dominant label d in the clique can be found, that is
       * when D > P - Q, where D is the number of nodes in the clique that take on label d != alpha,
       * P is the number of nodes in the clique, and Q is the truncation parameter.  Call the method
       * addRobustPottsNoDominantExpand0 if no dominant label can be found.
       *
       * IMPORTANT: this function assumes 0 represents the alpha-label and 1 represents the current
       * labeling.  Ensure to call the other energy functions appropriately.
       *
       * \param node_vars List of energy variables that constitute the clique
       * \param dominant_vars List of energy variables that currently take on the clique's dominant labels
       * \param gamma_alpha The energy if all variables in the clique take on the alpha label (value 0)
       * \param gamma_dominant The energy if dominant variables keep their label (value 1)
       * \param gamma_max The max energy value the clique can have
       * \param Q The truncation parameter, it is the number of nodes that can disagree with
       *          the dominant label.  IMPORTANT: 2Q < clique_vars.size()
       *
       * \return 0 on success, otherwise negative value on error
       */
      // --------------------------------------------------------------
      int addRobustPottsDominantExpand0(const list<EnergyVar>& node_vars,
                                        const list<EnergyVar>& dominant_vars,
                                        double gamma_alpha,
                                        double gamma_dominant,
                                        double gamma_max,
                                        double Q);

      // --------------------------------------------------------------
      /*!
       * \brief Adds a Robust Pn Potts high-order term E(x_1,...,x_n) over n variables, as described
       *        in Kohli et al., "Robust Higher Order Potentials for Enforcing Label Consistency", IJCV 2009
       *
       * This method implements the alpha-expansion move for the high-order potential. It uses a basic form
       * of the potential where all nodes have equal weight for all labels and the truncation parameter Q is
       * the same for all labels.
       *
       * This method should be called when NO dominant label in the clique can be found, that is
       * there is no label d != alpha such that D > P - Q, where D is the number of nodes in the clique that take
       * on label d, P is the number of nodes in the clique, and Q is the truncation parameter.
       *
       * IMPORTANT: this function assumes 0 represents the alpha-label and 1 represents the current
       * labeling.  Ensure to call the other energy functions appropriately.
       *
       * \param node_vars List of energy variables that constitute the clique
       * \param gamma_alpha The energy if all variables in the clique take on the alpha label (value 0)
       * \param gamma_max The max energy value the clique can have
       * \param Q The truncation parameter, it is the number of nodes that can disagree with
       *          the dominant label.  IMPORTANT: 2Q < clique_vars.size()
       *
       * \return 0 on success, otherwise negative value on error
       */
      // --------------------------------------------------------------
      int addRobustPottsNoDominantExpand0(const list<EnergyVar>& node_vars,
                                          double gamma_alpha,
                                          double gamma_max,
                                          double Q);

      // --------------------------------------------------------------
      /*!
       * \brief Minimizes the current energy function
       *
       * \return The minimum energy value
       */
      // --------------------------------------------------------------
      double minimize();

      // --------------------------------------------------------------
      /*!
       * \brief Returns the binary value for the variable
       *
       * This should be called after calling minimize()
       *
       * \return 0 or 1 for the value of x in the minimizing solution
       */
      // --------------------------------------------------------------
      unsigned int getValue(const EnergyVar& x);

    private:
      Graph<double, double, double>* graph_;

      double const_offset_;
  };
}

#endif
