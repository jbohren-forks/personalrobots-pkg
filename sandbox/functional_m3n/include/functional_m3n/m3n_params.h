#ifndef __M3N_PARAMS_H__
#define __M3N_PARAMS_H__
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

#include <vector>

#include <ros/ros.h>

#include <functional_m3n/regressors/regressor_params.h>
#include <functional_m3n/regressors/regressor_wrapper.h>

// --------------------------------------------------------------
//* M3NParams
/**
 * \brief Parameters to train a M3NModel
 */
// --------------------------------------------------------------
class M3NParams
{
  public:
    // --------------------------------------------------------------
    /**
     * \brief Instantiates an invalid container of parameters.  The following
     *        functions must be called to properly define the parameters:
     *            setLearningRate
     *            setNumberOfIterations
     *            setRegressorXYZ (e.g. XYZ = RegressionTrees)
     */
    // --------------------------------------------------------------
    M3NParams() :
      learning_rate_(-1.0), nbr_iterations_(0), regressor_algorithm_(RegressorWrapper::REGRESSION_NONE)
    {
    }

    // --------------------------------------------------------------
    /**
     * \brief Returns flag if all parameters have been defined
     */
    // --------------------------------------------------------------
    bool parametersDefined() const;

    // --------------------------------------------------------------
    /**
     * \brief Defines the step size to be: learning_rate / sqrt(t), where
     *        t is the iteration number
     *
     * \param learning_rate The learning rate, must be positive
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int setLearningRate(const double learning_rate);

    // --------------------------------------------------------------
    /**
     * \brief Returns the learning rate for learning
     */
    // --------------------------------------------------------------
    double getLearningRate() const;

    // --------------------------------------------------------------
    /**
     * \brief Defines the number of subgradient steps for learning
     *
     * \param nbr_iterations The number of iterations, must be positive
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int setNumberOfIterations(const unsigned int nbr_iterations);

    // --------------------------------------------------------------
    /**
     * \brief Returns the number of subgradient steps for learning
     */
    // --------------------------------------------------------------
    unsigned int getNumberOfIterations() const;

    // ===================================================================
    /*! \name Inference related  */
    // ===================================================================
    //@{

    // --------------------------------------------------------------
    /**
     * \brief
     *
     * \param robust_potts_params Truncation parameters q_i that define the robustness of the clique potential
     *                            in clique-set i, as defined in Kohli et al., CVPR 2007.
     *                            Each value must be in the range [0, 0.5[
     */
    // --------------------------------------------------------------
    int setInferenceRobustPotts(const vector<float>& robust_potts_params);

    // --------------------------------------------------------------
    /**
     * \brief
     */
    // --------------------------------------------------------------
    const vector<float>& getRobustPottsParams() const;

    //@}

    // ===================================================================
    /*! \name Regressor related  */
    // ===================================================================
    //@{

    // --------------------------------------------------------------
    /**
     * \brief Returns the type of regressor algorithm defined
     */
    // --------------------------------------------------------------
    RegressorWrapper::algorithm_t getRegressorAlgorithm() const;

    // --------------------------------------------------------------
    /**
     * \brief
     */
    // --------------------------------------------------------------
    int setRegressorRegressionTrees(const RegressionTreeWrapperParams& regression_tree_params);

    // --------------------------------------------------------------
    /**
     * \brief
     */
    // --------------------------------------------------------------
    const RegressionTreeWrapperParams& getRegressionTreeParams() const;

    //@}

  private:
    double learning_rate_;
    unsigned int nbr_iterations_;

    // Inference related
    vector<float> robust_potts_params_;

    // Regressor related
    RegressorWrapper::algorithm_t regressor_algorithm_;
    RegressionTreeWrapperParams regression_tree_params_;
};

#endif

