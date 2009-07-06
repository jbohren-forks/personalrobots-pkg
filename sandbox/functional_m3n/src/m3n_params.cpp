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

#include <functional_m3n/m3n_params.h>

using namespace std;

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
bool M3NParams::parametersDefined() const
{
  return learning_rate_ > 0.0 && nbr_iterations_ > 0 && regressor_algorithm_
      != RegressorWrapper::REGRESSION_NONE;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NParams::setLearningRate(const double learning_rate)
{
  if (learning_rate > 0.0)
  {
    learning_rate_ = learning_rate;
    return 0;
  }

  ROS_ERROR("Learning rate must be positive");
  return -1;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
double M3NParams::getLearningRate() const
{
  return learning_rate_;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NParams::setNumberOfIterations(const unsigned int nbr_iterations)
{
  if (nbr_iterations > 0)
  {
    nbr_iterations_ = nbr_iterations;
    return 0;
  }

  ROS_ERROR("Number of iterations must be positive");
  return -1;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
unsigned int M3NParams::getNumberOfIterations() const
{
  return nbr_iterations_;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NParams::setInferenceRobustPotts(const vector<float>& robust_potts_params)
{
  // Verify the truncation parameters are valid:
  //   less than or equal to 0 means use Potts
  //   Robust Potts truncation parameters must be between [0, 0.5]
  for (unsigned int i = 0 ; i < robust_potts_params.size() ; i++)
  {
    if (robust_potts_params[i] > 0.5)
    {
      ROS_ERROR("Invalid Robust Potts truncation parameters.  Must be less than 0.5");
      return -1;
    }
  }

  robust_potts_params_ = robust_potts_params;
  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
const vector<float>& M3NParams::getRobustPottsParams() const
{
  return robust_potts_params_;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
RegressorWrapper::algorithm_t M3NParams::getRegressorAlgorithm() const
{
  return regressor_algorithm_;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
int M3NParams::setRegressorRegressionTrees(const RegressionTreeWrapperParams& regression_tree_params)
{
  if (regression_tree_params.max_tree_depth_factor < 0.0 || regression_tree_params.max_tree_depth_factor
      > 1.0)
  {
    ROS_ERROR("Invalid max_tree_depth_factor");
    return -1;
  }

  if (regression_tree_params.regression_accuracy < 0.0 || regression_tree_params.regression_accuracy > 1.0)
  {
    ROS_ERROR("Invalid regression_accuracy");
    return -1;
  }

  if (regression_tree_params.min_sample_count == 0)
  {
    ROS_ERROR("Invalid min_sample_count");
    return -1;
  }

  regression_tree_params_ = regression_tree_params;
  regressor_algorithm_ = RegressorWrapper::REGRESSION_TREE;
  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
const RegressionTreeWrapperParams& M3NParams::getRegressionTreeParams() const
{
  return regression_tree_params_;
}
