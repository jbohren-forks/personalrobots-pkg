/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

//! \author Vijay Pradeep

#ifndef KINEMATIC_CALIBRATION_PARAMETER_ESTIMATOR_H_
#define KINEMATIC_CALIBRATION_PARAMETER_ESTIMATOR_H_


#include "kinematic_calibration/link_param_jacobian.h"
#include "kinematic_calibration/link_param_jacobian_solver.h"
#include "kinematic_calibration/jac_newmat_bridge.h"
#include "kinematic_calibration/active_link_params.h"


#include "kdl/chain.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/frames.hpp"

#include <vector>

using namespace std ;

namespace kinematic_calibration
{

class ParameterEstimator
{

public:

  /**
   * Stores a single data point used for calibration. Generally a vector of
   * MarkerData3d will be passed into a calibration routine
   */
  struct MarkerData3d
  {
    MarkerData3d(unsigned int num_joints) : joint_states(num_joints), marker_sensed(0,0,0) { }
    KDL::JntArray joint_states ;          //!< The joint angles recorded
    KDL::Vector marker_sensed ;           //!< The sensed position of a marker
  } ;

  int estimateParametersMarker3d( const KDL::Chain& chain_in, KDL::Chain& chain_out,
                                  const vector<MarkerData3d>& input_data, const ActiveLinkParams& active) ;
  int buildJacobianMat(const KDL::Chain& chain, const vector<MarkerData3d>& input_data,
                       const ActiveLinkParams& active, NEWMAT::Matrix& mat, const JacNewmatBridge::JacTerms::JacTerms jac_terms) ;
  int buildErrorVecMarker3d(const KDL::Chain& chain, const vector<MarkerData3d>& input_data, NEWMAT::ColumnVector& vec,
                            const JacNewmatBridge::JacTerms::JacTerms jac_terms) ;


} ;

}

#endif /* KINEMATIC_CALIBRATION_PARAMETER_ESTIMATOR_H_ */
