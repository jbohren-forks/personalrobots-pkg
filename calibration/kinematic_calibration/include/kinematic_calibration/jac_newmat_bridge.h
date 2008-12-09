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

#ifndef KINEMATIC_CALIBRATION_JAC_NEWMAT_BRIDGE_H_
#define KINEMATIC_CALIBRATION_JAC_NEWMAT_BRIDGE_H_

#include "kinematic_calibration/active_link_params.h"
#include "newmat10/newmat.h"
#include "kinematic_calibration/link_param_jacobian.h"

namespace kinematic_calibration
{

namespace JacNewmatBridge
{

//! Specifies which terms should be outputted by the bridge functions
namespace JacTerms
{
  enum JacTerms {ALL, ROT, TRANS} ;
}

/**
 * Builds a matrix with stacked jacobians, with each jacobian being a row.
 * \jacs Vector of Jacobians. Note that every LinkParamJacobian must have the same number of links.
 *          Otherwise this method will fail (and the matrix representation wouldn't make any sense anyways)
 * \active Specifies which link parameters are active. The number of active parameters is equal to the number of cols in the output mat
 * \mat The output matrix where the jacobian is stored. If this is sized correctly, then it won't be reallocated, thus saving time.
 * \jac_term Specifies whether each jacobian should store either translational terms \
 *               or rotational terms. If both are stored each jac has 6 rows, with \
 *               the first 3 being translational, and the last 3 being rotational
 * \return negative on error
 */
int jacVectorToNewmat(const vector<LinkParamJacobian>& jacs, const ActiveLinkParams& active, NEWMAT::Matrix& mat, const JacTerms::JacTerms jac_term ) ;

/**
 * Converts a linkparam jacobian into a newmat matrix. The original jacobian is
 * also pruned such that it only have the columns corresponding to the free matrix.
 * \param jac The input jacobian. The # of links in this jacobian must correspond to \
 *               the number of columns in active. Each true flag in active corresponds \
 *               to a column in jac
 * \active Specifies which link parameters are active.
 * \jac_term Specifies whether the jacobian should store either translational terms \
 *               or rotational terms. If both are stored then jac has 6 rows, with \
 *               the first 3 being translational, and the last 3 being rotational
 * \return negative on error
 */
int jacToNewmat(const LinkParamJacobian& jac, const ActiveLinkParams& active, NEWMAT::Matrix& mat, const JacTerms::JacTerms jac_term ) ;


}

}



#endif /* KINEMATIC_CALIBRATION_JAC_NEWMAT_BRIDGE_H_ */
