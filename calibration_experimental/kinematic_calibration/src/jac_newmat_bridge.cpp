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


#include "kinematic_calibration/jac_newmat_bridge.h"

using namespace kinematic_calibration ;

int JacNewmatBridge::jacVectorToNewmat(const vector<LinkParamJacobian>& jacs, const ActiveLinkParams& active, NEWMAT::Matrix& mat, const JacTerms::JacTerms jac_terms )
{
  const unsigned int N = jacs.size() ;
  const int total_cols = active.getNumActive() ;
  int rows_per_jac ;                                    //! \todo How do I make this into a const type?
  switch(jac_terms)
  {
    case JacTerms::ALL :
      rows_per_jac = 6 ;
      break ;
    case JacTerms::ROT :
    case JacTerms::TRANS :
      rows_per_jac = 3 ;
      break ;
    default:
      return -1 ;
  }
  const int total_rows = rows_per_jac*jacs.size() ;

  // Resize mat accordingly
  if (mat.Ncols() != total_cols || mat.Nrows() != (int)(rows_per_jac*jacs.size()) )
    mat.ReSize(total_rows, total_cols) ;

  NEWMAT::Matrix cur_jac_mat(rows_per_jac, total_cols) ;          // Stores our jacobian matrix for our current KDL jacobian
  for (unsigned int i=0; i<N; i++)
  {
    int result ;
    result = jacToNewmat(jacs[i], active, cur_jac_mat, jac_terms) ;
    if (result < 0)
      return result - 100 ;

    // Add cur_jac_mat to mat via a newmat submatrix call
    mat.SubMatrix(rows_per_jac*i+1, rows_per_jac*(i+1), 1, total_cols) = cur_jac_mat ;
  }

  return 0 ;
}

int JacNewmatBridge::jacToNewmat(const LinkParamJacobian& jac, const ActiveLinkParams& active, NEWMAT::Matrix& mat, const JacTerms::JacTerms jac_terms )
{
  unsigned int num_links = active.getNumLinks() ;
  unsigned int num_rows ;

  // Determine how many rows we need
  switch (jac_terms)
  {
    case JacTerms::ALL:
      num_rows = 6 ;
      break ;
    case JacTerms::TRANS:
    case JacTerms::ROT:
      num_rows = 3 ;
      break ;
    default :
      return -1 ;
  }

  // Count up how many columns we're going to need. This is the same as the number of active terms
  int num_cols = active.getNumActive() ;

  // Change the output matrix's size if needed
  if (mat.Ncols() != (int)num_cols || mat.Nrows() != (int)num_rows)
    mat.ReSize(num_rows, num_cols) ;

  // The # of links as defined by the active matrix has to be the same as the # links defined by the LinkParamJacobian
  if (num_links != jac.links_.size())
    return -2 ;

  // Populate the matrix
  unsigned int cur_col = 1 ;             // (Newmat indicies start at 1)

  for (unsigned int i=0; i<num_links; i++)                               // Copy over the translational terms
  {
    const LinkTwists& link_twists = jac.links_[i] ;
    for (unsigned int j=0; j<3; j++)                                     // Copy the translational parameters
    {
      if (active(j,i))
      {
        const KDL::Twist& twist = link_twists.trans_[j] ;
        switch(jac_terms)
        {
          case JacTerms::ALL:
            mat(1, cur_col) = twist.vel.x() ;
            mat(2, cur_col) = twist.vel.y() ;
            mat(3, cur_col) = twist.vel.z() ;
            mat(4, cur_col) = twist.rot.x() ;
            mat(5, cur_col) = twist.rot.y() ;
            mat(6, cur_col) = twist.rot.z() ;
            break ;
          case JacTerms::TRANS:
            mat(1, cur_col) = twist.vel.x() ;
            mat(2, cur_col) = twist.vel.y() ;
            mat(3, cur_col) = twist.vel.z() ;
            break ;
          case JacTerms::ROT:
            mat(1, cur_col) = twist.rot.x() ;
            mat(2, cur_col) = twist.rot.y() ;
            mat(3, cur_col) = twist.rot.z() ;
            break ;
        }
        cur_col++ ;
      }
    }
    for (unsigned int j=3; j<6; j++)                                    // Copy over the rotational parameters
    {
      if (active(j,i))
      {
        const KDL::Twist& twist = link_twists.rot_[j-3] ;
        switch(jac_terms)
        {
          case JacTerms::ALL:
            mat(1, cur_col) = twist.vel.x() ;
            mat(2, cur_col) = twist.vel.y() ;
            mat(3, cur_col) = twist.vel.z() ;
            mat(4, cur_col) = twist.rot.x() ;
            mat(5, cur_col) = twist.rot.y() ;
            mat(6, cur_col) = twist.rot.z() ;
            break ;
          case JacTerms::TRANS:
            mat(1, cur_col) = twist.vel.x() ;
            mat(2, cur_col) = twist.vel.y() ;
            mat(3, cur_col) = twist.vel.z() ;
            break ;
          case JacTerms::ROT:
            mat(1, cur_col) = twist.rot.x() ;
            mat(2, cur_col) = twist.rot.y() ;
            mat(3, cur_col) = twist.rot.z() ;
            break ;
        }
        cur_col++ ;
      }
    }
  }

  if ((int)cur_col != num_cols+1)                                   // If all our data was consistent, we should have exactly filled out entire matrix
    return -3 ;

  return 0 ;
}

