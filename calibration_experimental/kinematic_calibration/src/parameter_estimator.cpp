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

#include "kinematic_calibration/parameter_estimator.h"
#include "kinematic_calibration/chain_modifier.h"
#include "kdl/chainfksolverpos_recursive.hpp"

#define show(var) printf(" %s =%lf\n", #var, (double)var)

#define show_mat(var) \
  printf("%s=\n", #var) ;\
  for(int i=1; i<=var.Nrows(); i++) \
  { \
    printf("  ") ;\
    for (int j=1; j<=var.Ncols(); j++) \
      printf("% 10.7f  ", var(i,j)) ; \
    printf("\n") ; \
  }

#define show_col_vec(var) \
  printf("%s=\n", #var) ;\
  for(int i=1; i<=var.Nrows(); i++) \
  { \
    printf("  ") ;\
    printf("% 15.12f  ", var(i)) ; \
    printf("\n") ; \
  }


using namespace kinematic_calibration ;

static const unsigned int MAX_ITERATIONS = 10 ;      //! \todo Make this some form of changeable parameter

int ParameterEstimator::estimateParametersMarker3d( const KDL::Chain& chain_in, KDL::Chain& chain_out,
                                                    const vector<MarkerData3d>& input_data,  const ActiveLinkParams& active)
{
  int result ;

  assert(chain_in.getNrOfJoints() == chain_in.getNrOfSegments() ) ;     // Not yet accounting for the case where there are more joints than segments

  KDL::Chain cur_chain(chain_in) ;              // Stores

  ChainModifier chain_modifier  ;

  unsigned int iteration_count = 0   ;
  while(iteration_count < MAX_ITERATIONS)
  {
    //printf("Iteration: %u\n", iteration_count) ;
    NEWMAT::Matrix jacs_mat ;
    result = buildJacobianMat(cur_chain, input_data, active, jacs_mat, JacNewmatBridge::JacTerms::TRANS) ;
    if (result < 0)
      return result-100 ;

    NEWMAT::ColumnVector error_vec ;
    result = buildErrorVecMarker3d(cur_chain, input_data, error_vec, JacNewmatBridge::JacTerms::TRANS) ;
    if (result < 0)
      return result-200 ;
    double cur_rms_error = sqrt(error_vec.SumSquare()/error_vec.Nrows()) ;


    // Solve the current least squares problem
    const NEWMAT::Matrix A = jacs_mat ;
    const NEWMAT::Matrix b = error_vec ;

    /*show_mat(A) ;
    NEWMAT::Matrix AtA = A.t()*A ;
    show_mat(AtA) ;
    NEWMAT::Matrix AtA_i = AtA.i() ;
    show_mat(AtA_i) ;
    NEWMAT::Matrix AtA_i_At = AtA_i*A.t() ;
    show_mat(AtA_i_At) ;
    show_mat(b) ;
    NEWMAT::Matrix correction_mat = AtA_i_At * b ;
    show_mat(correction_mat) ;*/

    NEWMAT::ColumnVector correction_mat = (A.t()*A).i() * A.t() * b ;        //! \todo Use more efficient LeastSquares. ConjGrad?

    double alpha = 1.0 ;
    // Apply the correction
    KDL::Chain next_chain(cur_chain) ;
    while(true)
    {

      next_chain = cur_chain ;
      std::vector<double> correction_params ;
      correction_params.resize(correction_mat.Nrows()) ;

      for (int i=0; i<correction_mat.Nrows(); i++)
        correction_params[i] = correction_mat(i+1) * alpha ;

      result = chain_modifier.specifyActiveParams(correction_params, active) ;
      if (result < 0)
        return result-300 ;

      //printf("CorrectionParam: %15.10f\n", correction_params[0]) ;

      KDL::Frame cur_frame ;
      cur_frame = next_chain.getSegment(0).getFrameToTip() ;
      //printf("Before Mod:\n") ;
      //printf(" Translation:\n") ;
      //printf(" [ %15.12f %15.12f %15.12f ]\n", cur_frame.p.data[0], cur_frame.p.data[1], cur_frame.p.data[2]) ;
      //printf(" Rotation:\n") ;
      //printf(" [ %15.12f %15.12f %15.12f \n", cur_frame.M.data[0], cur_frame.M.data[1], cur_frame.M.data[2]) ;
      //printf("   %15.12f %15.12f %15.12f \n", cur_frame.M.data[3], cur_frame.M.data[4], cur_frame.M.data[5]) ;
      //printf("   %15.12f %15.12f %15.12f]\n", cur_frame.M.data[6], cur_frame.M.data[7], cur_frame.M.data[8]) ;

      result = chain_modifier.modifyChain(next_chain) ;
      if (result < 0)
        return result-400 ;

      cur_frame = next_chain.getSegment(0).getFrameToTip() ;
      //printf("After Mod:\n") ;
      //printf("Before Mod:\n") ;
      //printf(" Translation:\n") ;
      //printf(" [ %15.12f %15.12f %15.12f ]\n", cur_frame.p.data[0], cur_frame.p.data[1], cur_frame.p.data[2]) ;
      //printf(" Rotation:\n") ;
      //printf(" [ %15.12f %15.12f %15.12f \n", cur_frame.M.data[0], cur_frame.M.data[1], cur_frame.M.data[2]) ;
      //printf("   %15.12f %15.12f %15.12f \n", cur_frame.M.data[3], cur_frame.M.data[4], cur_frame.M.data[5]) ;
      //printf("   %15.12f %15.12f %15.12f]\n", cur_frame.M.data[6], cur_frame.M.data[7], cur_frame.M.data[8]) ;

      NEWMAT::ColumnVector next_error_vec ;
      result = buildErrorVecMarker3d(next_chain, input_data, next_error_vec, JacNewmatBridge::JacTerms::TRANS) ;
      if (result < 0)
        return result-500 ;
      double next_rms_error = sqrt(next_error_vec.SumSquare()/next_error_vec.Nrows()) ;
      //printf("   NextRMS: %f\n", next_rms_error) ;
      //printf("   CurRMS: %f\n", cur_rms_error) ;
      if (next_rms_error >= cur_rms_error)
      {
        alpha *= .5 ;
      }
      else
        break ;

      if (alpha < .00001)
        break ;
    }

    if (sqrt(correction_mat.SumSquare()*alpha*alpha) < 1e-10)         // If we're making really small corrections, then we're probably done
    {
      break ;
    }

    // Set up for next iteration
    cur_chain = next_chain ;
    iteration_count++ ;
  }

  //show(iteration_count) ;

  chain_out = cur_chain ;

  return 0 ;
}

int ParameterEstimator::buildJacobianMat(const KDL::Chain& chain, const vector<MarkerData3d>& input_data,
                                         const ActiveLinkParams& active, NEWMAT::Matrix& mat, const JacNewmatBridge::JacTerms::JacTerms jac_terms)
{
  const unsigned int N = input_data.size() ;    // # of Data points

  int result ;

  // Allocate memory
  vector<LinkParamJacobian> jacs ;              // Stores all the jacobians in native KDL format. Should this be a class variable?
  jacs.resize(N) ;

  // Compute all the necessary jacobians
  for(unsigned int i=0; i<N; i++)
  {
    result = LinkParamJacobianSolver::JointsToCartesian(chain, input_data[i].joint_states, jacs[i]) ;
    if (result < 0)
      return result-10 ;
  }

  // Convert the KDL jacobians into a newmat matrix
  result = JacNewmatBridge::jacVectorToNewmat(jacs, active, mat, jac_terms ) ;
  if (result < 0)
    return result-20 ;

  return 0 ;
}

int ParameterEstimator::buildErrorVecMarker3d(const KDL::Chain& chain, const vector<MarkerData3d>& input_data, NEWMAT::ColumnVector& vec,
                                              const JacNewmatBridge::JacTerms::JacTerms jac_terms)
{
  int result ;

  const unsigned int N = input_data.size() ;    // # of Data points

  KDL::ChainFkSolverPos_recursive fk_solver(chain) ;

  vector<KDL::Frame> markers_fk ;              // Stores the forward kinematics position of the marker.
  markers_fk.resize(N) ;

  // Compute the forward kinematics of the end effector
  for (unsigned int i=0; i<N; i++)
  {
    result = fk_solver.JntToCart(input_data[i].joint_states, markers_fk[i]) ;
    if (result < 0)
      return result-10 ;
  }

  //! \todo Currently ignoring all rotational forward-kinematics.
  switch (jac_terms)
  {
    case JacNewmatBridge::JacTerms::TRANS :
    {
      int num_rows = 3*N ;
      if (vec.Nrows() != num_rows)
        vec.ReSize(num_rows) ;

      // Compute each individual error vector, and then stack them in a 3Nx1 Newmat matrix
      for (unsigned int i=0; i<N; i++)
      {
        KDL::Vector cur_error = input_data[i].marker_sensed - markers_fk[i].p;
        vec(i*3+1) = cur_error.data[0] ;
        vec(i*3+2) = cur_error.data[1] ;
        vec(i*3+3) = cur_error.data[2] ;
      }
      break ;
    }
    case JacNewmatBridge::JacTerms::ROT :
      return -1 ;
    case JacNewmatBridge::JacTerms::ALL :
      return -2 ;
    default :
      return -3 ;
  }
  return 0 ;
}




