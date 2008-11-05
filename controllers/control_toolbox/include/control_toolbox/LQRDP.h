// This file is part of LeebA, a lightweight C++ template library
// for convex optimization and control.
//
// Copyright (C) 2008 Timothee Hunter <tjhunter@stanford.edu>
//
// LeebA is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// LeebA is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  for 
// more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with LeebA in the file COPYING.LESSER at the root of the 
// LeebA package. If not, see <http://www.gnu.org/licenses/>.

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <vector>
#include <boost/mpl/assert.hpp>

/** @author Timothy Hunter tjhunter@willowgarage.com
  * @brief LQR algorithmic solver for steady state control in discrete times
*/
namespace LQR
{

/** @brief templated structure around the lqr algorithm
  * @param AType Type of the state matrix
  * @param BType Type of the input matrix
  * @param QType
  * @param RType 
  */
template<typename AType, typename BType, typename QType, typename RType>
struct LQRDP
{
  enum
  {
    NumStates = AType::ColsAtCompileTime,
    NumCommands = BType::ColsAtCompileTime
  };
  
  // The scalar type used (double, float, complex, int if you really want)
  typedef typename AType::Scalar Scalar;
  
  // The type of the gains matrix
  typedef Eigen::Matrix<Scalar, NumCommands,NumStates> KType;
/*  // A return vector for the finite horizon
  typedef std::vector<KType> KVector;*/

  //NOTE: this code is still not very mature and can be optimized:
  //FIXME: move computation of K out of the loop
  //FIXME: use a better inverter (Cholesky decomposition)
  //FIXME: factorize inversion and various pieces that get transposed around
  //FIXME: documentation
  static void run(const AType & A,
    const BType & B,
    const QType & Q,
    const QType & Qf,
    const RType & R,
    Scalar precision,
    KType & K)
  {
    const int n=A.rows();
    QType P = Qf;
    Scalar error=Scalar(1000);
    int steps=0;
    while(error>precision*precision*n*n)
    {
      steps++;
      const QType & nextP = Q+A.transpose()*P*A
        -A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
      error = (P-nextP).lazy().cwise().square().sum();
      
      K = -(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
      P = nextP;
      
      
      // --- debug ---
//        Eigen::EigenSolver<AType> es(A+B*K);
//        std::cout<<"\nSTEP "<<steps<<"\nEigenValues(real part):\n"<<es.eigenvalues().real();
//        if(steps%10==1)
//          std::cout<<"\nERROR="<<error;

    }
//     std::cout<<"\nDone in "<<steps<<" steps\n";
  }
  
  // Continuous version
  static void runContinuous(const AType & A,
    const BType & B,
    const QType & Q,
    const QType & Qf,
    const RType & R,
    Scalar precision,
    Scalar h,
    KType & K)
  {
    const int n=A.rows();
    const AType & A_=AType::Identity(n,n)+h*A;
    const BType & B_=h*B;
    const QType & Q_=h*Q;
    const RType & R_=h*R;
    run(A_,B_,Q_,Qf,R_,precision,K);
  }

};

} //namespace