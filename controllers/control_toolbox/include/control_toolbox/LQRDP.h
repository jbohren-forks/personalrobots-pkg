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

namespace LQR
{

template<typename AType, typename BType, typename QType, typename RType>
struct LQRDP
{
  enum
  {
    NumStates = AType::ColsAtCompileTime,
    NumCommands = BType::ColsAtCompileTime
  };
  
  typedef typename AType::Scalar Scalar;
  
  typedef Eigen::Matrix<Scalar, NumCommands,NumStates> KType;
  typedef std::vector<KType> KVector;
  
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
      Eigen::EigenSolver<AType> es(A+B*K);
      std::cout<<"\nSTEP "<<steps<<"\nEV\n"<<es.eigenvalues().real();
      if(steps%10==1)
        std::cout<<"\nERROR="<<error;

    }
    std::cout<<"Done in "<<steps<<" steps\n";
  }
  
};

}