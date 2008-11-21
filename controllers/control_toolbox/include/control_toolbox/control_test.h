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

#ifndef CONTROL_TEST_H
#define CONTROL_TEST_H
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>

// template<typename AType, typename BType, typename CType>
// struct SystemTest
// {
//   enum
//   {
//     NumStates = AType::ColsAtCompileTime,
//     NumCommands = BType::ColsAtCompileTime,
//   };
//   
//   typedef typename AType::Scalar Scalar;
//   
//   // The gains matrix
//   typedef Eigen::Matrix<Scalar, NumCommands,NumStates> KType;
//   
//   static bool isCommandable(const AType & A, const BType & B)
//   {
//     typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ComMatrixType;
//     
//     ComMatrixType comMat(A.rows(), A.cols()*B.rows());
//     
//     comMat.block(0,0,A.rows(),B.cols())=A;
//     for(int i=1;i<A.rows();++i)
//       comMat.block(0,i*B.cols(),A.rows(),B.cols())=comMat.block(0,(i-1)*B.cols(),A.rows(),B.cols())*B;
// //     QR decomposition of the commandability matrix
//     Eigen::LU<ComMatrixType> lu(comMat);
//     return lu.rank()==A.rows();
//   }
//   
//   static bool isObservable(const AType & A, const CType & C)
//   {
//     return isCommandable(A.transpose(),C.transpose());
//   }
//   
//   static bool isConverging(const AType & A, Scalar epsi=Scalar(1e-4))
//   {
//     Eigen::EigenSolver<AType> es(A);
//     return es.eigenvalues().real().maxCoeff() < -epsi;
//   }
//   
//   static bool isConverging(const AType & A, const BType & B, const KType & K, Scalar epsi=1e-4)
//   {
//     return isConverging(A-B*K, epsi);
//   }
// };

namespace LQR
{

template<typename Derived1, typename Derived2>
static bool isCommandable(const Eigen::MatrixBase<Derived1> & A, const Eigen::MatrixBase<Derived2> & B)
{
  assert(A.cols()==A.rows());
  assert(A.cols()==B.rows());
  
  const int n=A.rows();
  const int m=B.cols();
  
  typedef Eigen::Matrix<typename Derived1::Scalar, Eigen::Dynamic, Eigen::Dynamic> ComMatrixType;
  
  ComMatrixType comMat(n, n*m);
  
  comMat.block(0,0,n,m)=B;
  for(int i=1;i<n;++i)
    comMat.block(0,i*m,n,m)=A*comMat.block(0,(i-1)*m,n,m);
  // LU decomposition of the commandability matrix
  Eigen::LU<ComMatrixType> lu(comMat);
  return lu.rank()==n;
}

template<typename Derived>
static bool isConverging(const Eigen::MatrixBase<Derived> & A, double epsi=1e-4)
{
  Eigen::EigenSolver<Derived> es(A);
//   Eigen::IOFormat fmt(4, Eigen::AlignCols, ", ", ";\n", "", "", "[", "]");
//   std::cout<<std::endl<<A.format(fmt)<<std::endl<<std::endl<<es.eigenvalues()<<std::endl;
  return es.eigenvalues().real().maxCoeff() < static_cast<typename Derived::Scalar>(-epsi);
}

template<typename DerivedA, typename DerivedB, typename DerivedG>
static bool isConverging(const Eigen::MatrixBase<DerivedA> & A, const Eigen::MatrixBase<DerivedB> & B, const Eigen::MatrixBase<DerivedG> & K, double epsi=1e-4)
{
//   Eigen::IOFormat fmt(4, Eigen::AlignCols, ", ", ";\n", "", "", "[", "]");
//   std::cout<<std::endl<<A.format(fmt)<<std::endl<<std::endl<<std::endl;
//   std::cout<<std::endl<<B.format(fmt)<<std::endl<<std::endl<<std::endl;
//   std::cout<<std::endl<<K.format(fmt)<<std::endl<<std::endl<<std::endl;
  //Need to create a temporary here:
  const DerivedA & A1=A+B*K;
  return isConverging(A1, epsi);
}

template<typename Derived>
static bool isConvergingDiscrete(const Eigen::MatrixBase<Derived> & A, double epsi=1e-4)
{
  Eigen::EigenSolver<Derived> es(A);
//   Eigen::IOFormat fmt(4, Eigen::AlignCols, ", ", ";\n", "", "", "[", "]");
//   std::cout<<std::endl<<A.format(fmt)<<std::endl<<std::endl<<es.eigenvalues()<<std::endl;
  return (es.eigenvalues().cwise()*es.eigenvalues().conjugate()).real().maxCoeff() < static_cast<typename Derived::Scalar>(1-epsi);
}

template<typename DerivedA, typename DerivedB, typename DerivedG>
static bool isConvergingDiscrete(const Eigen::MatrixBase<DerivedA> & A, const Eigen::MatrixBase<DerivedB> & B, const Eigen::MatrixBase<DerivedG> & K, double epsi=1e-4)
{
/*  Eigen::IOFormat fmt(4, Eigen::AlignCols, ", ", ";\n", "", "", "[", "]");
  std::cout<<std::endl<<A.format(fmt)<<std::endl<<std::endl<<std::endl;
  std::cout<<std::endl<<B.format(fmt)<<std::endl<<std::endl<<std::endl;
  std::cout<<std::endl<<K.format(fmt)<<std::endl<<std::endl<<std::endl;*/
  //Need to create a temporary here:
  const DerivedA & A1=A+B*K;
  return isConvergingDiscrete(A1, epsi);
}

};

#endif
