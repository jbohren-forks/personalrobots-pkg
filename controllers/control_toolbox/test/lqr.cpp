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

/** @author Timothy Hunter <tjhunter@willowgarage.com>
  * @brief Unit tests against the LQR algorithm in LQRDP.h
*/

#include <Eigen/Core>
#include <Eigen/Array> //For random, ones, etc
using Eigen::Dynamic;

#include <gtest/gtest.h>
#include <control_toolbox/control_test.h>
#include <control_toolbox/LQRDP.h>

template<typename DerivedA, typename VectorX>
bool test_convergence(const Eigen::MatrixBase<DerivedA> & A, VectorX x)
{
  double res=100;
  for(int i=0;i<100;i++)
  {
    res=(x).cwise().square().sum();
//     std::cout<<res<<std::endl;
    x=A*x;  
  }
  return res<1e-5;
}

template<typename Scalar, int N, int M>
void test_random(int n, int m, int repeats=1)
{
  ASSERT_TRUE(N==n||N==Dynamic);
  ASSERT_TRUE(M==m||M==Dynamic);
  
  typedef typename Eigen::Matrix<Scalar,N,N> AMatrix;
  typedef typename Eigen::Matrix<Scalar,N,1> AVector;
  typedef typename Eigen::Matrix<Scalar,N,M> BMatrix;
  typedef typename Eigen::Matrix<Scalar,M,N> GMatrix;
  typedef typename Eigen::Matrix<Scalar,M,M> WMatrix;
  
  AMatrix Q = AMatrix::Random(n,n);
  Q*=Q.transpose();
  WMatrix R = WMatrix::Random(m,m);
  R*=R.transpose();
  
  GMatrix G(m,n);
  
  for(int i=0;i<repeats;++i)
  {
    const AMatrix & A = AMatrix::Random(n,n);
    const BMatrix & B = BMatrix::Random(n,m);
    
    if(LQR::isCommandable(A,B))
    {
      LQR::LQRDP<AMatrix,BMatrix,AMatrix,BMatrix>::run(A,B,Q,Q,R,0.01,G);
      const AVector & x0 = AVector::Random(n,1);
      ASSERT_TRUE(test_convergence(A+B*G,x0));
      // The Eigen solver seems to be buggy => to check
//       ASSERT_TRUE(LQR::isConverging(A,B,G));
    }   
  }
}

template<typename Scalar, int N, int M>
void test_identity(int n, int m, int repeats=1)
{
  ASSERT_TRUE(N==n||N==Dynamic);
  ASSERT_TRUE(M==m||M==Dynamic);
  
  typedef typename Eigen::Matrix<Scalar,N,N> AMatrix;
  typedef typename Eigen::Matrix<Scalar,N,1> AVector;
  typedef typename Eigen::Matrix<Scalar,N,M> BMatrix;
  typedef typename Eigen::Matrix<Scalar,M,N> GMatrix;
  typedef typename Eigen::Matrix<Scalar,M,M> WMatrix;
  
  AMatrix Q = AMatrix::Identity(n,n);
  WMatrix R = WMatrix::Identity(m,m);
  R*=R.transpose();
  
  GMatrix G(m,n);
  
  for(int i=0;i<repeats;++i)
  {
    const AMatrix & A = AMatrix::Random(n,n);
    const BMatrix & B = BMatrix::Random(n,m);
    
    if(LQR::isCommandable(A,B))
    {
      LQR::LQRDP<AMatrix,BMatrix,AMatrix,BMatrix>::run(A,B,Q,Q,R,0.01,G);
      
      const AVector & x0 = AVector::Random(n,1);
      ASSERT_TRUE(test_convergence(A+B*G,x0));
      // The Eigen solver seems to be buggy => to check
//       ASSERT_TRUE(LQR::isConverging(A,B,G));
    }   
  }
}


TEST(LQR, Identity1)
{
  //Testing pathological cases
//   test_random<double,3,3>(3,3,10);
  test_identity<double,3,3>(3,3,10);
  // If it works with the answer to all questions in the universe, it short work, period :)
  test_identity<double,Dynamic,Dynamic>(42,10,10);
}


TEST(LQR, Random1)
{
  //Testing pathological cases
  test_random<double,3,3>(3,3,10);
  // If it works with the answer to all questions in the universe, it short work, period :)
  test_random<double,Dynamic,Dynamic>(42,10,10);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
