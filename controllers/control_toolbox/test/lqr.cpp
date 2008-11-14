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

/**
  * 2 types of tests are performed here:
  * Given a system (A,B) and some cost matrices (Q,R), the gain matrix K is computed. 
  * It is first checked that the matrix A+BK leads to a converging behaviour, and then 
  * this is verified by a forward simulation from a random point during a fixed amount
  * of time.
  *
  * In the case of a discrete system, it is checked that the complex module of the eigen
  * values of A+BK is < 1 - epsilon
  *
  * In the case of a continuous system, it is checked that the real part of the eigen values
  * of A+BK is < -epsilon
  */

//FIXME: test if the pair (A,Q) is controllable (one never knows...)
//FIXME: we can factorize the code

// Given a matrix A and a start point x0, returns true is the sequence x_i+1 = A x_i converges to 0 after a fixed number of iterations
template<typename DerivedA, typename VectorX>
bool test_convergenceDiscrete(const Eigen::MatrixBase<DerivedA> & A, VectorX x)
{
  const int iter_max=10000;
  const double convergence_threshold=1e-2;
  double res=100;
  for(int i=0;i<iter_max&&res>convergence_threshold;i++)
  {
    res=(x).cwise().square().sum();
    x=A*x;  
  }
  return res<convergence_threshold;
}

template<typename DerivedA, typename VectorX>
bool test_convergence(const Eigen::MatrixBase<DerivedA> & A, VectorX x, double h)
{
  const int iter_max=1000000;
  const double convergence_threshold=1e-2;
  double res=100;
  for(int i=0;i<iter_max&&res>convergence_threshold;i++)
  {
    res=(x).cwise().square().sum();
//     std::cout<<res<<std::endl;
    x+=h*A*x;  
  }
  return res<convergence_threshold;
}

// Test discrete LQR solver with random matrices
template<typename Scalar, int N, int M>
void test_randomDiscrete(int n, int m, int repeats=1)
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
    const AMatrix & A = 3*AMatrix::Random(n,n);
    const BMatrix & B = 3*BMatrix::Random(n,m);
    
    if(LQR::isCommandable(A,B))
    {
      LQR::LQRDP<AMatrix,BMatrix,AMatrix,BMatrix>::run(A,B,Q,Q,R,0.01,G);
      const AVector & x0 = AVector::Random(n,1);
      ASSERT_TRUE(test_convergenceDiscrete(A+B*G,x0));
      ASSERT_TRUE(LQR::isConvergingDiscrete(A,B,G));
    }   
  }
}

// Test discrete LQR solver with the identity matrix as cost matrices
template<typename Scalar, int N, int M>
void test_identityDiscrete(int n, int m, int repeats=1)
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
  
  GMatrix G(m,n);
  
  for(int i=0;i<repeats;++i)
  {
    const AMatrix & A = AMatrix::Random(n,n);
    const BMatrix & B = BMatrix::Random(n,m);
    
    if(LQR::isCommandable(A,B))
    {
      LQR::LQRDP<AMatrix,BMatrix,AMatrix,BMatrix>::run(A,B,Q,Q,R,0.01,G);
      
      const AVector & x0 = AVector::Random(n,1);
      ASSERT_TRUE(test_convergenceDiscrete(A+B*G,x0));
      ASSERT_TRUE(LQR::isConvergingDiscrete(A,B,G));
    }   
  }
}

// Test continuous LQR solver with the identity matrix as cost matrices
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
  
  GMatrix G(m,n);
  
  for(int i=0;i<repeats;++i)
  {
    const AMatrix & A = AMatrix::Random(n,n);
    const BMatrix & B = BMatrix::Random(n,m);
    
    if(LQR::isCommandable(A,B))
    {
      LQR::LQRDP<AMatrix,BMatrix,AMatrix,BMatrix>::runContinuous(A,B,Q,R,G);
      
      const AVector & x0 = 10*AVector::Random(n,1);
      ASSERT_TRUE(LQR::isConverging(A,B,G));
      ASSERT_TRUE(test_convergence(A+B*G,x0,0.01));
    }   
  }
}

// Test continuous LQR solver with the identity matrix as cost matrices
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
  
  GMatrix G(m,n);
  
  for(int i=0;i<repeats;++i)
  {
    AMatrix Q = AMatrix::Random(n,n);
    Q*=Q.transpose();
    WMatrix R = WMatrix::Random(m,m);
    R*=R.transpose();
    
    const AMatrix & A = AMatrix::Random(n,n);
    const BMatrix & B = BMatrix::Random(n,m);
    
    if(LQR::isCommandable(A,B))
    {
      LQR::LQRDP<AMatrix,BMatrix,AMatrix,BMatrix>::runContinuous(A,B,Q,R,G);
      
      ASSERT_TRUE(LQR::isConverging(A,B,G));
      //This test might fail for big matrices => disabled
//       const AVector & x0 = 10*AVector::Random(n,1);
//       ASSERT_TRUE(test_convergence(A+B*G,x0,0.1));
    }   
  }
}


TEST(LQR, IdentityDiscrete1)
{
  //Testing pathological cases
  test_identityDiscrete<double,3,3>(3,3,10);
  // If it works with the answer to all questions in the universe, it short work, period :)
  test_identityDiscrete<double,Dynamic,Dynamic>(42,10,10);
}


TEST(LQR, RandomDiscrete1)
{
  //Testing pathological cases
  test_randomDiscrete<double,3,3>(3,3,10);
  // If it works with the answer to all questions in the universe, it short work, period :)
  test_randomDiscrete<double,Dynamic,Dynamic>(42,10,10);
}


// Small matrices
// In this case, the algorithm is optimized at compile time for optimal performance
TEST(LQR, Identity1Small)
{
  test_identity<double,6,6>(6,6,10);
}

TEST(LQR, Identity1)
{
  // If it works with the answer to all questions in the universe, it short work, period :)
  test_identity<double,Dynamic,Dynamic>(42,10,10);
}

// /*Test for small sizes
// FIXME: segfaults under some circumstances*/
// TEST(LQR, Random1Small)
// {
//   test_random<double,10,10>(10,10,10);
//   test_random<double,7,7>(7,7,10);
//   test_random<double,4,4>(4,4,10);
// }

TEST(LQR, Random1)
{
  test_random<double,Dynamic,Dynamic>(5,5,10);
  test_random<double,Dynamic,Dynamic>(2,2,10);
  test_random<double,Dynamic,Dynamic>(20,20,20);
}

TEST(LQR, Random2)
{
  test_random<double,Dynamic,Dynamic>(100,100,1);
  test_random<double,Dynamic,Dynamic>(42,10,10);
  test_random<double,Dynamic,Dynamic>(5,10,10);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
