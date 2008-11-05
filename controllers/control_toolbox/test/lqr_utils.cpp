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
  * @brief Unit tests against basic control checks described in include/control_test.h
*/
#include <Eigen/Core>
#include <Eigen/Array> //For random, ones, etc
using Eigen::Dynamic;
#include <gtest/gtest.h>
#include <control_toolbox/control_test.h>

/** @brief test on random matrices
*/
template<typename Scalar, int N, int M>
void test_commandability1(int n, int m, int repeats=1)
{
  ASSERT_TRUE(N==n||N==Dynamic);
  ASSERT_TRUE(M==m||M==Dynamic);
  
  typedef typename Eigen::Matrix<Scalar,N,N> AMatrix;
  typedef typename Eigen::Matrix<Scalar,N,M> BMatrix;
  for(int i=0;i<repeats;++i)
  {
    const AMatrix & A1 = AMatrix::Random(n,n);
    const BMatrix & B1 = BMatrix::Random(n,m);
    
    EXPECT_TRUE(LQR::isCommandable(A1,B1));
  }
}

TEST(ControlUtils, Commandability)
{
  //Testing pathological cases
  test_commandability1<double,1,1>(1,1);
  test_commandability1<double,Dynamic,Dynamic>(1,1);
  
  //Dynamic matrices
  test_commandability1<double,Dynamic,Dynamic>(10,5,10);
  
  // This one fails:
//   test_commandability1<double,Dynamic,Dynamic>(30,30);
  test_commandability1<double,Dynamic,Dynamic>(20,20);
  
  //Static matrices
  test_commandability1<double,10,5>(10,5,10);
}

/** @brief convergence test on random matrices
*/
template<typename Scalar, int N>
void test_convergence1(int n, int repeats=1)
{
  ASSERT_TRUE(N==n||N==Dynamic);
  
  typedef typename Eigen::Matrix<Scalar,N,N> AMatrix;
  AMatrix A1(n,n);
  
  //This fails
  //fixme: SEND  BUG REPORT TO eigen-devel
/*  A1=AMatrix::Zero(n,n);
  ASSERT_FALSE(LQR::isConverging(A1));*/
  
  for(int i=0;i<repeats;++i)
  {
    A1 = AMatrix::Random(n,n);
    A1*=-A1.transpose();
    AMatrix A2=-A1;
    ASSERT_TRUE(LQR::isConverging(A1));
    ASSERT_FALSE(LQR::isConverging(A2));
  }
}

// This test fails for now due to problems in the Eigen library
TEST(ControlUtils, Convergence)
{
  //Testing pathological cases
  test_convergence1<double,1>(1,10);
  test_convergence1<double,Dynamic>(1,10);
  
  //Dynamic matrices
  test_convergence1<double,Dynamic>(10,10);
  
  // This one fails:
  test_convergence1<double,Dynamic>(30);
  
  //Static matrices
  test_convergence1<double,10>(10,10);  
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}