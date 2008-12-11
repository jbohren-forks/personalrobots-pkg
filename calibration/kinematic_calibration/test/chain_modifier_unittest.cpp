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

#include <gtest/gtest.h>

#include <string>

#include "kinematic_calibration/chain_modifier.h"
#include "kinematic_calibration/active_link_params.h"
#include "kinematic_calibration/verify_jacobian.h"

using namespace kinematic_calibration ;
using namespace std ;

static const double epsilon = 1e-6 ;

namespace CALC_MODE
{
  enum CALC_MODE {MAT, ACTIVE_ONLY} ;
}

class ChainModifierTester
{
public:
  static void ChainTest(const std::string path, unsigned int num_links, CALC_MODE::CALC_MODE calc_mode )
  {
    int result ;

    // Load all the data files
    ActiveLinkParams active ;
    result = active.loadFromFile(path+"/active_params.txt", num_links) ;
    EXPECT_EQ(result, 0) ;

    ModelGetter model_getter ;
    KDL::Chain chain_before, chain_after ;
    result = model_getter.OpenFile(path + "/model_before.txt") ;
    EXPECT_EQ(result, 0) ;
    chain_before = model_getter.GetModel() ;
    EXPECT_EQ(chain_before.getNrOfSegments(), num_links) ;
    model_getter.CloseFile() ;

    result = model_getter.OpenFile(path + "/model_after.txt") ;
    EXPECT_EQ(result, 0) ;
    chain_after = model_getter.GetModel() ;
    EXPECT_EQ(chain_after.getNrOfSegments(), num_links) ;
    model_getter.CloseFile() ;

    vector<double> corrections ;
    result = LoadCorrectionParams(path + "/corrections.txt", corrections, num_links) ;
    EXPECT_EQ(result, 0) ;

    NEWMAT::Matrix all_corrections ;
    result = LoadAllCorrectionParams(path + "/all_corrections.txt", all_corrections, num_links) ;
    EXPECT_EQ(result, 0) ;

    // Perform the correction
    KDL::Chain chain_corrected(chain_before) ;
    ChainModifier chain_modifier ;
    switch (calc_mode)
    {
      case CALC_MODE::MAT:
      {
        chain_modifier.specifyAllParams(all_corrections) ;
        break ;
      }
      case CALC_MODE::ACTIVE_ONLY:
      {
        chain_modifier.specifyActiveParams(corrections, active) ;
        break ;
      }
    }
    EXPECT_EQ(chain_modifier.getNumLinks(), num_links) ;

    result = chain_modifier.modifyChain(chain_corrected) ;
    EXPECT_EQ(result, 0) ;

    double chain_error = 100000.0 ;
    result = ComputeChainError(chain_after, chain_corrected, chain_error) ;
    EXPECT_EQ(result, 0) ;

    EXPECT_NEAR(chain_error, 0.00, epsilon) ;
  }

  static void SparseTest(const std::string path, unsigned int num_links)
  {
    int result ;
    ActiveLinkParams active ;
    result = active.loadFromFile(path+"/active_params.txt", num_links) ;
    EXPECT_EQ(result, 0) ;

    vector<double> corrections ;
    result = LoadCorrectionParams(path + "/corrections.txt", corrections, num_links) ;
    EXPECT_EQ(result, 0) ;

    NEWMAT::Matrix all_corrections ;
    result = LoadAllCorrectionParams(path + "/all_corrections.txt", all_corrections, num_links) ;
    EXPECT_EQ(result, 0) ;

    NEWMAT::Matrix sparse_corrections ;
    result = ChainModifier::buildParamMatrix(sparse_corrections, corrections, active) ;
    EXPECT_EQ(result, 0) ;

    EXPECT_EQ(sparse_corrections.Ncols(), all_corrections.Ncols()) ;
    EXPECT_EQ(sparse_corrections.Nrows(), all_corrections.Nrows()) ;

    double total_error = 0.0 ;
    for (int i=1; i<=all_corrections.Nrows(); i++)
    {
      for (int j=1; j<=all_corrections.Ncols(); j++)
      {
        double cur_error = all_corrections(i,j) - sparse_corrections(i,j) ;
        total_error += abs(cur_error) ;
      }
    }
    EXPECT_NEAR(total_error, 0.00, epsilon) ;
  }

  static int LoadAllCorrectionParams(const std::string& filename, NEWMAT::Matrix& all_params, int num_links)
  {
    FILE* infile ;
    infile = fopen(filename.c_str(), "r") ;

    if(infile == NULL)
      return -1 ;

    all_params.ReSize(6,num_links) ;

    for (int i=1; i<=6; i++)
    {
      for (int j=1; j<=num_links; j++)
      {
        double cur_param ;
        int result = fscanf(infile, "%lf", &cur_param) ;
        if (result != 1)
          return -2 ;
        all_params(i,j) = cur_param ;
      }
    }
    return 0 ;
  }

  static int LoadCorrectionParams(const std::string& filename, std::vector<double>& params, int num_links)
  {
    FILE* infile ;
    infile = fopen(filename.c_str(), "r") ;

    if(infile == NULL)
      return -1 ;

    params.clear() ;

    int result ;
    double cur_param ;
    result = fscanf(infile, "%lf", &cur_param) ;
    while (result == 1)
    {
      params.push_back(cur_param) ;
      result = fscanf(infile, "%lf", &cur_param) ;
    }

    return 0 ;
  }

  static int ComputeChainError(const KDL::Chain& chain1, const KDL::Chain& chain2, double& max_error)
  {
    max_error = 0.00 ;

    if (chain1.getNrOfSegments() != chain2.getNrOfSegments())
      return -1 ;

    for (unsigned int i=0; i<chain1.getNrOfSegments(); i++)
    {
      int result ;
      double cur_seg_error ;

      result = ComputeFrameError(chain1.getSegment(i).getFrameToTip(),  chain2.getSegment(i).getFrameToTip(), cur_seg_error) ;

      if (result < 0)
        return result ;

      if (cur_seg_error > max_error)
        max_error = cur_seg_error ;
    }
    return 0 ;
  }

  static int ComputeFrameError(const KDL::Frame& frame1, const KDL::Frame& frame2, double& error)
  {
    double frame_error = 0.0 ;
    for (int i=0; i<3; i++)
    {
      frame_error += abs( frame1.p.data[i] - frame2.p.data[i] ) ;
    }

    for (int i=0; i<9; i++)
    {
      frame_error += abs( frame1.M.data[i] - frame2.M.data[i] ) ;
    }

    error = frame_error ;

    return 0 ;
  }
} ;

TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_easy1_trans_mat)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/easy1_trans"), 1, CALC_MODE::MAT) ;
}
TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_easy1_trans_active)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/easy1_trans"), 1, CALC_MODE::ACTIVE_ONLY) ;
}
TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_easy1_trans_sparse_test)
{
  ChainModifierTester::SparseTest(string("./test/data/chain_modifier_unittest/easy1_trans"), 1) ;
}


TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_easy1_rot_mat)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/easy1_rot"), 1, CALC_MODE::MAT) ;
}
TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_easy1_rot_active)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/easy1_rot"), 1, CALC_MODE::ACTIVE_ONLY) ;
}


TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_easy2_trans_mat)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/easy2_trans"), 2, CALC_MODE::MAT) ;
}
TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_easy2_trans_active)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/easy2_trans"), 2, CALC_MODE::ACTIVE_ONLY) ;
}
TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_easy2_trans_sparse_test)
{
  ChainModifierTester::SparseTest(string("./test/data/chain_modifier_unittest/easy2_trans"), 2) ;
}


TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_med1_rot_mat)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/med1_rot"), 1, CALC_MODE::MAT) ;
}
TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_med1_rot_active)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/med1_rot"), 1, CALC_MODE::ACTIVE_ONLY) ;
}


TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_med2_all_mat)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/med2_all"), 2, CALC_MODE::MAT) ;
}
TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_med2_all_active)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/med2_all"), 2, CALC_MODE::ACTIVE_ONLY) ;
}

TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_hard1B_all_mat)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/hard1B_all"), 1, CALC_MODE::MAT) ;
}
TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_hard1B_all_active)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/hard1B_all"), 1, CALC_MODE::ACTIVE_ONLY) ;
}


TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_hard1_rot_mat)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/hard1_rot"), 1, CALC_MODE::MAT) ;
}
TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_hard1_rot_active)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/hard1_rot"), 1, CALC_MODE::ACTIVE_ONLY) ;
}

TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_hard10_all_mat)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/hard10_all"), 10, CALC_MODE::MAT) ;
}
TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_hard10_all_active)
{
  ChainModifierTester::ChainTest(string("./test/data/chain_modifier_unittest/hard10_all"), 10, CALC_MODE::ACTIVE_ONLY) ;
}
TEST(KINEMATIC_CALIBRATION_CHAIN_MODIFIER, chain_modifier_hard10_all_sparse_test)
{
  ChainModifierTester::SparseTest(string("./test/data/chain_modifier_unittest/hard10_all"), 10) ;
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
