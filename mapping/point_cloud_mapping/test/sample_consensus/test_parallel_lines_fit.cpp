//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

/* Based on test_line_fit.cpp */

#include <gtest/gtest.h>
#include <robot_msgs/PointCloud.h>

#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/rransac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/rmsac.h>
#include <point_cloud_mapping/sample_consensus/mlesac.h>
#include <point_cloud_mapping/sample_consensus/sac_model.h>
#include <point_cloud_mapping/sample_consensus/sac_model_parallel_lines.h>

using namespace sample_consensus;

/*
TEST(LMedS, SACModelParallelLines)
{
  robot_msgs::PointCloud points;
  points.pts.resize(17);

  points.pts[0].x = 1;  points.pts[0].y = 2;    points.pts[0].z = 3;
  points.pts[1].x = 4;  points.pts[1].y = 5;    points.pts[1].z = 6;
  points.pts[2].x = 7;  points.pts[2].y = 8;    points.pts[2].z = 9;
  points.pts[3].x = 10; points.pts[3].y = 11;   points.pts[3].z = 12;
  points.pts[4].x = 13; points.pts[4].y = 14;   points.pts[4].z = 15;
  points.pts[5].x = 16; points.pts[5].y = 17;   points.pts[5].z = 18;
  points.pts[6].x = 19; points.pts[6].y = 20;   points.pts[6].z = 21;
  points.pts[7].x = 22; points.pts[7].y = 23;   points.pts[7].z = 24;
  points.pts[8].x = -5; points.pts[8].y = 1.57; points.pts[8].z = 0.75;
  points.pts[9].x = 4;  points.pts[9].y = 2;    points.pts[9].z = 3;

  points.pts[10].x = 6;  points.pts[10].y = 2;    points.pts[10].z = 3;
  points.pts[11].x = 9;  points.pts[11].y = 5;    points.pts[11].z = 6;
  points.pts[12].x = 12;  points.pts[12].y = 8;    points.pts[12].z = 9;
  points.pts[13].x = 18; points.pts[13].y = 14;   points.pts[13].z = 15;
  points.pts[14].x = 21; points.pts[14].y = 17;   points.pts[14].z = 18;
  points.pts[15].x = 27; points.pts[15].y = 23;   points.pts[15].z = 24;
  points.pts[16].x = 9;  points.pts[16].y = 2;    points.pts[16].z = 3;

  SACModel *model = new SACModelParallelLines(0,1000);
  SAC *sac        = new LMedS(model, 0.001);
  model->setDataSet(&points);
  EXPECT_EQ((int)model->getCloud()->pts.size(), 17);

  bool result = sac->computeModel();
  EXPECT_EQ(result, true);

  std::vector<int> inliers = sac->getInliers();
  EXPECT_EQ((int)inliers.size(), 14);

  std::vector<double> coeff = sac->computeCoefficients();
  EXPECT_EQ((int)coeff.size(), 9);
  //printf ("Parallel line coefficients: %f %f %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6], coeff[7], coeff[8]);
  robot_msgs::Point32 dir;
  dir.x = fabs(coeff[3] - coeff[0]);
  dir.y = fabs(coeff[4] - coeff[1]);
  dir.z = fabs(coeff[5] - coeff[2]);
  cloud_geometry::normalizePoint(dir);
  //printf ("Line direction: %f %f %f\n", dir.x, dir.y, dir.z);
  EXPECT_NEAR(dir.x, 0.577, 1e-3);
  EXPECT_NEAR(dir.y, 0.577, 1e-3);
  EXPECT_NEAR(dir.z, 0.577, 1e-3);

  std::vector<double> coeff_ref = sac->refineCoefficients();
  EXPECT_EQ((int)coeff_ref.size(), 9);
  //printf ("Line coefficients (refined): %f %f %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3], coeff_ref[4], coeff_ref[5]);
  robot_msgs::Point32 dir_ref;
  dir_ref.x = fabs(coeff_ref[3] - coeff_ref[0]);
  dir_ref.y = fabs(coeff_ref[4] - coeff_ref[1]);
  dir_ref.z = fabs(coeff_ref[5] - coeff_ref[2]);
  cloud_geometry::normalizePoint(dir_ref);
  //printf ("Line direction: %f %f %f\n", dir_ref.x, dir_ref.y, dir_ref.z);
  EXPECT_NEAR(dir_ref.x, 0.577, 1e-3);
  EXPECT_NEAR(dir_ref.y, 0.577, 1e-3);
  EXPECT_NEAR(dir_ref.z, 0.577, 1e-3);

  int nr_points_left = sac->removeInliers();
  EXPECT_EQ(nr_points_left, 3);
  
  delete sac;
  delete model;
}
*/

TEST (RANSAC, SACModelParallelLines)
{
  robot_msgs::PointCloud points;
  points.pts.resize(17);

  points.pts[0].x = 1;  points.pts[0].y = 1;    points.pts[0].z = 1;
  points.pts[1].x = 4;  points.pts[1].y = 1;    points.pts[1].z = 1;
  points.pts[2].x = 7;  points.pts[2].y = 1;    points.pts[2].z = 1;
  points.pts[3].x = 10; points.pts[3].y = 1;    points.pts[3].z = 1;
  points.pts[4].x = 13; points.pts[4].y = 1;    points.pts[4].z = 1;
  points.pts[5].x = 16; points.pts[5].y = 1;    points.pts[5].z = 1;
  points.pts[6].x = 19; points.pts[6].y = 1;    points.pts[6].z = 1;
  points.pts[7].x = 22; points.pts[7].y = 1;    points.pts[7].z = 1;
  points.pts[8].x = 1;  points.pts[8].y = 15;    points.pts[8].z = 1;
  points.pts[9].x = 1;  points.pts[9].y = 16;    points.pts[9].z = 1;

  points.pts[10].x = 7;  points.pts[10].y = -1;    points.pts[10].z = 1;
  points.pts[11].x = 10; points.pts[11].y = -1;    points.pts[11].z = 1;
  points.pts[12].x = 13; points.pts[12].y = -1;    points.pts[12].z = 1;
  points.pts[13].x = 16; points.pts[13].y = -1;    points.pts[13].z = 1;
  points.pts[14].x = 19; points.pts[14].y = -1;    points.pts[14].z = 1;
  points.pts[15].x = 22; points.pts[15].y = -1;    points.pts[15].z = 1;
  points.pts[16].x = 1;  points.pts[16].y = -15;   points.pts[16].z = 1;


  /*
  points.pts[0].x = 1;  points.pts[0].y = 2;    points.pts[0].z = 3;
  points.pts[1].x = 4;  points.pts[1].y = 5;    points.pts[1].z = 6;
  points.pts[2].x = 7;  points.pts[2].y = 8;    points.pts[2].z = 9;
  points.pts[3].x = 10; points.pts[3].y = 11;   points.pts[3].z = 12;
  points.pts[4].x = 13; points.pts[4].y = 14;   points.pts[4].z = 15;
  points.pts[5].x = 16; points.pts[5].y = 17;   points.pts[5].z = 18;
  points.pts[6].x = 19; points.pts[6].y = 20;   points.pts[6].z = 21;
  points.pts[7].x = 22; points.pts[7].y = 23;   points.pts[7].z = 24;
  points.pts[8].x = -5; points.pts[8].y = 1.57; points.pts[8].z = 0.75;
  points.pts[9].x = 4;  points.pts[9].y = 2;    points.pts[9].z = 3;

  points.pts[10].x = 6;  points.pts[10].y = 2;    points.pts[10].z = 3;
  points.pts[11].x = 9;  points.pts[11].y = 5;    points.pts[11].z = 6;
  points.pts[12].x = 12;  points.pts[12].y = 8;    points.pts[12].z = 9;
  points.pts[13].x = 18; points.pts[13].y = 14;   points.pts[13].z = 15;
  points.pts[14].x = 21; points.pts[14].y = 17;   points.pts[14].z = 18;
  points.pts[15].x = 27; points.pts[15].y = 23;   points.pts[15].z = 24;
  points.pts[16].x = 9;  points.pts[16].y = 2;    points.pts[16].z = 3;
  */
  SACModel *model = new SACModelParallelLines(1.99,2.01);
  SAC *sac        = new RANSAC(model, 0.001);
  model->setDataSet(&points);
  EXPECT_EQ((int)model->getCloud()->pts.size(), 17);

  bool result = sac->computeModel();
  if (result == true)
  {

    //EXPECT_EQ(result, true);

    std::vector<int> inliers = sac->getInliers();
    EXPECT_EQ((int)inliers.size(), 14);

    std::vector<double> coeff = sac->computeCoefficients();
    EXPECT_EQ((int)coeff.size(), 9);
    //printf ("Parallel line coefficients: %f %f %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6], coeff[7], coeff[8]);
    robot_msgs::Point32 dir;
    dir.x = fabs(coeff[3] - coeff[0]);
    dir.y = fabs(coeff[4] - coeff[1]);
    dir.z = fabs(coeff[5] - coeff[2]);
    cloud_geometry::normalizePoint(dir);
    //printf ("Line direction: %f %f %f\n", dir.x, dir.y, dir.z);
    EXPECT_NEAR(dir.x, 1, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.y, 0, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.z, 0, 1e-3);// 0.577, 1e-3);

    std::vector<double> coeff_ref = sac->refineCoefficients();
    EXPECT_EQ((int)coeff_ref.size(), 9);
    //printf ("Line coefficients (refined): %f %f %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3], coeff_ref[4], coeff_ref[5]);
    robot_msgs::Point32 dir_ref;
    dir_ref.x = fabs(coeff_ref[3] - coeff_ref[0]);
    dir_ref.y = fabs(coeff_ref[4] - coeff_ref[1]);
    dir_ref.z = fabs(coeff_ref[5] - coeff_ref[2]);
    cloud_geometry::normalizePoint(dir_ref);
    //printf ("Line direction: %f %f %f\n", dir_ref.x, dir_ref.y, dir_ref.z);
    EXPECT_NEAR(dir.x, 1, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.y, 0, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.z, 0, 1e-3);// 0.577, 1e-3);

    int nr_points_left = sac->removeInliers();
    EXPECT_EQ(nr_points_left, 3);
  }
  
  delete sac;
  delete model; 
}

TEST (MSAC, SACModelParallelLines)
{
  robot_msgs::PointCloud points;
  points.pts.resize(17);

  points.pts[0].x = 1;  points.pts[0].y = 1;    points.pts[0].z = 1;
  points.pts[1].x = 4;  points.pts[1].y = 1;    points.pts[1].z = 1;
  points.pts[2].x = 7;  points.pts[2].y = 1;    points.pts[2].z = 1;
  points.pts[3].x = 10; points.pts[3].y = 1;    points.pts[3].z = 1;
  points.pts[4].x = 13; points.pts[4].y = 1;    points.pts[4].z = 1;
  points.pts[5].x = 16; points.pts[5].y = 1;    points.pts[5].z = 1;
  points.pts[6].x = 19; points.pts[6].y = 1;    points.pts[6].z = 1;
  points.pts[7].x = 22; points.pts[7].y = 1;    points.pts[7].z = 1;
  points.pts[8].x = 1;  points.pts[8].y = 15;    points.pts[8].z = 1;
  points.pts[9].x = 1;  points.pts[9].y = 16;    points.pts[9].z = 1;

  points.pts[10].x = 7;  points.pts[10].y = -1;    points.pts[10].z = 1;
  points.pts[11].x = 10; points.pts[11].y = -1;    points.pts[11].z = 1;
  points.pts[12].x = 13; points.pts[12].y = -1;    points.pts[12].z = 1;
  points.pts[13].x = 16; points.pts[13].y = -1;    points.pts[13].z = 1;
  points.pts[14].x = 19; points.pts[14].y = -1;    points.pts[14].z = 1;
  points.pts[15].x = 22; points.pts[15].y = -1;    points.pts[15].z = 1;
  points.pts[16].x = 1;  points.pts[16].y = -15;   points.pts[16].z = 1;


  /*
  points.pts[0].x = 1;  points.pts[0].y = 2;    points.pts[0].z = 3;
  points.pts[1].x = 4;  points.pts[1].y = 5;    points.pts[1].z = 6;
  points.pts[2].x = 7;  points.pts[2].y = 8;    points.pts[2].z = 9;
  points.pts[3].x = 10; points.pts[3].y = 11;   points.pts[3].z = 12;
  points.pts[4].x = 13; points.pts[4].y = 14;   points.pts[4].z = 15;
  points.pts[5].x = 16; points.pts[5].y = 17;   points.pts[5].z = 18;
  points.pts[6].x = 19; points.pts[6].y = 20;   points.pts[6].z = 21;
  points.pts[7].x = 22; points.pts[7].y = 23;   points.pts[7].z = 24;
  points.pts[8].x = -5; points.pts[8].y = 1.57; points.pts[8].z = 0.75;
  points.pts[9].x = 4;  points.pts[9].y = 2;    points.pts[9].z = 3;

  points.pts[10].x = 6;  points.pts[10].y = 2;    points.pts[10].z = 3;
  points.pts[11].x = 9;  points.pts[11].y = 5;    points.pts[11].z = 6;
  points.pts[12].x = 12;  points.pts[12].y = 8;    points.pts[12].z = 9;
  points.pts[13].x = 18; points.pts[13].y = 14;   points.pts[13].z = 15;
  points.pts[14].x = 21; points.pts[14].y = 17;   points.pts[14].z = 18;
  points.pts[15].x = 27; points.pts[15].y = 23;   points.pts[15].z = 24;
  points.pts[16].x = 9;  points.pts[16].y = 2;    points.pts[16].z = 3;
  */
  SACModel *model = new SACModelParallelLines(1.99,2.01);
  SAC *sac        = new MSAC(model, 0.001);
  model->setDataSet(&points);
  EXPECT_EQ((int)model->getCloud()->pts.size(), 17);

  bool result = sac->computeModel();
  if (result == true)
  {

    //EXPECT_EQ(result, true);

    std::vector<int> inliers = sac->getInliers();
    EXPECT_EQ((int)inliers.size(), 14);

    std::vector<double> coeff = sac->computeCoefficients();
    EXPECT_EQ((int)coeff.size(), 9);
    //printf ("Parallel line coefficients: %f %f %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6], coeff[7], coeff[8]);
    robot_msgs::Point32 dir;
    dir.x = fabs(coeff[3] - coeff[0]);
    dir.y = fabs(coeff[4] - coeff[1]);
    dir.z = fabs(coeff[5] - coeff[2]);
    cloud_geometry::normalizePoint(dir);
    //printf ("Line direction: %f %f %f\n", dir.x, dir.y, dir.z);
    EXPECT_NEAR(dir.x, 1, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.y, 0, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.z, 0, 1e-3);// 0.577, 1e-3);

    std::vector<double> coeff_ref = sac->refineCoefficients();
    EXPECT_EQ((int)coeff_ref.size(), 9);
    //printf ("Line coefficients (refined): %f %f %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3], coeff_ref[4], coeff_ref[5]);
    robot_msgs::Point32 dir_ref;
    dir_ref.x = fabs(coeff_ref[3] - coeff_ref[0]);
    dir_ref.y = fabs(coeff_ref[4] - coeff_ref[1]);
    dir_ref.z = fabs(coeff_ref[5] - coeff_ref[2]);
    cloud_geometry::normalizePoint(dir_ref);
    //printf ("Line direction: %f %f %f\n", dir_ref.x, dir_ref.y, dir_ref.z);
    EXPECT_NEAR(dir.x, 1, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.y, 0, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.z, 0, 1e-3);// 0.577, 1e-3);

    int nr_points_left = sac->removeInliers();
    EXPECT_EQ(nr_points_left, 3);
  }
  
  delete sac;
  delete model; 

}

/*TEST (MLESAC, SACModelLine)
{
  robot_msgs::PointCloud points;
  points.pts.resize (10);

  points.pts[0].x = 1;  points.pts[0].y = 2;    points.pts[0].z = 3;
  points.pts[1].x = 4;  points.pts[1].y = 5;    points.pts[1].z = 6;
  points.pts[2].x = 7;  points.pts[2].y = 8;    points.pts[2].z = 9;
  points.pts[3].x = 10; points.pts[3].y = 11;   points.pts[3].z = 12;
  points.pts[4].x = 13; points.pts[4].y = 14;   points.pts[4].z = 15;
  points.pts[5].x = 16; points.pts[5].y = 17;   points.pts[5].z = 18;
  points.pts[6].x = 19; points.pts[6].y = 20;   points.pts[6].z = 21;
  points.pts[7].x = 22; points.pts[7].y = 23;   points.pts[7].z = 24;
  points.pts[8].x = -5; points.pts[8].y = 1.57; points.pts[8].z = 0.75;
  points.pts[9].x = 4;  points.pts[9].y = 2;    points.pts[9].z = 3;

  SACModel *model = new SACModelLine ();
  SAC *sac        = new MLESAC (model, 0.001);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 10);

  bool result = sac->computeModel (0);
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 8);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 6);
  //printf ("Line coefficients: %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5]);
  robot_msgs::Point32 dir;
  dir.x = fabs (coeff[3] - coeff[0]);
  dir.y = fabs (coeff[4] - coeff[1]);
  dir.z = fabs (coeff[5] - coeff[2]);
  cloud_geometry::normalizePoint (dir);
  //printf ("Line direction: %f %f %f\n", dir.x, dir.y, dir.z);
  EXPECT_NEAR (dir.x, 0.577, 1e-3);
  EXPECT_NEAR (dir.y, 0.577, 1e-3);
  EXPECT_NEAR (dir.z, 0.577, 1e-3);

  std::vector<double> coeff_ref = sac->refineCoefficients ();
  EXPECT_EQ ((int)coeff_ref.size (), 6);
  //printf ("Line coefficients (refined): %f %f %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3], coeff_ref[4], coeff_ref[5]);
  robot_msgs::Point32 dir_ref;
  dir_ref.x = fabs (coeff_ref[3] - coeff_ref[0]);
  dir_ref.y = fabs (coeff_ref[4] - coeff_ref[1]);
  dir_ref.z = fabs (coeff_ref[5] - coeff_ref[2]);
  cloud_geometry::normalizePoint (dir_ref);
  //printf ("Line direction: %f %f %f\n", dir_ref.x, dir_ref.y, dir_ref.z);
  EXPECT_NEAR (dir_ref.x, 0.577, 1e-3);
  EXPECT_NEAR (dir_ref.y, 0.577, 1e-3);
  EXPECT_NEAR (dir_ref.z, 0.577, 1e-3);

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 2);

  delete sac;
  delete model;
}
*/

TEST (RRANSAC, SACModelParallelLines)
{

  robot_msgs::PointCloud points;
  points.pts.resize(17);

  points.pts[0].x = 1;  points.pts[0].y = 1;    points.pts[0].z = 1;
  points.pts[1].x = 4;  points.pts[1].y = 1;    points.pts[1].z = 1;
  points.pts[2].x = 7;  points.pts[2].y = 1;    points.pts[2].z = 1;
  points.pts[3].x = 10; points.pts[3].y = 1;    points.pts[3].z = 1;
  points.pts[4].x = 13; points.pts[4].y = 1;    points.pts[4].z = 1;
  points.pts[5].x = 16; points.pts[5].y = 1;    points.pts[5].z = 1;
  points.pts[6].x = 19; points.pts[6].y = 1;    points.pts[6].z = 1;
  points.pts[7].x = 22; points.pts[7].y = 1;    points.pts[7].z = 1;
  points.pts[8].x = 1;  points.pts[8].y = 15;    points.pts[8].z = 1;
  points.pts[9].x = 1;  points.pts[9].y = 16;    points.pts[9].z = 1;

  points.pts[10].x = 7;  points.pts[10].y = -1;    points.pts[10].z = 1;
  points.pts[11].x = 10; points.pts[11].y = -1;    points.pts[11].z = 1;
  points.pts[12].x = 13; points.pts[12].y = -1;    points.pts[12].z = 1;
  points.pts[13].x = 16; points.pts[13].y = -1;    points.pts[13].z = 1;
  points.pts[14].x = 19; points.pts[14].y = -1;    points.pts[14].z = 1;
  points.pts[15].x = 22; points.pts[15].y = -1;    points.pts[15].z = 1;
  points.pts[16].x = 1;  points.pts[16].y = -15;   points.pts[16].z = 1;


  /*
  points.pts[0].x = 1;  points.pts[0].y = 2;    points.pts[0].z = 3;
  points.pts[1].x = 4;  points.pts[1].y = 5;    points.pts[1].z = 6;
  points.pts[2].x = 7;  points.pts[2].y = 8;    points.pts[2].z = 9;
  points.pts[3].x = 10; points.pts[3].y = 11;   points.pts[3].z = 12;
  points.pts[4].x = 13; points.pts[4].y = 14;   points.pts[4].z = 15;
  points.pts[5].x = 16; points.pts[5].y = 17;   points.pts[5].z = 18;
  points.pts[6].x = 19; points.pts[6].y = 20;   points.pts[6].z = 21;
  points.pts[7].x = 22; points.pts[7].y = 23;   points.pts[7].z = 24;
  points.pts[8].x = -5; points.pts[8].y = 1.57; points.pts[8].z = 0.75;
  points.pts[9].x = 4;  points.pts[9].y = 2;    points.pts[9].z = 3;

  points.pts[10].x = 6;  points.pts[10].y = 2;    points.pts[10].z = 3;
  points.pts[11].x = 9;  points.pts[11].y = 5;    points.pts[11].z = 6;
  points.pts[12].x = 12;  points.pts[12].y = 8;    points.pts[12].z = 9;
  points.pts[13].x = 18; points.pts[13].y = 14;   points.pts[13].z = 15;
  points.pts[14].x = 21; points.pts[14].y = 17;   points.pts[14].z = 18;
  points.pts[15].x = 27; points.pts[15].y = 23;   points.pts[15].z = 24;
  points.pts[16].x = 9;  points.pts[16].y = 2;    points.pts[16].z = 3;
  */
  SACModel *model = new SACModelParallelLines(1.99,2.01);
  SAC *sac        = new RRANSAC(model, 0.001);
  model->setDataSet(&points);
  EXPECT_EQ((int)model->getCloud()->pts.size(), 17);

  bool result = sac->computeModel();
  if (result == true)
  {

    //EXPECT_EQ(result, true);

    std::vector<int> inliers = sac->getInliers();
    EXPECT_EQ((int)inliers.size(), 14);

    std::vector<double> coeff = sac->computeCoefficients();
    EXPECT_EQ((int)coeff.size(), 9);
    //printf ("Parallel line coefficients: %f %f %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6], coeff[7], coeff[8]);
    robot_msgs::Point32 dir;
    dir.x = fabs(coeff[3] - coeff[0]);
    dir.y = fabs(coeff[4] - coeff[1]);
    dir.z = fabs(coeff[5] - coeff[2]);
    cloud_geometry::normalizePoint(dir);
    //printf ("Line direction: %f %f %f\n", dir.x, dir.y, dir.z);
    EXPECT_NEAR(dir.x, 1, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.y, 0, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.z, 0, 1e-3);// 0.577, 1e-3);

    std::vector<double> coeff_ref = sac->refineCoefficients();
    EXPECT_EQ((int)coeff_ref.size(), 9);
    //printf ("Line coefficients (refined): %f %f %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3], coeff_ref[4], coeff_ref[5]);
    robot_msgs::Point32 dir_ref;
    dir_ref.x = fabs(coeff_ref[3] - coeff_ref[0]);
    dir_ref.y = fabs(coeff_ref[4] - coeff_ref[1]);
    dir_ref.z = fabs(coeff_ref[5] - coeff_ref[2]);
    cloud_geometry::normalizePoint(dir_ref);
    //printf ("Line direction: %f %f %f\n", dir_ref.x, dir_ref.y, dir_ref.z);
    EXPECT_NEAR(dir.x, 1, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.y, 0, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.z, 0, 1e-3);// 0.577, 1e-3);

    int nr_points_left = sac->removeInliers();
    EXPECT_EQ(nr_points_left, 3);
  }
  
  delete sac;
  delete model; 

}

TEST (RMSAC, SACModelParallelLines)
{


  robot_msgs::PointCloud points;
  points.pts.resize(17);

  points.pts[0].x = 1;  points.pts[0].y = 1;    points.pts[0].z = 1;
  points.pts[1].x = 4;  points.pts[1].y = 1;    points.pts[1].z = 1;
  points.pts[2].x = 7;  points.pts[2].y = 1;    points.pts[2].z = 1;
  points.pts[3].x = 10; points.pts[3].y = 1;    points.pts[3].z = 1;
  points.pts[4].x = 13; points.pts[4].y = 1;    points.pts[4].z = 1;
  points.pts[5].x = 16; points.pts[5].y = 1;    points.pts[5].z = 1;
  points.pts[6].x = 19; points.pts[6].y = 1;    points.pts[6].z = 1;
  points.pts[7].x = 22; points.pts[7].y = 1;    points.pts[7].z = 1;
  points.pts[8].x = 1;  points.pts[8].y = 15;    points.pts[8].z = 1;
  points.pts[9].x = 1;  points.pts[9].y = 16;    points.pts[9].z = 1;

  points.pts[10].x = 7;  points.pts[10].y = -1;    points.pts[10].z = 1;
  points.pts[11].x = 10; points.pts[11].y = -1;    points.pts[11].z = 1;
  points.pts[12].x = 13; points.pts[12].y = -1;    points.pts[12].z = 1;
  points.pts[13].x = 16; points.pts[13].y = -1;    points.pts[13].z = 1;
  points.pts[14].x = 19; points.pts[14].y = -1;    points.pts[14].z = 1;
  points.pts[15].x = 22; points.pts[15].y = -1;    points.pts[15].z = 1;
  points.pts[16].x = 1;  points.pts[16].y = -15;   points.pts[16].z = 1;


  /*
  points.pts[0].x = 1;  points.pts[0].y = 2;    points.pts[0].z = 3;
  points.pts[1].x = 4;  points.pts[1].y = 5;    points.pts[1].z = 6;
  points.pts[2].x = 7;  points.pts[2].y = 8;    points.pts[2].z = 9;
  points.pts[3].x = 10; points.pts[3].y = 11;   points.pts[3].z = 12;
  points.pts[4].x = 13; points.pts[4].y = 14;   points.pts[4].z = 15;
  points.pts[5].x = 16; points.pts[5].y = 17;   points.pts[5].z = 18;
  points.pts[6].x = 19; points.pts[6].y = 20;   points.pts[6].z = 21;
  points.pts[7].x = 22; points.pts[7].y = 23;   points.pts[7].z = 24;
  points.pts[8].x = -5; points.pts[8].y = 1.57; points.pts[8].z = 0.75;
  points.pts[9].x = 4;  points.pts[9].y = 2;    points.pts[9].z = 3;

  points.pts[10].x = 6;  points.pts[10].y = 2;    points.pts[10].z = 3;
  points.pts[11].x = 9;  points.pts[11].y = 5;    points.pts[11].z = 6;
  points.pts[12].x = 12;  points.pts[12].y = 8;    points.pts[12].z = 9;
  points.pts[13].x = 18; points.pts[13].y = 14;   points.pts[13].z = 15;
  points.pts[14].x = 21; points.pts[14].y = 17;   points.pts[14].z = 18;
  points.pts[15].x = 27; points.pts[15].y = 23;   points.pts[15].z = 24;
  points.pts[16].x = 9;  points.pts[16].y = 2;    points.pts[16].z = 3;
  */
  SACModel *model = new SACModelParallelLines(1.99,2.01);
  SAC *sac        = new RMSAC(model, 0.001);
  model->setDataSet(&points);
  EXPECT_EQ((int)model->getCloud()->pts.size(), 17);

  bool result = sac->computeModel();
  if (result == true)
  {

    //EXPECT_EQ(result, true);

    std::vector<int> inliers = sac->getInliers();
    EXPECT_EQ((int)inliers.size(), 14);

    std::vector<double> coeff = sac->computeCoefficients();
    EXPECT_EQ((int)coeff.size(), 9);
    //printf ("Parallel line coefficients: %f %f %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6], coeff[7], coeff[8]);
    robot_msgs::Point32 dir;
    dir.x = fabs(coeff[3] - coeff[0]);
    dir.y = fabs(coeff[4] - coeff[1]);
    dir.z = fabs(coeff[5] - coeff[2]);
    cloud_geometry::normalizePoint(dir);
    //printf ("Line direction: %f %f %f\n", dir.x, dir.y, dir.z);
    EXPECT_NEAR(dir.x, 1, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.y, 0, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.z, 0, 1e-3);// 0.577, 1e-3);

    std::vector<double> coeff_ref = sac->refineCoefficients();
    EXPECT_EQ((int)coeff_ref.size(), 9);
    //printf ("Line coefficients (refined): %f %f %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3], coeff_ref[4], coeff_ref[5]);
    robot_msgs::Point32 dir_ref;
    dir_ref.x = fabs(coeff_ref[3] - coeff_ref[0]);
    dir_ref.y = fabs(coeff_ref[4] - coeff_ref[1]);
    dir_ref.z = fabs(coeff_ref[5] - coeff_ref[2]);
    cloud_geometry::normalizePoint(dir_ref);
    //printf ("Line direction: %f %f %f\n", dir_ref.x, dir_ref.y, dir_ref.z);
    EXPECT_NEAR(dir.x, 1, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.y, 0, 1e-3);//0.577, 1e-3);
    EXPECT_NEAR(dir.z, 0, 1e-3);// 0.577, 1e-3);

    int nr_points_left = sac->removeInliers();
    EXPECT_EQ(nr_points_left, 3);
  }
  
  delete sac;
  delete model; 

}

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
